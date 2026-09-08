#include "pyro_infantry2_booster.h"
#include "pyro_rc_base_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_module_base.h"
#include "pyro_dji_motor_drv.h"

using namespace pyro;

// 定义任务通知的位掩码 (Event Bits)
// VT03
constexpr uint32_t EVENT_BIT_FRIC_TOGGLE = (1 << 0);
constexpr uint32_t EVENT_BIT_SINGLE_FIRE = (1 << 1);
constexpr uint32_t EVENT_BIT_CONTINUE_FIRE = (1 << 2);
constexpr uint32_t EVENT_BIT_CONTINUE_END = (1 << 3);

//DR16
constexpr uint32_t EVENT_BIT_FRIC_ON = (1 << 4);
constexpr uint32_t EVENT_BIT_FRIC_OFF = (1 << 5);
constexpr uint32_t EVENT_BIT_FIRE = (1 << 6);
constexpr uint32_t EVENT_BIT_FIRE_END = (1 << 7);

infantry2_booster_cmd_t *booster_cmd_ptr = nullptr;
infantry2_booster_deps_t *booster_deps_ptr = nullptr;
infantry2_booster_t *booster_ptr = nullptr;

static TaskHandle_t booster_task_handle = nullptr;

void booster_deps_init() {
    // 摩擦轮电机初始化
    booster_deps_ptr->motor.fric[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, bsp_can::can2);
    booster_deps_ptr->motor.fric[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, bsp_can::can2);
    // 拨弹盘电机初始化
    booster_deps_ptr->motor.trigger = new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_3, bsp_can::can1);

    // 弹速pid初始化
    booster_deps_ptr->pid.bullet_spd_pid = new pid_t(0.0f, 0.0f, 0.0f, 1.0f, 20, 60.0f, 1, 15.0f, 1, 4);

    // 摩擦轮pid初始化
    booster_deps_ptr->pid.fric_pid[0] = new pid_t(0.3f, 0.0f, 0.0f, 1.0f, 20, 60.0f, 1, 15.0f, 1, 4);
    booster_deps_ptr->pid.fric_pid[1] = new pid_t(0.3f, 0.0f, 0.0f, 1.0f, 20, 60.0f, 1, 15.0f, 1, 4);

    // 拨弹盘pid初始化
    booster_deps_ptr->pid.trigger_pos_pid = new pid_t(60.0f, 0.5f, 0.0f, 3.0f, 20.0f);
    booster_deps_ptr->pid.trigger_spd_pid = new pid_t(10.0f, 0.02f, 0.0f, 5.0f, 10.0f);;
}

void booster_vt032cmd(uint32_t notify_val) {
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if(vrc.switches.gear.current_pos == pyro::sw_pos_t::UP ||
            vrc.switches.gear.current_pos == pyro::sw_pos_t::DOWN) {
        booster_cmd_ptr->mode = infantry2_booster_cmd_t::mode_t::PASSIVE;
        booster_cmd_ptr->is_fric_on = false;
        booster_cmd_ptr->fire_licence = false;
    }
    if(vrc.switches.gear.current_pos == pyro::sw_pos_t::MID) {
        booster_cmd_ptr->mode = infantry2_booster_cmd_t::mode_t::ACTIVE;
        booster_cmd_ptr->fire_licence = true; //没接热量管理，所以临时直接授予发射许可

        if(notify_val & EVENT_BIT_FRIC_TOGGLE)
            booster_cmd_ptr->is_fric_on = !booster_cmd_ptr->is_fric_on;
        
        if(notify_val & EVENT_BIT_SINGLE_FIRE) {
            booster_cmd_ptr->is_fric_on = true;
            booster_ptr->notify_single_shoot();
        }
        else if(notify_val & EVENT_BIT_CONTINUE_FIRE) {
            booster_cmd_ptr->is_fric_on = true;
            booster_cmd_ptr->continue_shoot =true;
        }

        if(notify_val & EVENT_BIT_CONTINUE_END)
            booster_cmd_ptr->continue_shoot = false;
    }
}

void booster_dr162cmd(uint32_t notify_val) {
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if(vrc.switches.right.current_pos == pyro::sw_pos_t::UP) {
        booster_cmd_ptr->mode = infantry2_booster_cmd_t::mode_t::PASSIVE;
        booster_cmd_ptr->is_fric_on = false;
        booster_cmd_ptr->fire_licence = false;
    }
    if(vrc.switches.right.current_pos == pyro::sw_pos_t::MID
            || vrc.switches.right.current_pos == pyro::sw_pos_t::DOWN) {
        booster_cmd_ptr->mode = infantry2_booster_cmd_t::mode_t::ACTIVE;
        booster_cmd_ptr->fire_licence = true; //没接热量管理，所以临时直接授予发射许可

        if(notify_val & EVENT_BIT_FRIC_ON)
            booster_cmd_ptr->is_fric_on = true;
        
        if(notify_val & EVENT_BIT_FRIC_OFF)
            booster_cmd_ptr->is_fric_on = false;

        if(notify_val & EVENT_BIT_FIRE) {
            booster_cmd_ptr->is_fric_on = true;
            booster_ptr->notify_single_shoot();
            booster_cmd_ptr->continue_shoot = false;
        }

        if (notify_val & EVENT_BIT_FIRE_END) {
            booster_cmd_ptr->continue_shoot = false;
        }

        if (fabs(vrc.axes.wheel) > 0.5f)
            booster_cmd_ptr->continue_shoot = true;
        else
            booster_cmd_ptr->continue_shoot = false;
    }
}

extern "C" {
    void infantry2_booster_thread(void *argument) {
        while(true) {
#if BOOSTER_EN
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

            if(vt03_drv_t::instance().check_online()) {
                booster_vt032cmd(notify_val);
            }
            else if(dr16_drv_t::instance().check_online()) {
                booster_dr162cmd(notify_val);
            }
            else
                booster_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;
#else
            booster_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;
#endif
            booster_ptr->set_command(*booster_cmd_ptr);

            vTaskDelay(1);
        }
    }

    void infantry2_booster_init(void *argument) {
        booster_cmd_ptr = new infantry2_booster_cmd_t();
        booster_deps_ptr = new infantry2_booster_deps_t();
        booster_ptr = infantry2_booster_t::instance();

        booster_deps_init();
        booster_ptr->configure(*booster_deps_ptr);
        booster_ptr->start();

        xTaskCreate(infantry2_booster_thread, "infantry2_booster_thread", 256, nullptr,
                    configMAX_PRIORITIES - 1, &booster_task_handle);
        
        auto &vrc = pyro::rc_drv_t::read();

        // --- VT03 按键绑定 ---
        pyro::btn_broker::subscribe(&vrc.buttons.fn_l, pyro::btn_event_t::PRESS_DOWN,
            booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::SINGLE_CLICK,
            booster_task_handle, EVENT_BIT_SINGLE_FIRE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::LONG_PRESS_START,
            booster_task_handle, EVENT_BIT_CONTINUE_FIRE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::PRESS_UP,
            booster_task_handle, EVENT_BIT_CONTINUE_END);

        // --- DR16 按键绑定 ---
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::UP_TO_MID,
            booster_task_handle, EVENT_BIT_FRIC_ON);
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::MID_TO_UP,
            booster_task_handle, EVENT_BIT_FRIC_OFF);
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::MID_TO_DOWN,
            booster_task_handle, EVENT_BIT_FIRE);
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::DOWN_TO_MID,
            booster_task_handle, EVENT_BIT_FIRE_END);
        
        vTaskDelete(nullptr);
    }
}