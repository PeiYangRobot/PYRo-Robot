#include "pyro_can_drv.h"
#include "pyro_wl_booster.h"
#include "pyro_bsp_can.h"
#include "pyro_dji_motor_drv.h"
#include "booster_config.h"
#include "pyro_rw_lock.h"
#include "pyro_virtual_rc.h"
#include "pyro_rc_core.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_autoaim_drv.h"

using namespace pyro;


//发射机构部分
constexpr uint32_t EVENT_BIT_FRIC_TOGGLE              = (1 << 0);
constexpr uint32_t EVENT_BIT_SINGLE_FIRE              = (1 << 1);
constexpr uint32_t EVENT_BIT_BURST_FIRE               = (1 << 2);
constexpr uint32_t EVENT_BIT_BURST_END                = (1 << 3);
constexpr uint32_t EVENT_BIT_AUTOAIM_TOGGLE_BOOSTER   = (1 << 4);


static TaskHandle_t            booster_task_handle = nullptr;
static pyro::wl_booster_t      *wl_booster_ptr     = nullptr;
static pyro::wl_booster_cmd_t  *wl_booster_cmd_ptr = nullptr;
static pyro::wl_booster_deps_t *wl_booster_deps    = nullptr;
static pyro::autoaim_drv_t     *autoaim_ptr        = nullptr;

static virtual_rc_t vrc_t;

static void booster_deps_init();
static void booster_cmd(virtual_rc_t vrc, uint32_t notify);


extern "C"
{
    void wl_booster_thread(void *argument)
    {
        while(true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);
            if (vt03_drv_t::instance().check_online())
            {
                pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
                vrc_t = pyro::rc_drv_t::read();
                booster_cmd(vrc_t, notify_val);
            }

            wl_booster_ptr->set_command(*wl_booster_cmd_ptr);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    void infantry1_booster_init(void *argument)
    {


        wl_booster_cmd_ptr = new pyro::wl_booster_cmd_t();
        wl_booster_ptr     = pyro::wl_booster_t::instance();

        booster_deps_init();
        wl_booster_ptr->configure(*wl_booster_deps);
        wl_booster_ptr->start();

        autoaim_ptr = &pyro::autoaim_drv_t::get_instance();

        xTaskCreate(wl_booster_thread, "infantry_booster_thread", 256,
                    nullptr,configMAX_PRIORITIES - 1, &booster_task_handle);

        auto &vrc = rc_drv_t::read();

        //这里添加要订阅的按键
        pyro::btn_broker::subscribe(&vrc.buttons.fn_r, pyro::btn_event_t::SINGLE_CLICK,
                            booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::SINGLE_CLICK,
                            booster_task_handle, EVENT_BIT_SINGLE_FIRE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::LONG_PRESS_START,
                            booster_task_handle, EVENT_BIT_BURST_FIRE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::PRESS_UP,
                            booster_task_handle, EVENT_BIT_BURST_END);
        pyro::btn_broker::subscribe(&vrc.buttons.fn_l, pyro::btn_event_t::SINGLE_CLICK,
                            booster_task_handle, EVENT_BIT_AUTOAIM_TOGGLE_BOOSTER);


        vTaskDelete(nullptr);
    }
}


void booster_cmd(virtual_rc_t vrc, uint32_t notify)
{
    auto autoaim_cmd    = autoaim_ptr->get_target_data();


    if(vrc.switches.gear.current_pos == pyro::sw_pos_t::UP)
    {
        wl_booster_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;//Passive
    }
    else if(vrc.switches.gear.current_pos == pyro::sw_pos_t::MID ||
            vrc.switches.gear.current_pos == pyro::sw_pos_t::DOWN)
    {
        wl_booster_cmd_ptr->mode = cmd_base_t::mode_t::ACTIVE;//Active
    }


    static bool if_autoaim = false;
    if(notify & EVENT_BIT_AUTOAIM_TOGGLE_BOOSTER)
    {
        if_autoaim = !if_autoaim;
    }
    if(if_autoaim && autoaim_ptr->check_online())
    {
        wl_booster_cmd_ptr->event = ShootEvent::NONE;
        //自瞄
        if(autoaim_cmd.is_single_shot)
        {
            //自瞄单发逻辑处理
            //此时检测fireCommand的跳变沿来判断是否单发
            static int last_fireCommand = 0;
            if(last_fireCommand != autoaim_cmd.fireCommand)
            {
                wl_booster_cmd_ptr->event = ShootEvent::SINGLE_FIRE;
            }
            last_fireCommand = autoaim_cmd.fireCommand;
        }
        else
        {
            //自瞄连发逻辑处理
            //此时检测fireCommand是否为1来判断是否连发
            static int last_fireCommand = 0;
            if(last_fireCommand == 0 && autoaim_cmd.fireCommand == 1)
            {
                wl_booster_cmd_ptr->event = ShootEvent::BURST_START;
            }
            if(last_fireCommand == 0 && autoaim_cmd.fireCommand == 1)
            {
                wl_booster_cmd_ptr->event = ShootEvent::BURST_END;
            }
            last_fireCommand = autoaim_cmd.fireCommand;
        }
    }
    else
    {
        wl_booster_cmd_ptr->event = ShootEvent::NONE;
        //手动控制是否开火
        if(notify & EVENT_BIT_BURST_FIRE)
        {
            wl_booster_cmd_ptr->event = ShootEvent::BURST_START;
        }
        else if(notify & EVENT_BIT_SINGLE_FIRE)
        {
            wl_booster_cmd_ptr->event = ShootEvent::SINGLE_FIRE;
        }
        else if(notify & EVENT_BIT_BURST_END)
        {
            wl_booster_cmd_ptr->event = ShootEvent::BURST_END;
        }
    }

    //最高优先级
    if(notify & EVENT_BIT_FRIC_TOGGLE)
    {
        wl_booster_cmd_ptr->event = ShootEvent::FRIC_TOGGLE;
    }

}





void booster_deps_init()
{
    wl_booster_deps = new pyro::wl_booster_deps_t();

    // 初始化电机

    wl_booster_deps->motor_deps.fric1 =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,pyro::bsp_can::can2);
    wl_booster_deps->motor_deps.fric2 =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,pyro::bsp_can::can2);
    wl_booster_deps->motor_deps.trigger =
        new pyro::dji_m2006_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,pyro::bsp_can::can1);

    // 初始化串级 PID

    wl_booster_deps->pid_deps.fric1_spd =
        new pyro::pid_t(FRIC_SPEED_PID_KP, 0.0f, FRIC_SPEED_PID_KD, 0.0f, 20.0f);
    wl_booster_deps->pid_deps.fric2_spd =
        new pyro::pid_t(FRIC_SPEED_PID_KP,0.0f, FRIC_SPEED_PID_KD, 0.0f, 20.0f);
    wl_booster_deps->pid_deps.trigger_pos =
        new pyro::pid_t(TRIGGER_POS_PID_KP, 0.0f, TRIGGER_POS_PID_KD, 100.0f, 1000.0f);
    wl_booster_deps->pid_deps.trigger_spd =
        new pyro::pid_t(TRIGGER_SPEED_PID_KP, 0.0f,TRIGGER_SPEED_PID_KD, 5.0f, 10.0f);
    wl_booster_deps->speed_control_deps =
        new pyro::BulletSpeedControl(BULLET_SPEED_CONTROL_KP, BULLET_SPEED_CONTROL_KI, BULLET_SPEED_CONTROL_KD);
    wl_booster_deps->heat_control_deps =
        new HeatControl();
}
