#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"

#include "pyro_sentry_gimbal.h"

#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_motor_base.h"
#include "pyro_can_drv.h"
#include "pyro_bsp_can.h"
#include "pyro_board_comm.h"

using namespace pyro;

constexpr uint32_t EVENT_BIT_SPINNING = (1 << 0);

static TaskHandle_t gimbal_task_handle                        = nullptr;
static pyro::sentry_gimbal_t *sentry_gimbal_ptr                = nullptr;
static pyro::sentry_gimbal_cmd_t *gimbal_cmd_ptr               = nullptr;
static pyro::sentry_gimbal_deps_t *gimbal_deps_ptr             = nullptr;

extern "C" {

void deps_init()
{
    gimbal_deps_ptr = new pyro::sentry_gimbal_deps_t();

    gimbal_deps_ptr->motor_deps.motor_yaw = new pyro::dji_gm_6020_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                    pyro::bsp_can::can1);
    gimbal_deps_ptr->motor_deps.motor_pitch = new pyro::dm_motor_drv_t(0x1, 0x0, bsp_can::can2);

    static_cast<dm_motor_drv_t *>(gimbal_deps_ptr->motor_deps.motor_pitch)->set_position_range(-PI , PI);

    static_cast<dm_motor_drv_t *>(gimbal_deps_ptr->motor_deps.motor_pitch)->set_rotate_range(-20, 20);

    static_cast<dm_motor_drv_t *>(gimbal_deps_ptr->motor_deps.motor_pitch)->set_torque_range(-10, 10);

    gimbal_deps_ptr->pitch_max_rad     = -0.11f; // 最高的时候
    gimbal_deps_ptr->pitch_min_rad     = -0.34f; // 最低的时候
    gimbal_deps_ptr->yaw_max_rad       = 0.70f;
    gimbal_deps_ptr->yaw_min_rad       = -0.70f;

    gimbal_deps_ptr->pid_deps.yaw_pos_pid = new pyro::pid_t(6.0f, 0.0f, 0.0f,0,4);
    gimbal_deps_ptr->pid_deps.yaw_spd_pid = new pyro::pid_t(1.0f, 0.0f, 0.0f,0,10);

    gimbal_deps_ptr->pid_deps.pitch_pos_pid = new pyro::pid_t(15.0f, 1.2f, 0.0f,1.0f,9);
    gimbal_deps_ptr->pid_deps.pitch_spd_pid = new pyro::pid_t(3.2f, 0.45f, 0.003f,2,9);
}

void gimbal_dr162cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
    {
        gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->delta_pitch = 0;
        gimbal_cmd_ptr->delta_yaw   = 0;
        return;
    }
    gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::ACTIVE;
    gimbal_cmd_ptr->delta_pitch = -vrc.axes.ry * 0.0005f;
    gimbal_cmd_ptr->delta_yaw   = 0* (-vrc.axes.rx * 0.001f);
}

void gimbal_dr162chassis_cmd(uint32_t notify_value)
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    static int8_t vx        = 0;
    static int8_t vy        = 0;
    static int8_t wz        = 0;
    static int8_t delta_yaw = 0;
    static bool active      = false;
    static bool follow_en   = false;
    static bool spinning    = false;

    g2c_msg_t msg{};

    if (pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
    {
        active              = 0;
        vx                  = 0;
        vy                  = 0;
        wz                  = 0;
        delta_yaw           = 0;
        follow_en           = false;
        spinning            = false;

        msg.vx        = 0;
        msg.vy        = 0;
        msg.delta_yaw = 0;
        msg.flags     = 0;

        pyro::board_comm_t::instance().send(msg);

        return;
    }

    active              = 1;
    vx     = static_cast<int8_t>(-(vrc.axes.ly) * 127);
    vy     = static_cast<int8_t>(vrc.axes.lx * 127);
    wz     = 0;
    delta_yaw = static_cast<int8_t>(vrc.axes.rx * 127);

    if(abs(vx) < 5)vx = 0;
    if(abs(vy) < 5)vy = 0;
    if(abs(wz) < 5)wz = 0;
    if(abs(delta_yaw) < 5)delta_yaw = 0;

    static uint16_t spinning_count = 0;

    if(pyro::sw_pos_t::DOWN == vrc.switches.right.current_pos){
        if(spinning_count > 1000)
        {
            spinning = true;
            follow_en = true; //依赖底盘写法  谨慎调整！！！
        }
        else {spinning_count++;}
    }

    if(EVENT_BIT_SPINNING & notify_value)
    {
        spinning = false;
        spinning_count = 0;
        follow_en =  !follow_en;
    }

    msg.vx        = vx;
    msg.vy        = vy;
    msg.delta_yaw = delta_yaw;
    msg.flags     = static_cast<uint8_t>((active<<0)|(follow_en<<1)|(spinning<<2));

    pyro::board_comm_t::instance().send(msg);
}

static void chassis2gimbal_rx()
{
    c2g_msg_t msg;
    if (pyro::board_comm_t::instance().read(msg))
    {
        // 4字节预留数据，格式待定
        gimbal_cmd_ptr->chassis_data[0] = msg.data[0];
        gimbal_cmd_ptr->chassis_data[1] = msg.data[1];
        gimbal_cmd_ptr->chassis_data[2] = msg.data[2];
        gimbal_cmd_ptr->chassis_data[3] = msg.data[3];
    }
#if BOARD_COMM_TIMEOUT_C2G_ENABLE
    else if (pyro::board_comm_t::instance().is_stale<c2g_msg_t>(BOARD_COMM_TIMEOUT_C2G_MS))
    {
        // 底盘失联：数据清零
        gimbal_cmd_ptr->chassis_data[0] = 0;
        gimbal_cmd_ptr->chassis_data[1] = 0;
        gimbal_cmd_ptr->chassis_data[2] = 0;
        gimbal_cmd_ptr->chassis_data[3] = 0;
    }
#endif
}

void sentry_gimbal_thread(void *argument)
{
    while (true)
    {
        uint32_t notify_val = 0;
        xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

        // 如果后续希望由底盘板直接解算 RC，可以取消下面这行的注释
         gimbal_dr162cmd();
         gimbal_dr162chassis_cmd(notify_val);
         chassis2gimbal_rx();

        sentry_gimbal_ptr->set_command(*gimbal_cmd_ptr);
        vTaskDelay(1);
    }
}

void sentry_gimbal_init(void)
{
    gimbal_cmd_ptr     = new pyro::sentry_gimbal_cmd_t();
    sentry_gimbal_ptr = pyro::sentry_gimbal_t::instance();

    deps_init();
    sentry_gimbal_ptr->configure(*gimbal_deps_ptr);
    sentry_gimbal_ptr->start();

    xTaskCreate(sentry_gimbal_thread, "start_sentry_gimbal_thread", 256,
                nullptr, configMAX_PRIORITIES - 1, &gimbal_task_handle);
    auto &vrc = pyro::rc_drv_t::read();
    pyro::sw_broker::subscribe(&vrc.switches.right, pyro::sw_event_t::DOWN_TO_MID, gimbal_task_handle, EVENT_BIT_SPINNING);
    vTaskDelete(nullptr);
}

}
