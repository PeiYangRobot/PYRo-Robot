#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"

#include "pyro_rudder_chassis.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_motor_base.h"
#include "pyro_bsp_can.h"
#include "pyro_can_drv.h"
#include "pyro_board_comm.h"

using namespace pyro;


static pyro::rudder_chassis_t *rudder_chassis_ptr       = nullptr;
static pyro::rudder_cmd_t *rudder_cmd_ptr               = nullptr;
static pyro::rudder_deps_t *rudder_deps_ptr             = nullptr;

float test_imu11;


extern "C"
{


static void deps_init();
static void chassis_rxcmd();
static void chassis_dr162cmd();
static void imu2chassis();

void imu2chassis()
{
    imu2chassis_msg_t msg;
    if (pyro::board_comm_t::instance().read(msg))
    {
        if (msg.yaw_deg == 0)
            return;

        rudder_cmd_ptr->imu_yaw_rad   = msg.yaw_deg / 180 * PI;
        rudder_cmd_ptr->imu_yaw_radps = msg.yaw_radps;

        test_imu11 = rudder_cmd_ptr->imu_yaw_rad;
    }
#if BOARD_COMM_TIMEOUT_IMU_ENABLE
    else if (pyro::board_comm_t::instance().is_stale<imu2chassis_msg_t>(BOARD_COMM_TIMEOUT_IMU_MS))
    {
        // IMU 失联：航向不可信，清零
        rudder_cmd_ptr->imu_yaw_rad   = 0.0f;
        rudder_cmd_ptr->imu_yaw_radps = 0.0f;
    }
#endif
}

void deps_init()
{
    rudder_deps_ptr = new pyro::rudder_deps_t();
    rudder_deps_ptr->motor_deps.rudder[rudder_kin_t::FL] =
        new pyro::dji_gm_6020_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                        pyro::bsp_can::can2); // FL Rudder
    rudder_deps_ptr->motor_deps.rudder[rudder_kin_t::BL] =
        new pyro::dji_gm_6020_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,
                                        pyro::bsp_can::can2); // BL Rudder
    rudder_deps_ptr->motor_deps.rudder[rudder_kin_t::BR] =
        new pyro::dji_gm_6020_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,
                                        pyro::bsp_can::can1); // BR Rudder
    rudder_deps_ptr->motor_deps.rudder[rudder_kin_t::FR] =
        new pyro::dji_gm_6020_motor_drv_t(pyro::dji_motor_tx_frame_t::id_4,
                                        pyro::bsp_can::can1); // FR Rudder

    rudder_deps_ptr->motor_deps.wheel[rudder_kin_t::FL] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                          pyro::bsp_can::can2); // FL Wheel
    rudder_deps_ptr->motor_deps.wheel[rudder_kin_t::BL] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,
                                          pyro::bsp_can::can2); // BL Wheel
    rudder_deps_ptr->motor_deps.wheel[rudder_kin_t::BR] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,
                                          pyro::bsp_can::can1); // BR Wheel
    rudder_deps_ptr->motor_deps.wheel[rudder_kin_t::FR] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_4,
                                          pyro::bsp_can::can1); // FR Wheel

    rudder_deps_ptr->motor_deps.yaw = new pyro::dm_motor_drv_t(0x8, 0x9, bsp_can::can2);

    static_cast<dm_motor_drv_t *>(rudder_deps_ptr->motor_deps.yaw)
        ->set_position_range(-PI, PI);

    static_cast<dm_motor_drv_t *>(rudder_deps_ptr->motor_deps.yaw)
        ->set_rotate_range(-20, 20);
    static_cast<dm_motor_drv_t *>(rudder_deps_ptr->motor_deps.yaw)
        ->set_torque_range(-12.5, 12.5);



    rudder_deps_ptr->pid_deps.yaw_pos_pid =
        new pid_t(11.0f, 0.0f, 0.00f, 0.0f, 20.0f);
    // rudder_deps_ptr->pid_deps.yaw_spd_pid =565
    //     new pid_t(0.3f, 0.0f, 0.7f, 0.0f, 10.0f);
    rudder_deps_ptr->pid_deps.yaw_spd_pid =
        new pid_t(2.0f, 0.0f, 0.025f, 0.0f, 10.0f);


    // rudder_deps_ptr->pid_deps.yaw_pos_pid =
    //     new pid_t(8, 0, 1.6f, 1, 40);
    // rudder_deps_ptr->pid_deps.yaw_spd_pid =
    //     new pid_t(1.0, 0, 0.0f, 2, 12);

    // rudder_deps_ptr->pid_deps.yaw_pos_pid =
    //     new pid_t(5, 0, 0, 1, 10);
    // rudder_deps_ptr->pid_deps.yaw_spd_pid =
    //     new pid_t(0.8f, 0.01f, 0, 2, 10);



    rudder_deps_ptr->pid_deps.wheel_pid[0] =
        new pid_t(15.0f     -1  , 0.0f, 0.00f, 0.00f, 20.0f);
    rudder_deps_ptr->pid_deps.wheel_pid[1] =
        new pid_t(15.0f     -1  , 0.0f, 0.00f, 0.00f, 20.0f);
    rudder_deps_ptr->pid_deps.wheel_pid[2] =
         new pid_t(15.0f     -1  , 0.0f, 0.00f, 0.00f, 20.0f);
    rudder_deps_ptr->pid_deps.wheel_pid[3] =
         new pid_t(15.0f     -1  , 0.0f, 0.00f, 0.00f, 20.0f);


    rudder_deps_ptr->pid_deps.follow_yaw_pid =
        new pid_t(7.2f     , 0.0f, 0.1, 0, 10.0f);


    rudder_deps_ptr->pid_deps.rudder_pos_pid[0] =
        new pid_t(22.0f    , 0.0f, 0.00f, 0.0f, 16.0f);
    rudder_deps_ptr->pid_deps.rudder_pos_pid[1] =
        new pid_t(22.0f    , 0.0f, 0.00f, 0.0f, 16.0f);
    rudder_deps_ptr->pid_deps.rudder_pos_pid[2] =
        new pid_t(22.0f    , 0.0f, 0.00f, 0.0f, 16.0f);
    rudder_deps_ptr->pid_deps.rudder_pos_pid[3] =
        new pid_t(22.0f    , 0.0f, 0.00f, 0.0f, 16.0f);

    rudder_deps_ptr->pid_deps.rudder_spd_pid[0] =
        new pid_t(0.13f, 0.0f, 0.00f, 0.0f, 3.0f);
    rudder_deps_ptr->pid_deps.rudder_spd_pid[1] =
        new pid_t(0.13f, 0.0f, 0.00f, 0.0f, 3.0f);
    rudder_deps_ptr->pid_deps.rudder_spd_pid[2] =
        new pid_t(0.13f, 0.0f, 0.00f, 0.0f, 3.0f);
    rudder_deps_ptr->pid_deps.rudder_spd_pid[3] =
        new pid_t(0.13f, 0.0f, 0.00f, 0.0f, 3.0f);
}

void chassis_rxcmd()
{
    g2c_msg_t msg;
    if (pyro::board_comm_t::instance().read(msg))
    {
        rudder_cmd_ptr->vx =
            2.0f * static_cast<float>(msg.vx) / 127.0f;
        rudder_cmd_ptr->vy =
            2.0f * static_cast<float>(msg.vy) / 127.0f;
        rudder_cmd_ptr->delta_yaw =
            -0.004f * static_cast<float>(msg.delta_yaw) / 127.0f;

        rudder_cmd_ptr->mode =
            msg.active() ? pyro::cmd_base_t::mode_t::ACTIVE : pyro::cmd_base_t::mode_t::PASSIVE;
        rudder_cmd_ptr->follow_yaw = msg.follow_en();
        rudder_cmd_ptr->spinning   = msg.spinning();

        if (rudder_cmd_ptr->spinning == true)
        {
            rudder_cmd_ptr->follow_yaw = false;
        }
    }
#if BOARD_COMM_TIMEOUT_G2C_ENABLE
    else if (pyro::board_comm_t::instance().is_stale<g2c_msg_t>(BOARD_COMM_TIMEOUT_G2C_MS))
    {
        // 云台失联：停车并切被动
        rudder_cmd_ptr->vx         = 0.0f;
        rudder_cmd_ptr->vy         = 0.0f;
        rudder_cmd_ptr->wz         = 0.0f;
        rudder_cmd_ptr->delta_yaw  = 0.0f;
        rudder_cmd_ptr->mode       = pyro::cmd_base_t::mode_t::PASSIVE;
        rudder_cmd_ptr->follow_yaw = false;
        rudder_cmd_ptr->spinning   = false;
    }
#endif
}

void chassis_dr162cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::MID != vrc.switches.right.current_pos)
    {
        rudder_cmd_ptr->vx          = 0;
        rudder_cmd_ptr->vy          = 0;
        rudder_cmd_ptr->wz          = 0;
        rudder_cmd_ptr->delta_yaw   = 0;
        rudder_cmd_ptr->mode        = pyro::cmd_base_t::mode_t::PASSIVE;
    }
    else
    {
        rudder_cmd_ptr->vx          = 2.0f * vrc.axes.ly;
        rudder_cmd_ptr->vy          = -2.0f * vrc.axes.lx;
        rudder_cmd_ptr->wz          = 0.002f * vrc.axes.ry;
        rudder_cmd_ptr->delta_yaw   = -0.003f * vrc.axes.rx;
        rudder_cmd_ptr->mode        = pyro::cmd_base_t::mode_t::ACTIVE;

        if (pyro::sw_pos_t::DOWN == vrc.switches.right.current_pos)
        {
            rudder_cmd_ptr->follow_yaw = true;
        }
        else
        {
            rudder_cmd_ptr->follow_yaw = false;
        }
    }
}

void sentry_chassis_thread(void *argument)
{
    while (true)
    {
        chassis_rxcmd();
        // 如果后续希望由底盘板直接解算 RC，可以取消下面这行的注释
        //chassis_dr162cmd();
        imu2chassis();
        rudder_chassis_ptr->set_command(*rudder_cmd_ptr);
        vTaskDelay(1);
    }
}

void sentry_chassis_init(void *argument)
{
    rudder_cmd_ptr     = new pyro::rudder_cmd_t();
    rudder_chassis_ptr = pyro::rudder_chassis_t::instance();

    deps_init();
    rudder_chassis_ptr->configure(*rudder_deps_ptr);
    rudder_chassis_ptr->start();

    xTaskCreate(sentry_chassis_thread, "start_sentry_chassis_thread", 512,
                nullptr, configMAX_PRIORITIES - 1, nullptr);

    vTaskDelete(nullptr);
}

}
