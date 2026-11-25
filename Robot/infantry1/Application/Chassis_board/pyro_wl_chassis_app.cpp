#include "pyro_can_drv.h"
#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_core.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_wl_chassis.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_bsp_can.h"
#include "pyro_dji_motor_drv.h"
#include "wl_config.h"
#include <cstdint>
#include "pyro_board_drv.h"

using namespace pyro;


constexpr uint32_t EVENT_BIT_RESTART   = (1 << 0);
constexpr uint32_t EVENT_BIT_STEP      = (1 << 1);
constexpr uint32_t EVENT_BIT_JUMP      = (1 << 2);
constexpr uint32_t EVENT_BIT_GETOVER_DETECTION = (1 << 3);






static TaskHandle_t chassis_task_handle           = nullptr;
static pyro::wl_chassis_t *wl_chassis_ptr         = nullptr;
static pyro::wl_chassis_cmd_t *wl_chassis_cmd_ptr = nullptr;
static pyro::wl_chassis_deps_t *wl_chassis_deps   = nullptr;
static pyro::board_drv_t *board_ptr               = nullptr;

static pyro::board_drv_t::g2c_data_t last_g2c_data;


static void gimbal_cmd();
static void chassis_dr162cmd(uint32_t notify);
static void deps_init();

extern "C"
{
    void infantry1_chassis_thread(void *argument)
    {
        while (true)
        {
            uint32_t notify_val = 0;
            // 接收任务通知事件（不阻塞等待，0 tick延时）
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);
            wl_chassis_cmd_ptr->jump_duo_enabled = false;

            if (board_ptr->check_online())
            {

                gimbal_cmd();

            }
            else if (dr16_drv_t::instance().check_online())
            {
                // 当前没有板间通信，直接检测并使用遥控器控制
                chassis_dr162cmd(notify_val);
            }
            else
            {
                wl_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
            }

            wl_chassis_ptr->set_command(*wl_chassis_cmd_ptr);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    void infantry1_chassis_init(void *argument)
    {
        vTaskDelay(10);

        wl_chassis_cmd_ptr = new pyro::wl_chassis_cmd_t();
        wl_chassis_ptr     = pyro::wl_chassis_t::instance();

        deps_init();
        wl_chassis_ptr->configure(*wl_chassis_deps);
        wl_chassis_ptr->start();

        board_ptr = &pyro::board_drv_t::get_instance(pyro::board_drv_t::role_t::CHASSIS,pyro::bsp_can::can3);


        xTaskCreate(infantry1_chassis_thread, "chassis_app_thread", 256,
                    nullptr, configMAX_PRIORITIES - 1, &chassis_task_handle);

        //订阅按键部分
        auto &vrc = pyro::rc_drv_t::read();
    pyro::sw_broker::subscribe(&vrc.switches.right, pyro::sw_event_t::MID_TO_UP,
                            chassis_task_handle, EVENT_BIT_RESTART);
    pyro::sw_broker::subscribe(&vrc.switches.right, pyro::sw_event_t::DOWN_TO_UP,
                            chassis_task_handle, EVENT_BIT_RESTART);
    // pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::MID_TO_UP,
    //                         chassis_task_handle, EVENT_BIT_STEP);
    pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::MID_TO_DOWN,
                            chassis_task_handle, EVENT_BIT_JUMP);
    pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::MID_TO_UP,
                            chassis_task_handle, EVENT_BIT_GETOVER_DETECTION);

        vTaskDelete(nullptr);
    }
}

void gimbal_cmd()
{
    //云台命令
    const auto& g2c_data = board_ptr->get_g2c_rx_data();
    wl_chassis_cmd_ptr->getover_detection_enabled = false;
    //下力模式
    if(g2c_data.mode == 0)
    {
        wl_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
        wl_chassis_cmd_ptr->cmd_function_state = pyro::chassis_function_state_t::NONE;
        wl_chassis_cmd_ptr->delta_leg_length[leg_def::L] = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_length[leg_def::R] = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_rad[leg_def::L]    = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_rad[leg_def::R]    = 0.0f;
        wl_chassis_cmd_ptr->v                            = 0.0f;
        wl_chassis_cmd_ptr->wz                           = 0.0f;
        memcpy(&last_g2c_data, &g2c_data, sizeof(pyro::board_drv_t::g2c_data_t));
        return;
    }
    if (g2c_data.mode == 1)
    {
        wl_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        wl_chassis_cmd_ptr->cmd_function_state = pyro::chassis_function_state_t::NONE;

        // 手动通道输入控制腿长和腿度（角度）的偏置量
        //用spining按键来切换左右腿
        static int which_leg = 0;
        if(g2c_data.spining)
        {
            which_leg = !which_leg;
        }
        wl_chassis_cmd_ptr->delta_leg_length[!which_leg]= 0;
        wl_chassis_cmd_ptr->delta_leg_rad[!which_leg]   = 0;
        wl_chassis_cmd_ptr->delta_leg_length[which_leg] = g2c_data.vx / 31.0f * 0.001f;
        wl_chassis_cmd_ptr->delta_leg_rad[which_leg]    = g2c_data.w  / 31.0f * 0.001f;
        wl_chassis_cmd_ptr->v                           = 0.0f;
        wl_chassis_cmd_ptr->wz                          = 0.0f;
        wl_chassis_cmd_ptr->cmd_continus_state          = pyro::chassis_active_state_t::MANUAL;
    }
    else if (g2c_data.mode == 2)//平衡模式
    {

        //平衡模式下的键位判断
        wl_chassis_cmd_ptr->cmd_function_state = pyro::chassis_function_state_t::NONE;
        if (g2c_data.step_mode == 1)
        {
            wl_chassis_cmd_ptr->cmd_function_state = pyro::chassis_function_state_t::STEP;
        }
        if (g2c_data.spining == 1)
        {
            wl_chassis_cmd_ptr->cmd_function_state       = pyro::chassis_function_state_t::JUMP;
        }
        if (g2c_data.rescue == 1)
        {
            wl_chassis_cmd_ptr->cmd_function_state       = pyro::chassis_function_state_t::RESCUE;
        }
        if (g2c_data.jump == 1)
        {
            wl_chassis_cmd_ptr->cmd_function_state       = pyro::chassis_function_state_t::JUMP;
        }

        wl_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        wl_chassis_cmd_ptr->delta_leg_length[leg_def::L] = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_rad[leg_def::L]    = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_length[leg_def::R] = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_rad[leg_def::R]    = 0.0f;
        wl_chassis_cmd_ptr->v                            = g2c_data.vx / 31.0f;
        wl_chassis_cmd_ptr->cmd_continus_state           = pyro::chassis_active_state_t::NORMAL;


        if(g2c_data.delta_leg == 0)
        {
            wl_chassis_cmd_ptr->dot_L                    = 0.0f;
        }
        else if(g2c_data.delta_leg == 1)
        {
            wl_chassis_cmd_ptr->dot_L                    = 0.3f;
        }
        else if(g2c_data.delta_leg == 2)
        {
            wl_chassis_cmd_ptr->dot_L                    = -0.3f;
        }
        if (last_g2c_data.mode != 2)
        {
            wl_chassis_cmd_ptr->cmd_function_state =
                pyro::chassis_function_state_t::RESTART;
        }

    }

    memcpy(&last_g2c_data, &g2c_data, sizeof(pyro::board_drv_t::g2c_data_t));
}

void chassis_dr162cmd(uint32_t notify)
{


    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    wl_chassis_cmd_ptr->jump_duo_enabled =
        vrc.switches.left.current_pos == pyro::sw_pos_t::UP;

    if (vrc.switches.left.current_pos != pyro::sw_pos_t::UP)
    {
        wl_chassis_cmd_ptr->getover_detection_enabled = false;
    }
    else if (notify & EVENT_BIT_GETOVER_DETECTION)
    {
        wl_chassis_cmd_ptr->getover_detection_enabled = true;
    }

    // 右开关控制底盘使能模式：不处于MID或DOWN时，失能
    if (pyro::sw_pos_t::DOWN == vrc.switches.right.current_pos)
    {
        wl_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
        wl_chassis_cmd_ptr->cmd_function_state = pyro::chassis_function_state_t::NONE;
        wl_chassis_cmd_ptr->delta_leg_length[leg_def::L] = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_length[leg_def::R] = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_rad[leg_def::L]    = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_rad[leg_def::R]    = 0.0f;
        wl_chassis_cmd_ptr->v                            = 0.0f;
        wl_chassis_cmd_ptr->wz                           = 0.0f;
        return;
    }

    //ACTIVE模式下的键位判断
    wl_chassis_cmd_ptr->cmd_function_state = pyro::chassis_function_state_t::NONE;
    if (notify & EVENT_BIT_RESTART)
    {
        wl_chassis_cmd_ptr->cmd_function_state = pyro::chassis_function_state_t::RESTART;
    }
    else if (notify & EVENT_BIT_STEP)
    {
        wl_chassis_cmd_ptr->cmd_function_state = pyro::chassis_function_state_t::STEP;
    }
    else if ((notify & EVENT_BIT_JUMP) &&
             pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
    {
        wl_chassis_cmd_ptr->cmd_function_state = pyro::chassis_function_state_t::JUMP;
    }
    if (pyro::sw_pos_t::MID == vrc.switches.right.current_pos)
    {
        wl_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;

        // 手动通道输入控制腿长和腿度（角度）的偏置量

        wl_chassis_cmd_ptr->delta_leg_length[leg_def::L] = vrc.axes.ly * 0.001f;
        wl_chassis_cmd_ptr->delta_leg_rad[leg_def::L]    = vrc.axes.lx * 0.001f;
        wl_chassis_cmd_ptr->delta_leg_length[leg_def::R] = vrc.axes.ry * 0.001f;
        wl_chassis_cmd_ptr->delta_leg_rad[leg_def::R]    = vrc.axes.rx * 0.001f;
        wl_chassis_cmd_ptr->v                            = 0.0f;
        wl_chassis_cmd_ptr->wz                           = 0.0f;
        wl_chassis_cmd_ptr->cmd_continus_state = pyro::chassis_active_state_t::MANUAL;
    }
    else if (pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
    {

        wl_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        wl_chassis_cmd_ptr->delta_leg_length[leg_def::L] = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_rad[leg_def::L]    = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_length[leg_def::R] = 0.0f;
        wl_chassis_cmd_ptr->delta_leg_rad[leg_def::R]    = 0.0f;
        wl_chassis_cmd_ptr->v                            = vrc.axes.ry * 1.3f;
        wl_chassis_cmd_ptr->wz                           = - vrc.axes.lx * 2.0f;
        wl_chassis_cmd_ptr->delta_leg_length[leg_def::L] =
            vrc.axes.ly * 0.0003f;
        wl_chassis_cmd_ptr->delta_leg_length[leg_def::R] =
            vrc.axes.ly * 0.0003f;
        wl_chassis_cmd_ptr->dot_L                        = vrc.axes.ly * 0.4f;
        wl_chassis_cmd_ptr->cmd_continus_state           = pyro::chassis_active_state_t::NORMAL;
    }
}

void deps_init()
{
    wl_chassis_deps = new pyro::wl_chassis_deps_t();

    // 1. 初始化二维数组形式的 4 个关节达妙电机。
    // 电机与链轮已交叉安装：原 knee 电机驱动 hip 链轮，
    // 原 hip 电机驱动 knee 链轮，因此此处按实际关节语义交换电机 ID。
    wl_chassis_deps->motor.joint[leg_def::L][joint_def::HIP] =
        new pyro::dm_motor_drv_t(0x03, 0x13, pyro::bsp_can::can2);
    wl_chassis_deps->motor.joint[leg_def::L][joint_def::KNEE] =
        new pyro::dm_motor_drv_t(0x04, 0x14, pyro::bsp_can::can2);
    wl_chassis_deps->motor.joint[leg_def::R][joint_def::HIP] =
        new pyro::dm_motor_drv_t(0x02, 0x12, pyro::bsp_can::can1);
    wl_chassis_deps->motor.joint[leg_def::R][joint_def::KNEE] =
        new pyro::dm_motor_drv_t(0x01, 0x11, pyro::bsp_can::can1);
    wl_chassis_deps->motor.wheel[leg_def::L] = new pyro::dji_m3508_motor_drv_t(
        pyro::dji_motor_tx_frame_t::id_3, pyro::bsp_can::can2);
    wl_chassis_deps->motor.wheel[leg_def::R] = new pyro::dji_m3508_motor_drv_t(
        pyro::dji_motor_tx_frame_t::id_2, pyro::bsp_can::can1);

    //登记yaw轴6020，获取反馈信息
    wl_chassis_deps->motor.yaw =
        new pyro::dji_gm_6020_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_5, pyro::bsp_can::can3,
            pyro::dji_gm_6020_motor_drv_t::CURRENT_MODE);

    static_cast<dm_motor_drv_t *>(
        wl_chassis_deps->motor.joint[leg_def::L][joint_def::HIP])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(
        wl_chassis_deps->motor.joint[leg_def::L][joint_def::HIP])
        ->set_rotate_range(-45.0f, 45.0f);
    static_cast<dm_motor_drv_t *>(
        wl_chassis_deps->motor.joint[leg_def::L][joint_def::HIP])
        ->set_torque_range(-54.0f, 54.0f);

    static_cast<dm_motor_drv_t *>(
        wl_chassis_deps->motor.joint[leg_def::L][joint_def::KNEE])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(
        wl_chassis_deps->motor.joint[leg_def::L][joint_def::KNEE])
        ->set_rotate_range(-45.0f, 45.0f);
    static_cast<dm_motor_drv_t *>(
        wl_chassis_deps->motor.joint[leg_def::L][joint_def::KNEE])
        ->set_torque_range(-54.0f, 54.0f);

    static_cast<dm_motor_drv_t *>(
        wl_chassis_deps->motor.joint[leg_def::R][joint_def::HIP])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(
        wl_chassis_deps->motor.joint[leg_def::R][joint_def::HIP])
        ->set_rotate_range(-45.0f, 45.0f);
    static_cast<dm_motor_drv_t *>(
        wl_chassis_deps->motor.joint[leg_def::R][joint_def::HIP])
        ->set_torque_range(-54.0f, 54.0f);

    static_cast<dm_motor_drv_t *>(
        wl_chassis_deps->motor.joint[leg_def::R][joint_def::KNEE])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(
        wl_chassis_deps->motor.joint[leg_def::R][joint_def::KNEE])
        ->set_rotate_range(-45.0f, 45.0f);
    static_cast<dm_motor_drv_t *>(
        wl_chassis_deps->motor.joint[leg_def::R][joint_def::KNEE])
        ->set_torque_range(-54.0f, 54.0f);


    // 2. Initialize conservative PD controllers for unloaded suspended testing.
    // Integral is disabled; only output and derivative filters are enabled.
    constexpr float OUTPUT_CUTOFF_HZ            = 20.0f;
    constexpr float DERIVATIVE_CUTOFF_HZ        = 10.0f;

    // Leg-length PD: output F_L (N), limited to 80 N.
    wl_chassis_deps->pid.leg_length[leg_def::L] = new pyro::pd_ctrl_t(
        1300.0f, 42.0f, 80.0f, OUTPUT_CUTOFF_HZ, 1, DERIVATIVE_CUTOFF_HZ, 1);
    wl_chassis_deps->pid.leg_length[leg_def::R] = new pyro::pd_ctrl_t(
        1300.0f, 42.0f, 80.0f, OUTPUT_CUTOFF_HZ, 1, DERIVATIVE_CUTOFF_HZ, 1);


    // Leg-angle PD: output T_p (N m), limited to 32 N m.

    //双环控制的腿角度
    wl_chassis_deps->pid.leg_control_rad[leg_def::R]   = new pyro::pid_t(60.0f, 0.0f, 0.0f, 0.0f, 15.0f);
    wl_chassis_deps->pid.leg_control_rad[leg_def::L]   = new pyro::pid_t(60.0f, 0.0f, 0.0f, 0.0f, 15.0f);
    wl_chassis_deps->pid.leg_control_radps[leg_def::R] = new pyro::pid_t(15.0f, 0.0f, 0.0f, 0.0f, 20.0f);
    wl_chassis_deps->pid.leg_control_radps[leg_def::L] = new pyro::pid_t(15.0f, 0.0f, 0.0f, 0.0f, 20.0f);

}
