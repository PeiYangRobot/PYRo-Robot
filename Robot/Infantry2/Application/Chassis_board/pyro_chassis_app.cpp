#include "pyro_infantry2_chassis.h"
#include "pyro_bsp_can.h"
#include "pyro_board_comm.h"
#include "pyro_dji_motor_drv.h"

using namespace pyro;

infantry2_chassis_cmd_t *chassis_cmd_ptr = nullptr;
infantry2_chassis_deps_t *chassis_deps_ptr = nullptr;
infantry2_chassis_t *chassis_ptr = nullptr;

static TaskHandle_t chassis_task_handle = nullptr;

// 板间 CAN 接收 buffer (can3)
static pyro::can_msg_buffer_t chassis_rx(pyro::CHASSIS_CMD_ID);
static TickType_t last_cmd_tick = 0;

// 目标速度 (来自云台, 归一化 x 最大速度) 与 当前输出速度 (斜坡限幅后)
static float tgt_vx = 0.0f, tgt_vy = 0.0f, tgt_wz = 0.0f;
static float cur_vx = 0.0f, cur_vy = 0.0f, cur_wz = 0.0f;

void chassis_deps_init() {
    // ===== 电机 (按实际硬件核对型号 / CAN总线 / ID) =====
    // 舵电机
    chassis_deps_ptr->motor.rudder[0] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_3, bsp_can::can2); // FL
    chassis_deps_ptr->motor.rudder[1] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1, bsp_can::can2); // FR
    chassis_deps_ptr->motor.rudder[2] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_4, bsp_can::can1); // BR
    chassis_deps_ptr->motor.rudder[3] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2, bsp_can::can1); // BL

    // 轮电机
    chassis_deps_ptr->motor.wheel[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3, bsp_can::can2); // FL
    chassis_deps_ptr->motor.wheel[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, bsp_can::can2); // FR
    chassis_deps_ptr->motor.wheel[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4, bsp_can::can1); // BR
    chassis_deps_ptr->motor.wheel[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, bsp_can::can1); // BL

    // 注意: yaw 电机需占用独立帧槽位, 与上方 wheel/rudder 错开, 按实际接线填写
    chassis_deps_ptr->motor.yaw =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::register_id_t::id_5, bsp_can::can3);

    // ===== PID (按实际整定) =====
    for (int i = 0; i < 4; i++) {
        chassis_deps_ptr->pid.rud_pos_pid[i] =
            new pid_t(40.0f, 0.0f, 0.0f, 5.0f, 16.0f, 20.0f, 1, 0.0f, 0, 0);
        chassis_deps_ptr->pid.rud_spd_pid[i] =
            new pid_t(0.06f, 0.0f, 0.0f, 1.0f, 12.0f, 30.0f, 1, 0.0f, 0, 0);
        chassis_deps_ptr->pid.wheel_pid[i] =
            new pid_t(0.05f, 0.0f, 0.0f, 1.0f, 20.0f);
    }

    chassis_deps_ptr->pid.yaw_follow_pid =
        new pid_t(20.0f, 0.01f, 0.05f, 1.0f, 15.0f);
}

// 底盘板 <- 云台板: 接收遥控器指令, 心跳判定 -> 最大速度缩放 -> 斜坡限幅 -> 写 cmd
void chassis_rxcmd() {
    pyro::infantry2_chassis_rc_u u{};

    // 1) 收到新帧 -> 更新目标速度 (归一化 x 最大速度) 与 mode/state
    //    get_data() 不清除 fresh 标志, 必须 mark_read() 才能让掉线保护生效
    if (chassis_rx.get_data(u.data)) {
        chassis_rx.mark_read();
        last_cmd_tick = xTaskGetTickCount();

        tgt_vx = u.cmd.vx / (float)pyro::CHASSIS_RC_SCALE * infantry2_chassis::MAX_VX;
        tgt_vy = u.cmd.vy / (float)pyro::CHASSIS_RC_SCALE * infantry2_chassis::MAX_VY;
        tgt_wz = u.cmd.wz / (float)pyro::CHASSIS_RC_SCALE * infantry2_chassis::MAX_WZ;

        chassis_cmd_ptr->mode  = (u.cmd.mode == pyro::MODE_ACTIVE)
                                 ? cmd_base_t::mode_t::ACTIVE
                                 : cmd_base_t::mode_t::PASSIVE;
        chassis_cmd_ptr->state = (infantry2_chassis_cmd_t::state_t)u.cmd.state;
    }

    // 2) 掉线保护: 超过阈值未收到新帧 -> 目标清零并切 PASSIVE
    TickType_t now = xTaskGetTickCount();
    if ((now - last_cmd_tick) > pdMS_TO_TICKS(infantry2_chassis::LOST_TIMEOUT_MS)) {
        tgt_vx = tgt_vy = tgt_wz = 0.0f;
        chassis_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;
    }

    // 3) 加速度斜坡限幅 (防摇杆跳变急冲), 固定 1ms 控制周期
    const float dt = 0.001f;
    const float max_step = infantry2_chassis::MAX_ACCEL * dt;
    cur_vx = pyro::ramp_value(cur_vx, tgt_vx, max_step);
    cur_vy = pyro::ramp_value(cur_vy, tgt_vy, max_step);
    cur_wz = pyro::ramp_value(cur_wz, tgt_wz, max_step);

    chassis_cmd_ptr->vx = cur_vx;
    chassis_cmd_ptr->vy = cur_vy;
    chassis_cmd_ptr->wz = cur_wz;
}

extern "C" {
    void infantry2_chassis_thread(void *argument) {
        while (true) {

#if CHASSIS_EN
            chassis_rxcmd();
#else
            chassis_cmd_ptr->mode = infantry2_chassis_cmd_t::mode_t::PASSIVE;
            chassis_cmd_ptr->vx = 0;
            chassis_cmd_ptr->vy = 0;
            chassis_cmd_ptr->wz = 0;
#endif

            chassis_ptr->set_command(*chassis_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void infantry2_chassis_init(void *argument) {
        chassis_cmd_ptr = new infantry2_chassis_cmd_t();
        chassis_deps_ptr = new infantry2_chassis_deps_t();
        chassis_ptr = infantry2_chassis_t::instance();

        chassis_deps_init();
        chassis_ptr->configure(*chassis_deps_ptr);
        chassis_ptr->start();

        // 注册板间 CAN 接收 (can3)
        pyro::bsp_can::get_can3().register_rx_msg(&chassis_rx);

        xTaskCreate(infantry2_chassis_thread, "infantry2_chassis_thread", 256, nullptr,
                    configMAX_PRIORITIES - 1, &chassis_task_handle);
        vTaskDelete(nullptr);
    }
}
