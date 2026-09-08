/**
 * @file pyro_chassis_app.cpp
 * @brief 工程车底盘 Application 层
 *
 * 重构说明：去掉了 DataBoard 中转和 1ms 喂命令任务。
 * 板间通信（interboard_com）收到数据后直接调用 set_command()。
 * 本文件只负责：创建依赖 → 注入模块 → 启动模块。
 */
extern"C"{
    #include "FreeRTOS.h"
    #include "task.h"
}

#include <cstdint>

#include "pyro_engineer_chassis.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_algo_pid.h"
#include "pyro_bsp_can.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_module_base.h"

using namespace pyro;

/* ====================== 全局指针 ====================== */
static engineer_chassis_t *s_chassis = nullptr;
static engineer_deps_t    *s_deps    = nullptr;

/* ====================== PID 参数 ====================== */
static constexpr float MECANUM_PID_KP     = 0.3f;
static constexpr float MECANUM_PID_KI     = 0.5f;
static constexpr float MECANUM_PID_KD     = 0.0f;
static constexpr float MECANUM_PID_ILIMIT = 10.0f;
static constexpr float MECANUM_PID_MAXOUT = 15.0f;

/* ====================== 依赖初始化 ====================== */
static void deps_init()
{
    s_deps = new engineer_deps_t();

    /* ---------- 麦轮电机（旧工程车顺序：CAN1, id1~4） ---------- */
    // 数组索引：[0]FL前左, [1]FR前右, [2]BL后左, [3]BR后右
    // 对应 ID：  id_1       id_2        id_4        id_3
    s_deps->motor_deps.mecanum[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, bsp_can::can1); // FL
    s_deps->motor_deps.mecanum[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, bsp_can::can1); // FR
    s_deps->motor_deps.mecanum[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4, bsp_can::can1); // BL
    s_deps->motor_deps.mecanum[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3, bsp_can::can1); // BR

    /* ---------- 麦轮速度环 PID ---------- */
    s_deps->pid_deps.mecanum_pid[0] = new pid_t(MECANUM_PID_KP,MECANUM_PID_KI,MECANUM_PID_KD,MECANUM_PID_ILIMIT,MECANUM_PID_MAXOUT,pid_t::INTEGRAL_LIMIT);//FL
    s_deps->pid_deps.mecanum_pid[1] = new pid_t(MECANUM_PID_KP,MECANUM_PID_KI,MECANUM_PID_KD,MECANUM_PID_ILIMIT,MECANUM_PID_MAXOUT,pid_t::INTEGRAL_LIMIT);//FR
    s_deps->pid_deps.mecanum_pid[2] = new pid_t(MECANUM_PID_KP,MECANUM_PID_KI,MECANUM_PID_KD,MECANUM_PID_ILIMIT,MECANUM_PID_MAXOUT,pid_t::INTEGRAL_LIMIT);//BL
    s_deps->pid_deps.mecanum_pid[3] = new pid_t(MECANUM_PID_KP,MECANUM_PID_KI,MECANUM_PID_KD,MECANUM_PID_ILIMIT,MECANUM_PID_MAXOUT,pid_t::INTEGRAL_LIMIT);//BR

    /* ---------- 后摇臂电机 ---------- */
    s_deps->motor_deps.lift[0] = new dm_motor_drv_t(0x02,0x03,bsp_can::can2);
    s_deps->motor_deps.lift[1] = new dm_motor_drv_t(0x00,0x01,bsp_can::can2);

    // 设置达秒电机的 range 范围
    static_cast<dm_motor_drv_t*>(s_deps->motor_deps.lift[0])->set_position_range(-PI,PI);
    static_cast<dm_motor_drv_t*>(s_deps->motor_deps.lift[0])->set_rotate_range(-52.0f,52.0f);
    static_cast<dm_motor_drv_t*>(s_deps->motor_deps.lift[0])->set_torque_range(-27.0f,27.0f);

    static_cast<dm_motor_drv_t*>(s_deps->motor_deps.lift[1])->set_position_range(-PI,PI);
    static_cast<dm_motor_drv_t*>(s_deps->motor_deps.lift[1])->set_rotate_range(-52.0f,52.0f);
    static_cast<dm_motor_drv_t*>(s_deps->motor_deps.lift[1])->set_torque_range(-27.0f,27.0f);

    s_deps->pid_deps.lift_pos_pid[0] = new pid_t(14.0f, 0.005f, 0.0012f, 0.5f, 52.0f, pid_t::INTEGRAL_LIMIT);
    s_deps->pid_deps.lift_pos_pid[1] = new pid_t(14.0f, 0.005f, 0.0012f, 0.5f, 52.0f, pid_t::INTEGRAL_LIMIT);
    s_deps->pid_deps.lift_vel_pid[0] = new pid_t(2.0f, 0.005f, 0.0012f, 0.5f, 27.0f, pid_t::INTEGRAL_LIMIT);
    s_deps->pid_deps.lift_vel_pid[1] = new pid_t(2.0f, 0.005f, 0.0012f, 0.5f, 27.0f, pid_t::INTEGRAL_LIMIT);

    /* ---------- 矿仓电机 ---------- */
    s_deps->motor_deps.magazine = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1,bsp_can::can3);
    s_deps->pid_deps.magazine_pos_pid = new pid_t(10.0f,0.01f,0.3f,3.0f,10.0f,pid_t::INTEGRAL_LIMIT);
    s_deps->pid_deps.magazine_vel_pid = new pid_t(0.5f, 0.2f, 0.00f, 1.0f, 3.0f,pid_t::INTEGRAL_LIMIT);
}

/* ====================== 对外初始化入口 ====================== */
extern "C" void chassis_app_init()
{
    // 1. 创建所有依赖（电机、PID）
    deps_init();

    // 2. 获取底盘单例
    s_chassis = engineer_chassis_t::instance();

    // 3. 注入依赖
    s_chassis->configure(*s_deps);

    // 4. 启动模块（内部调用 _init → 创建模块控制任务）
    s_chassis->start();

    // 注意：不再创建 1ms 喂命令任务。
    // 板间通信任务（interboard_com）收到数据后直接调用 set_command()。
}
