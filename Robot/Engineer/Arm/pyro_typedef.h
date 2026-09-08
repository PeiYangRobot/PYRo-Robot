#pragma once
#include <stdint.h>
#include "pyro_databoard.h"
#define PYRO_ARM_JOINT_NUM 6

// ==================== 控制模式枚举 ====================
// 描述：计算任务当前所处的控制模式
typedef enum {
    PYRO_ARM_MODE_IDLE = 0,          // 零力/空闲（不下发力矩或保持当前位置）
    PYRO_ARM_MODE_JOINT_DIRECT,      // 直接关节映射（遥控器直通角度）
    PYRO_ARM_MODE_CARTESIAN,         // 笛卡尔坐标控制（需逆解算）
    PYRO_ARM_MODE_AUTO_SEQUENCE,     // 一键/自动序列（预留）
    PYRO_ARM_MODE_MAX
} PYRO_ArmControlMode_t;



// ==================== 执行层目标指令 ====================
// 描述：计算任务最终产出，写入 DataBoard，供执行任务读取
// 注意：此结构体仅描述“关节空间的目标值”，不关心上层是怎么算出来的
typedef struct {
    float joint[6];             //六个关节目标角度 (单位：rad，范围需在机械限位内)
    float gripper;              // 夹爪
} PYRO_ArmJointCommand_t;




// ==================== 关节直通模式输入数据 ====================
typedef struct {
    float joint[6];             // 六个关节目标角度
    float gripper;              // 夹爪
} PYRO_JointDirectInput_t;

// ==================== 笛卡尔坐标模式输入数据 ====================
typedef struct {
    float position[3];          // 笛卡尔空间位置 [X, Y, Z] 
    float orientation[3];       // 笛卡尔空间姿态 [Roll, Pitch, Yaw] 
    float gripper;              // 夹爪
} PYRO_CartesianInput_t;




// ==================== 计算任务顶层输入指令 ====================
// 描述：数据接收任务产出，写入 DataBoard ID=1，供计算任务读取
typedef struct {
    PYRO_ArmControlMode_t mode;      // 当前控制模式
    PYRO_JointDirectInput_t joint_direct;   // mode = JOINT_DIRECT 时使用
    PYRO_CartesianInput_t cartesian;        // mode = CARTESIAN 时使用
    //后面待扩展
} PYRO_ArmControlSignal_t;

extern pyro::databoard global_databoard;