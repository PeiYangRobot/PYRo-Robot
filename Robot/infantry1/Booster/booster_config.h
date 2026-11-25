#ifndef PYRO_ROBOT_BOOSTER_CONFIG_H
#define PYRO_ROBOT_BOOSTER_CONFIG_H

#include "pyro_core_def.h"
#include "stdint.h"

namespace pyro
{

//拨弹盘速度环pid参数
constexpr float TRIGGER_SPEED_PID_KP = 0.16f;
constexpr float TRIGGER_SPEED_PID_KD = 0.0f;

//拨弹盘位置环pid参数
constexpr float TRIGGER_POS_PID_KP = 2300.0f;
constexpr float TRIGGER_POS_PID_KD = 0.0002f;

//摩擦轮速度环pid参数
constexpr float FRIC_SPEED_PID_KP = 0.3f;
constexpr float FRIC_SPEED_PID_KD = 0.00002f;


// 弹丸初速度
constexpr float PROJECTILE_TARGET_MUZZLE_VELOCITY                 = 23.5f;

// 弹速调整系数
constexpr float FRIC_ADJUST_K                                     = 0.81f;
// 摩擦轮半径
constexpr float FRIC_RADIUS                                       = 0.03f;
//摩擦轮速度
constexpr float FRIC_TARGET_SPEED = PROJECTILE_TARGET_MUZZLE_VELOCITY / FRIC_RADIUS * FRIC_ADJUST_K;


// 发射速度 (发/秒)
constexpr float SHOOT_SPEED                                       = 10.0f;

// 拨弹盘电机减速比
constexpr int TRIGGER_MOTOR_REDUCTION_RATIO                       = 36;
// 拨弹盘一圈可以装几发弹
constexpr float HOLDING_BULLET_NUM                                = 8.0f;
// 拨弹盘速度
constexpr float TRIGGER_SPEED        = SHOOT_SPEED / HOLDING_BULLET_NUM * 2 * PI * TRIGGER_MOTOR_REDUCTION_RATIO;
//一发弹丸对应的电机需要转的弧度
constexpr float ONE_BULLET_RAD       = 2 * PI * TRIGGER_MOTOR_REDUCTION_RATIO / HOLDING_BULLET_NUM;
//拨弹盘一圈对应的电机需要转过的弧度
constexpr float ONE_CIRCLE_RAD       = 2 * PI * TRIGGER_MOTOR_REDUCTION_RATIO;






//弹速控制

#define BULLET_SPEED_CONTROL 1                 // 开关

constexpr float BULLET_SPEED_CONTROL_KP = 1.0f;
constexpr float BULLET_SPEED_CONTROL_KI = 0.0f;
constexpr float BULLET_SPEED_CONTROL_KD = 0.0f;
//弹速补偿最小有效速度
constexpr float VALID_MIN_SPEED = 17.0f;




//热量控制
#define HEAT_CONTROL 1                         // 开关
//预留的安全弹药空间
constexpr uint8_t SAFE_BULLET_NUM       = 3;



}
#endif