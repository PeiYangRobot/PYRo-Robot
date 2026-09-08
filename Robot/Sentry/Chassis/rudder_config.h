#ifndef __CONFIG_H__
#define __CONFIG_H__
#include <cstdint>
#include "BMI088_driver.h"
#include "pyro_algo_common.h"
#include <arm_math.h> // 引入 CMSIS-DSP 库

constexpr float RUDDER_WHEELBASE          = 0.355f; // 舵左右轴距 
constexpr float RUDDER_FRONTBASE          = 0.36f; // 舵前后轴距 

constexpr float  FL_OFFSET_RAD            = 1.07301962f +PI/2 ;
constexpr float  FR_OFFSET_RAD            = -1.02776718f +PI/2;
constexpr float  BL_OFFSET_RAD            = -0.223194122f +PI/2;
constexpr float  BR_OFFSET_RAD            = -1.86455393f +PI/2;

constexpr float YAW_OFFSET_RAD            = -2.70102878f;

constexpr float WHEEL_SIZE                = 0.0525f;


// 加速度限制：每帧最大速度变化量 (m/s)
// 1kHz → 0.001f = 1.0 m/s²
constexpr float ACCEL_LIMIT_PER_FRAME       = 0.002f;


#endif

