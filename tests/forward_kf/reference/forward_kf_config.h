// Frozen KF_ok 19ea989 forward-filter parameters; test-only.
#ifndef KF_OK_FORWARD_CONFIG_H
#define KF_OK_FORWARD_CONFIG_H
#include "pyro_core_def.h"
namespace pyro
{
// 四状态前向 KF，固定 1 kHz；Q/R/P 均为方差，以下为初始整定值。
constexpr float FORWARD_KF_TS                    = 0.001f;
constexpr float FORWARD_KF_ACCEL_VARIANCE        = 0.04f;   // (m/s^2)^2 / step
constexpr float FORWARD_KF_BIAS_VARIANCE         = 1.0e-7f; // (m/s^2)^2 / step
constexpr float FORWARD_KF_R_IMU                 = 0.25f;   // (m/s^2)^2
constexpr float FORWARD_KF_R_WHEEL               = 0.0025f; // (m/s)^2
constexpr float FORWARD_KF_P_INIT[4]             = {0.01f, 0.25f, 1.0f, 0.25f};
constexpr float FORWARD_KF_MIN_VARIANCE[4]       = {1.0e-8f, 1.0e-6f, 1.0e-5f, 1.0e-8f};
constexpr float FORWARD_KF_WHEEL_CUTOFF_HZ       = 15.0f;
constexpr float FORWARD_KF_WHEEL_WC              = 2.0f * PI * FORWARD_KF_WHEEL_CUTOFF_HZ;
constexpr float FORWARD_KF_WHEEL_DIFF_A          =
    (2.0f - FORWARD_KF_WHEEL_WC * FORWARD_KF_TS) /
    (2.0f + FORWARD_KF_WHEEL_WC * FORWARD_KF_TS);
constexpr float FORWARD_KF_WHEEL_DIFF_B          =
    2.0f * FORWARD_KF_WHEEL_WC / (2.0f + FORWARD_KF_WHEEL_WC * FORWARD_KF_TS);
constexpr float FORWARD_KF_SLIP_NIS_LOW          = 4.0f;
constexpr float FORWARD_KF_SLIP_NIS_HIGH         = 16.0f;
constexpr float FORWARD_KF_SLIP_ACCEL_LOW        = 0.30f; // m/s^2，约 20% 无滑样本的加速度分支不降权
constexpr float FORWARD_KF_SLIP_ACCEL_HIGH       = 0.60f; // m/s^2，加速度残差分支的满评分阈值
constexpr float FORWARD_KF_BIAS_FREEZE_SCORE     = 0.2f; // 轮速不可信时暂停偏置修正
constexpr float FORWARD_KF_SLIP_RISE_TAU         = 0.002f; // s，打滑评分上升
constexpr float FORWARD_KF_SLIP_FALL_TAU         = 0.03f;  // s，轮速权重恢复
constexpr float FORWARD_KF_WHEEL_R_MULTIPLIER    = 500.0f;

}
#endif
