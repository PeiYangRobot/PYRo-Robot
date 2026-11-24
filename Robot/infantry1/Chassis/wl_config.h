#ifndef __WL_CONFIG_H__
#define __WL_CONFIG_H__
#include "pyro_algo_common.h"
#include "lqr_coef.h"
#include "pyro_core_def.h"

namespace pyro
{
// IMU 安装偏移与前向加速度带限微分，固定 1 ms 更新。
constexpr float IMU_DX                           = 0.24f;   // m，向前为正
constexpr float IMU_DZ                           = -0.034f; // m，向上为正
constexpr float IMU_ACCEL_TS                     = 0.001f;
constexpr float IMU_ACCEL_CUTOFF_HZ              = 15.0f;
constexpr float IMU_ACCEL_WC                     = 2.0f * PI * IMU_ACCEL_CUTOFF_HZ;
constexpr float IMU_ACCEL_DIFF_A                  =
    (2.0f - IMU_ACCEL_WC * IMU_ACCEL_TS) / (2.0f + IMU_ACCEL_WC * IMU_ACCEL_TS);
constexpr float IMU_ACCEL_DIFF_B                  =
    2.0f * IMU_ACCEL_WC / (2.0f + IMU_ACCEL_WC * IMU_ACCEL_TS);

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
constexpr float FORWARD_KF_SLIP_ACCEL_LOW        = 0.30f; // m/s^2，KF_ok 实测起评值 / Tested onset.
constexpr float FORWARD_KF_SLIP_ACCEL_HIGH       = 0.60f; // m/s^2，KF_ok 实测满评分值 / Tested full score.
constexpr float FORWARD_KF_BIAS_FREEZE_SCORE     = 0.2f; // 不可信轮速暂停零偏学习 / Freeze bias on untrusted wheels.
constexpr float FORWARD_KF_SLIP_RISE_TAU         = 0.002f; // s，打滑评分上升 / Slip-score rise.
constexpr float FORWARD_KF_SLIP_FALL_TAU         = 0.03f;  // s，轮速权重恢复
constexpr float FORWARD_KF_WHEEL_R_MULTIPLIER    = 500.0f;

constexpr float loop_fp32_PI(float val)
{
    while (val > PI)
    {
        val -= 2 * PI;
    }
    while (val < -PI)
    {
        val += 2 * PI;
    }
    return val;
}
constexpr float OJ5                     = 0.11340f;//0.1134
constexpr float J4J5                    = 0.1350f;//0.1350
constexpr float OJ8                     = 0.210f;
// constexpr float MIN_LEG_LENGTH = 0.18f;
// constexpr float MAX_LEG_LENGTH = 2.5f;
constexpr float HIP_CALIBRATION_OFFSET  = 0.50981f;
constexpr float KNEE_CALIBRATION_OFFSET = -1.208815f  ;

constexpr float LEFT_HIP_OFFSET =
    -loop_fp32_PI(0.89581f + HIP_CALIBRATION_OFFSET);
constexpr float LEFT_KNEE_OFFSET =
    -loop_fp32_PI(2.81456f + KNEE_CALIBRATION_OFFSET);
constexpr float RIGHT_HIP_OFFSET =
    -loop_fp32_PI(0.31509f + HIP_CALIBRATION_OFFSET);
constexpr float RIGHT_KNEE_OFFSET =
    -loop_fp32_PI(0.77319f + KNEE_CALIBRATION_OFFSET);


// constexpr float LEFT_HIP_OFFSET   =0;
// constexpr float LEFT_KNEE_OFFSET  =0;
// constexpr float RIGHT_HIP_OFFSET  =0;
// constexpr float RIGHT_KNEE_OFFSET =0;

constexpr float MAX_LEG_LENGTH        = 0.33f;
constexpr float MIN_LEG_LENGTH        = 0.10f;
// Both margins are measured inward from the mechanical limit. Damping starts
// first; stiffness joins it only in the inner region nearest the stop.
constexpr float LEG_LENGTH_WALL_MARGIN = 0.005f; // stiffness + damping: 5 mm
constexpr float LEG_LENGTH_WALL_DAMPING_MARGIN = 0.020f; // damping only starts at 20 mm
constexpr float LEG_LENGTH_WALL_MAX    = MAX_LEG_LENGTH - LEG_LENGTH_WALL_MARGIN;
constexpr float LEG_LENGTH_WALL_MIN    = MIN_LEG_LENGTH + LEG_LENGTH_WALL_MARGIN;
constexpr float LEG_LENGTH_WALL_DAMPING_MAX = MAX_LEG_LENGTH - LEG_LENGTH_WALL_DAMPING_MARGIN;
constexpr float LEG_LENGTH_WALL_DAMPING_MIN = MIN_LEG_LENGTH + LEG_LENGTH_WALL_DAMPING_MARGIN;
constexpr float LEG_LENGTH_WALL_K      = 250000.0f; // N/m
constexpr float LEG_LENGTH_WALL_D      = 750.0f;    // N*s/m; softer damping across the wider region

constexpr float LEFT_LEG_DIRECTION    = -1.0f;
constexpr float RIGHT_LEG_DIRECTION   = 1.0f;

constexpr float LEFT_WHEEL_DIRECTION  = -1.0f;
constexpr float RIGHT_WHEEL_DIRECTION = 1.0f;

// Foot masses used by flight feedforward and contact inertia correction.
constexpr float LEG_MASS              = 1.232f;
constexpr float WHEEL_MASS            = 0.57f;
constexpr float GRAVITY_ACCELERATION  = 9.81f;
// Total robot mass for fixed equal-load support; independent of the LQR model.
constexpr float BALANCE_SUPPORT_MASS = wheel_leg_ground10::kBalanceSupportMass;
constexpr float BALANCE_SUPPORT_FORCE_PER_LEG =
    wheel_leg_ground10::kSupportForcePerLeg; // Exported motor-feedforward support, N per leg.
constexpr float K_t                                  = 0.21777611f;
constexpr float reduction_ratio                      = 13.94f;
constexpr float rec_reduction_ratio                  = 1 / reduction_ratio;
constexpr float MAX_F_L                              = 300.0f;
constexpr float MAX_T_P                              = 60.0f;
constexpr float MAX_CURRENT                          = 15.0f;
constexpr float MAX_T_W                              = K_t * MAX_CURRENT;
constexpr float BALANCE_FORWARD_ACCEL_LIMIT          = 3.0f;  // m/s^2，平衡态前进速度参考的上升速率
constexpr float SPIN_YAW_ACCEL                       = 10.0f; // rad/s^2 //最大的小陀螺加速时的加速度
constexpr float SPIN_YAW_DECEL                       = 10.0f; // rad/s^2 //最大的小陀螺减速是的加速度
constexpr float SPIN_EXIT_DECEL_TIME                 = 0.2f;  // s
constexpr float SPIN_RECOVERY_YAW_KP                 = 2.0f;

// 平均腿长 PID。左右腿施加相同的轴向修正力。
constexpr float MEAN_LEG_LENGTH_PID_KP               = 800.0f; // N/m
constexpr float MEAN_LEG_LENGTH_PID_KI               = 80.0f;  // N/(m*s)
constexpr float MEAN_LEG_LENGTH_PID_KD               = 100.0f; // N*s/m
constexpr float MEAN_LEG_LENGTH_PID_INTEGRAL_LIMIT   = 80.0f;  // N/per leg
constexpr float MEAN_LEG_LENGTH_PID_MAX_OUT          = 80.0f;  // N/per leg
static_assert(MEAN_LEG_LENGTH_PID_INTEGRAL_LIMIT >= 0.0f && MEAN_LEG_LENGTH_PID_MAX_OUT > 0.0f,
              "Mean leg-length PID force limits must be valid");

// 横滚 PID 输出每条腿的轴向差动力，单位 N。
// 左腿加修正力，右腿减修正力。参数供实车调试。
constexpr float NORMAL_ROLL_KP                       = 180.0f; // N/rad
constexpr float NORMAL_ROLL_INTEGRAL_KI              = 60.0f;  // N/(rad*s)
constexpr float NORMAL_ROLL_KD                       = 32.0f;  // N*s/rad
constexpr float NORMAL_ROLL_INTEGRAL_LIMIT           = 10.0f;  // N per leg
constexpr float NORMAL_ROLL_MAX_OUT                  = 40.0f;  // N per leg
constexpr float NORMAL_ROLL_INTEGRAL_DEADBAND        =
    0.2f * PI / 180.0f;
static_assert(NORMAL_ROLL_INTEGRAL_LIMIT >= 0.0f && NORMAL_ROLL_MAX_OUT > 0.0f,
              "Roll PID force limits must be valid");
// 左右腿长差的补偿，削弱右腿
constexpr float NORMAL_ROLL_HEIGHT_RATE_FF_GAIN     = 25.0f; // N/(m/s)
constexpr float NORMAL_ROLL_HEIGHT_RATE_FF_LIMIT    = 7.0f;  // N/腿
constexpr float NORMAL_ROLL_HEIGHT_RATE_FF_TAU      = 0.08f; // s
static_assert(NORMAL_ROLL_HEIGHT_RATE_FF_GAIN >= 0.0f &&
              NORMAL_ROLL_HEIGHT_RATE_FF_LIMIT >= 0.0f &&
              NORMAL_ROLL_HEIGHT_RATE_FF_LIMIT <= NORMAL_ROLL_MAX_OUT &&
              NORMAL_ROLL_HEIGHT_RATE_FF_TAU > 0.0f,
              "Roll height-rate feedforward settings must be valid");

// 反馈使用 Python 导出的完整气弹簧力。此倍率用于电机侧轴向力补偿。
// 平衡通过 evaluateFeedforward 应用。直接下力通过 _motor_leg_force_from_total 应用。
constexpr float GAS_SPRING_COMPENSATION_SCALE        = 1.0f;
//过坎
constexpr float LEG_ACCEL_THRESHOLD = 30.0f;
constexpr float GETOVER_BETA_THRESHOLD = 8.0f * PI / 180.0f;
constexpr float GETOVER_SHORTEN_LENGTH = 0.05f;
constexpr float GETOVER_MIN_TARGET_LENGTH = 0.20f;
constexpr float GETOVER_FORWARD_RAD = 2.0f * PI / 180.0f;
constexpr float GETOVER_FORCE_SCALE = 2.0f;
constexpr float GETOVER_TIMEOUT = 0.12f;
constexpr float GETOVER_REARM_DELAY = 1.0f;
// Airborne and landing detection defaults. Tune from logged support-force data.
constexpr float AIR_LENGTH_TARGET                   = 0.33f;//腿长目标值，低于上限虚拟墙
constexpr float NORMAL_LENGTH_TARGET                 = 0.15f;
constexpr float AIR_LENGTH_RATE                      = 0.60f;
constexpr float AIR_WHEEL_LOCK_K                     = 0.08f;
constexpr float AIR_EXTENSION_MOTOR_TORQUE           = 0.0f;//空中伸腿力矩
constexpr float AIR_TAKEOFF_FORCE_ON                 = 10.0f;//单腿支持力阈值，越小越易离地
//测距模块
constexpr uint32_t AIR_TAKEOFF_DISTANCE_ON_MM          = 305U;
constexpr uint32_t AIR_TAKEOFF_DISTANCE_DOWN_MM        = 270U;

constexpr float AIR_CONTACT_FORCE_OFF                = 35.0f;//落地支持力判据
constexpr float AIR_LANDING_ACC_ON                   = 3.0f;
constexpr float AIR_LANDING_COMPRESSION              = 0.03f;
constexpr float AIR_LENGTH_RECOVERY_RATE             = 0.20f;
constexpr float AIR_LENGTH_RECOVERY_EPSILON          = 0.005f;//腿长误差
constexpr float AIR_ACCEL_LPF_TAU                    = 0.02f;
constexpr float SUPPORT_FORCE_LPF_TAU                = 0.01f;
constexpr float SUPPORT_FORCE_BIAS[2]                = {0.0f, 0.0f};
constexpr uint16_t AIR_TAKEOFF_DEBOUNCE_TICKS        = 8;
constexpr uint16_t AIR_LANDING_DEBOUNCE_TICKS        = 8;

// Active jump state machine. These values are separate from passive airborne
// detection so a commanded jump never depends on a takeoff-force threshold.
constexpr uint32_t JUMP_DUO_TRIGGER_DISTANCE_MM       = 450U;
constexpr float JUMP_PRECOMPRESS_LENGTH               = 0.12f; // 预压缩目标腿长 (m)
constexpr float JUMP_AIR_LENGTH                       = 0.13f; // 收腿完成门限基准 (m)
constexpr float JUMP_PRECOMPRESS_MOTOR_TORQUE         = 5.0f; // 预压缩附加收腿力矩 (N m)
constexpr float JUMP_PUSH_MOTOR_TORQUE                = 15.0f; // 蹬腿伸展力矩 (N m)28
constexpr float JUMP_RECOVERY_MOTOR_TORQUE            = 4.0f; // 空中收腿力矩 (N m)
constexpr float JUMP_PRECOMPRESS_TIME                 = 0.05f; // 预压缩最长时间 (s)
constexpr float JUMP_LENGTH_TOLERANCE                 = 0.003f;
constexpr float JUMP_SETTLED_LEG_SPEED                = 0.2f;
constexpr float JUMP_PUSH_LENGTH_MARGIN               = 0.01f;
constexpr float JUMP_PUSH_TIMEOUT                     = 0.2f;  // 蹬腿最长时间 (s)
constexpr float JUMP_RECOVERY_TIMEOUT                 = 0.2f;  // 收腿最长时间 (s)
constexpr float JUMP_RECOVERY_HOLD_TIME               = 0.1f;
constexpr float JUMP_TOTAL_TIMEOUT                    = 3.0f;  // 空中阶段最长时间 (s)
constexpr float JUMP_LANDING_REVERSE_SPEED            = -0.05f; // 判据 A：腿长收缩速度阈值 (m/s)
constexpr float JUMP_LANDING_LENGTH_MARGIN            = 0.03f; // 判据 B：腿长低于机械最大腿长的余量 (m)
constexpr float JUMP_LANDING_EXTENSION_SPEED_MIN      = 0.10f; // 启用减速判定所需的最小伸展速度峰值 (m/s)
constexpr float JUMP_LANDING_EXTENSION_SPEED_DROP     = 0.15f; // 伸展速度相对峰值的最小下降量 (m/s)
constexpr float JUMP_LANDING_CONFIRM_TIME             = 0.020f; // A 或 B 连续成立的触地确认时间 (s)
constexpr float JUMP_T_P_LIMIT = 8.0f;
constexpr float JUMP_POSITION_ERROR_LIMIT            = 0.8f;
// 当前落地速度恢复流程保留此参考缩放。
constexpr float JUMP_LANDING_VELOCITY_SCALE          = 0.8f; // 退出预压缩时实测速度到落地参考的比例
constexpr float AIR_CONTACT_WALL_GUARD_MARGIN        = 0.03f; // 触地速度判据排除末端制动区
constexpr float AIR_CONTACT_MIN_TIME                = 0.025f;
constexpr float AIR_CONTACT_CONFIRM_TIME            = 0.020f;
constexpr float AIR_CONTACT_RELEASE_FORCE           = 20.0f;
constexpr float AIR_CONTACT_RELEASE_TIME            = 0.02f;
constexpr float LANDING_PITCH_REFERENCE_RATE         = 10.0f; // rad/s; pitch 参考角速度上限
constexpr float LANDING_PITCH_REFERENCE_ACCELERATION = 10.0f; // rad/s^2; pitch 参考角加速度上限
constexpr float LANDING_BETA_REFERENCE_RATE          = 10.0f; // rad/s; 两侧腿角参考角速度上限
constexpr float LANDING_BETA_REFERENCE_ACCELERATION  = 15.0f; // rad/s^2; 两侧腿角参考角加速度上限
constexpr float LANDING_VELOCITY_REFERENCE_RATE      = 5.0f; // m/s^2; 落地速度参考向当前遥控速度的变化率
static_assert(LANDING_PITCH_REFERENCE_RATE > 0.0f &&
              LANDING_PITCH_REFERENCE_ACCELERATION > 0.0f &&
              LANDING_BETA_REFERENCE_RATE > 0.0f &&
              LANDING_BETA_REFERENCE_ACCELERATION > 0.0f,
              "Landing angular reference limits must be positive");

constexpr uint32_t L_WP_POLY_DEGREE                  = 3;

// Python DEFAULT_ROBOT COM curves, in metres; L is in metres.
// Coefficients are in ascending-power order: c0, c1, c2, c3.
constexpr float L_WP_POLY_COEF[L_WP_POLY_DEGREE + 1] = {
    0.05806884f, 0.37602793f, 0.79724260f, -0.73809850f};

static_assert(MIN_LEG_LENGTH == wheel_leg_ground10::kLowerLength &&
              MAX_LEG_LENGTH == wheel_leg_ground10::kUpperLength,
              "Regenerate ground10 coefficients after changing the leg-length domain");

constexpr float WHEEL_RADIUS                         = 0.06f;

constexpr static float YAW_OFFSET = -1.152f;



namespace leg_def
{
enum : uint8_t
{
    L = 0, // LEFT
    R = 1  // RIGHT
};
}
namespace joint_def
{
enum : uint8_t
{
    HIP  = 0,
    KNEE = 1,
};
}



//各种开关
#define Using_Gimbal_Cmd 0
} // namespace pyro
#endif
