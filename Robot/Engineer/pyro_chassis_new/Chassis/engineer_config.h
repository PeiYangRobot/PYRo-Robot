#ifndef __ENGINEER_CONFIG_H__
#define __ENGINEER_CONFIG_H__

// =========================================================
// 麦轮底盘参数
// =========================================================
constexpr float WHEEL_RADIUS  = 0.076f;   // 轮子半径 (m)
constexpr float WHEELBASE     = 0.42f;    // 轴距 (m) — 前后轮中心距离
constexpr float TRACK_WIDTH   = 0.42f;    // 轮距 (m) — 左右轮中心距离

// =========================================================
// 矿仓参数（4个位置对应的角度）
// =========================================================
// TODO: 根据实际机械结构调整这4个角度
constexpr float MAGAZINE_ANGLES[4] = {
    0.0f,       // POS_1
    1.57f,      // POS_2 (90度)
    3.14f,      // POS_3 (180度)
    4.71f       // POS_4 (270度)
};
//零点位移 -1.635

// =========================================================
// 摇臂参数
// =========================================================

// ===== 摇臂校准参数 =====
constexpr float LIFT_CALIB_SPEED        = -3.0f;    // 校准速度（rad/s，负=往零点方向）
constexpr float LIFT_CALIB_STALL_SPEED  = 0.5f;     // 堵转速度阈值（rad/s）
constexpr uint32_t LIFT_CALIB_STALL_MS  = 3000;      // 堵转持续时间（ms）
constexpr uint32_t LIFT_CALIB_TIMEOUT   = 10000;    // 单次校准超时（ms）

// 区间核验（观察标定后填实际值）
constexpr float LIFT_ZERO_EXPECTED_MIN  = -3.14f;     // 零点最小值（raw角度，rad）
constexpr float LIFT_ZERO_EXPECTED_MAX  = 6.28f;     // 零点最大值（raw角度，rad）

// 重试参数
constexpr int   LIFT_CALIB_MAX_RETRY    = 3;        // 最大重试次数
constexpr float LIFT_BACKOFF_ANGLE      = 0.8f;     // 每次回退角度（rad）
constexpr float LIFT_BACKOFF_SPEED      = 2.0f;     // 回退速度（rad/s）

// 软件限位
constexpr float LIFT_ANGLE_MIN          = 0.0f;     // 放下位置（相对零点）
constexpr float LIFT_ANGLE_MAX          = 8.0f;     // 收起位置（相对零点，约1.27圈）

constexpr float LIFT_CALIB_DIR[2] = {1.0f,-1.0f};
// =========================================================
// 最大速度限制
// =========================================================
constexpr float MAX_CHASSIS_VX = 3.0f;   // 最大前后速度 m/s
constexpr float MAX_CHASSIS_VY = 3.0f;   // 最大左右速度 m/s
constexpr float MAX_CHASSIS_WZ = 6.0f;   // 最大旋转角速度 rad/s

#endif // __ENGINEER_CONFIG_H__
