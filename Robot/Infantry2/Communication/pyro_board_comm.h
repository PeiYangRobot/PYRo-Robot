#ifndef __PYRO_BOARD_COMM_H__
#define __PYRO_BOARD_COMM_H__

#include <cstdint>
#include <array>

namespace pyro {

// ============================================================================
// 板间 CAN 通信协议 (云台板 <-> 底盘板)
// ----------------------------------------------------------------------------
// 说明:
//   1. 使用 packed 位域结构体定义协议, 通过 union 与 8 字节 CAN buffer 内存重叠,
//      收发两端各用一次整块搬运, 位打包交给编译器完成。
//   2. 两端必须使用同一编译器 (gcc-arm-none-eabi, 位域 LSB 优先),
//      才能保证位域布局一致。
//   3. mode/state 取值与底盘侧 cmd_base_t::mode_t / infantry2_chassis_cmd_t::state_t
//      的枚举顺序对齐, 传输时直接整型互转即可。
// ============================================================================

// 板间 CAN 标准帧 ID (建议使用 can1 总线)
constexpr uint32_t CHASSIS_CMD_ID = 0x100;

// mode 取值 (与 cmd_base_t::mode_t 对齐: PASSIVE=0, ACTIVE=1)
constexpr uint8_t MODE_PASSIVE = 0;
constexpr uint8_t MODE_ACTIVE  = 1;

// state 取值 (与 infantry2_chassis_cmd_t::state_t 对齐: NORMAL=0, FOLLOW_YAW=1, SPIN=2)
constexpr uint8_t STATE_NORMAL     = 0;
constexpr uint8_t STATE_FOLLOW_YAW = 1;
constexpr uint8_t STATE_SPIN       = 2;

// 归一化定点比例: 摇杆 float(-1..1) <-> int(-1023..1023)
constexpr int16_t CHASSIS_RC_SCALE = 1023;

// 位域协议结构体 (2+2+11+11+11+8+5 = 50 bit = 6.25 B, 单个 CAN 帧可容纳)
struct __attribute__((packed)) infantry2_chassis_rc_t {
    uint8_t mode   : 2;   // MODE_PASSIVE / MODE_ACTIVE (留1位扩展)
    uint8_t state  : 2;   // STATE_NORMAL / STATE_FOLLOW_YAW / STATE_SPIN
    int16_t vx     : 11;  // 前后归一化 -1023..1023
    int16_t vy     : 11;  // 左右归一化 -1023..1023
    int16_t wz     : 11;  // 旋转归一化 -1023..1023
    uint8_t seq    : 8;   // 心跳帧计数
    uint8_t rsvd   : 5;   // 预留, 以后加字段用
};
static_assert(sizeof(infantry2_chassis_rc_t) <= 8,
              "chassis rc cmd must fit one CAN frame");

// 结构体 <-> 8 字节 buffer 内存重叠
union infantry2_chassis_rc_u {
    infantry2_chassis_rc_t cmd;
    std::array<uint8_t, 8> data;
};
static_assert(sizeof(infantry2_chassis_rc_u) == 8,
              "board comm frame must be 8 bytes");

// 摇杆 float 归一化 -> int 定点 (带死区, 云台板发送侧使用)
inline int16_t rc_norm(float x, float deadzone = 0.02f) {
    if (x > -deadzone && x < deadzone)
        return 0;
    long v = (long)(x * CHASSIS_RC_SCALE);
    if (v >  CHASSIS_RC_SCALE)
        v =  CHASSIS_RC_SCALE;
    if (v < -CHASSIS_RC_SCALE)
        v = -CHASSIS_RC_SCALE;
    return (int16_t)v;
}

// 速度斜坡限幅: 当前值向目标值每次最多变化 max_step (底盘板接收侧使用)
inline float ramp_value(float cur, float target, float max_step) {
    float diff = target - cur;
    if (diff >  max_step) diff =  max_step;
    if (diff < -max_step) diff = -max_step;
    return cur + diff;
}

} // namespace pyro

#endif // __PYRO_BOARD_COMM_H__