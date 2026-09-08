#ifndef PYRO_BOARD_MSG_H
#define PYRO_BOARD_MSG_H

#include "pyro_bsp_can.h"
#include <cstdint>

namespace pyro
{
#pragma pack(push, 1)

// 0x132 云台→底盘 控制指令
struct g2c_msg_t
{
    int8_t vx;         // 前向速度
    int8_t vy;         // 横向速度
    int8_t delta_yaw;  // 航向角增量
    uint8_t flags;     // bit0 active, bit1 follow_en, bit2 spinning
    uint8_t rsv[4];    // 预留

    static constexpr uint32_t ID = 0x132;
    static constexpr bsp_can::which_can BUS = bsp_can::can3;

    bool active() const { return flags & 0x01; }
    bool follow_en() const { return flags & 0x02; }
    bool spinning() const { return flags & 0x04; }
};

// 0x133 底盘→云台 数据（4 字节预留，格式待定）
struct c2g_msg_t
{
    uint8_t data[4];

    static constexpr uint32_t ID = 0x133;
    static constexpr bsp_can::which_can BUS = bsp_can::can3;
};

// 0x103 云台IMU→底盘
struct imu2chassis_msg_t
{
    float yaw_deg;    // 航向角（度），接收端自行转弧度
    float yaw_radps;  // 航向角速度（rad/s）

    static constexpr uint32_t ID = 0x103;
    static constexpr bsp_can::which_can BUS = bsp_can::can3;
};

#pragma pack(pop)

static_assert(sizeof(g2c_msg_t) == 8, "g2c_msg_t must be 8 bytes");
static_assert(sizeof(imu2chassis_msg_t) == 8, "imu2chassis_msg_t must be 8 bytes");

} // namespace pyro

#endif
