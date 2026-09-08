/**
 * @file pyro_debug_task.cpp
 * @brief 工程车 Debug 任务（Jcom 波形调试 + 功率计）
 *
 * 参考旧版 VOFA 功能 + Hero JCOM 架构
 * - 发送：JustFloat 协议，上位机用 Jcom/VOFA 软件看波形
 * - 功率计初始化 + 底盘状态数据上送
 * - 调试串口：UART10（和旧版 VOFA 一致）
 */

#include "pyro_powermeter.h"
#include "pyro_engineer_chassis.h"
#include "pyro_bsp_uart.h"
#include "pyro_module_base.h"
#include "pyro_core_dma_heap.h"

#include "FreeRTOS.h"
#include "task.h"
#include <cstring>

using namespace pyro;

/* ==================== 配置 ==================== */
static constexpr uint8_t  DEBUG_VAR_COUNT  = 12;  // 调试变量数量
static constexpr uint32_t DEBUG_PERIOD_MS  = 3;   // 发送周期(ms)

/* ==================== JustFloat 协议帧尾 ==================== */
static const uint8_t JCOM_FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};

/* ==================== 内部变量 ==================== */
static uart_drv_t        *s_debug_uart = nullptr;
static powermeter_drv_t  *s_powermeter = nullptr;
static engineer_chassis_t *s_chassis   = nullptr;
static uint8_t           *s_send_buf   = nullptr;  // DMA 发送缓冲区

// 调试变量
static float s_debug_vars[DEBUG_VAR_COUNT];

// 变量索引，对应 Jcom 软件的通道号
enum debug_var_index_e {
    VAR_POWER = 0,      // [0] 总功率 W
    VAR_WHEEL_RPM_FL,   // [1] 麦轮转速 FL (rpm)
    VAR_WHEEL_RPM_FR,   // [2] 麦轮转速 FR (rpm)
    VAR_WHEEL_RPM_BL,   // [3] 麦轮转速 BL (rpm)
    VAR_WHEEL_RPM_BR,   // [4] 麦轮转速 BR (rpm)
    VAR_WHEEL_TRQ_FL,   // [5] 麦轮输出 FL (归一化)
    VAR_WHEEL_TRQ_FR,   // [6] 麦轮输出 FR (归一化)
    VAR_WHEEL_TRQ_BL,   // [7] 麦轮输出 BL (归一化)
    VAR_WHEEL_TRQ_BR,   // [8] 麦轮输出 BR (归一化)
    VAR_VX_TARGET,      // [9] 目标 vx (m/s)
    VAR_VY_TARGET,      // [10] 目标 vy (m/s)
    VAR_WZ_TARGET,      // [11] 目标 wz (rad/s)
};

/* ==================== 发送 JustFloat 帧 ==================== */
static void jcom_send()
{
    // 拷贝数据到发送缓冲区
    uint8_t offset = 0;
    memcpy(s_send_buf + offset, s_debug_vars, sizeof(float) * DEBUG_VAR_COUNT);
    offset += sizeof(float) * DEBUG_VAR_COUNT;

    // 加帧尾
    memcpy(s_send_buf + offset, JCOM_FRAME_TAIL, sizeof(JCOM_FRAME_TAIL));
    offset += sizeof(JCOM_FRAME_TAIL);

    // 发送
    if (s_debug_uart) {
        s_debug_uart->write(s_send_buf, offset);
    }
}

/* ==================== 更新调试变量 ==================== */
static void update_debug_vars()
{
    // 1. 功率计数据
    powermeter_data pm_data;
    if (s_powermeter && s_powermeter->get_data(pm_data)) {
        s_debug_vars[VAR_POWER] = pm_data.power;
    } else {
        s_debug_vars[VAR_POWER] = 0.0f;
    }

    // 2. 底盘数据
    if (!s_chassis) return;
    const auto &ctx = s_chassis->get_ctx();

    // 4个麦轮转速 + 输出
    for (int i = 0; i < 4; i++) {
        s_debug_vars[VAR_WHEEL_RPM_FL + i] = ctx.data.current_wheel_rpm[i];
        s_debug_vars[VAR_WHEEL_TRQ_FL + i] = ctx.data.out_wheel_torque[i];
    }

    // 目标速度
    if (ctx.cmd) {
        s_debug_vars[VAR_VX_TARGET] = ctx.cmd->chassis.vx;
        s_debug_vars[VAR_VY_TARGET] = ctx.cmd->chassis.vy;
        s_debug_vars[VAR_WZ_TARGET] = ctx.cmd->chassis.wz;
    }
}

/* ==================== Debug 主任务 ==================== */
static void debug_thread(void *argument)
{
    (void)argument;

    // 等底盘模块启动
    vTaskDelay(pdMS_TO_TICKS(100));
    s_chassis = engineer_chassis_t::instance();

    // 1. 初始化功率计（CAN2, ID 0x212）
    s_powermeter = new powermeter_drv_t(0x212, bsp_can::can2);

    // 2. 初始化调试串口（UART10，和旧版 VOFA 一致）
    s_debug_uart = &bsp_uart::get_uart10();

    // 3. 分配 DMA 发送缓冲区（变量 + 帧尾）
    uint16_t buf_size = (DEBUG_VAR_COUNT + 1) * sizeof(float);
    s_send_buf = static_cast<uint8_t *>(pvPortDmaMalloc(buf_size));

    // 4. 主循环：读数据 → 更新 → 发送
    while (true)
    {
        update_debug_vars();
        jcom_send();
        vTaskDelay(pdMS_TO_TICKS(DEBUG_PERIOD_MS));
    }
}

/* ==================== 对外入口（mission_planner 调用） ==================== */
extern "C" void start_engineer_debug_task(void *arg)
{
    xTaskCreate(debug_thread,
                "debug_task",
                512,
                nullptr,
                configMAX_PRIORITIES - 4,
                nullptr);
    vTaskDelete(nullptr);
}
