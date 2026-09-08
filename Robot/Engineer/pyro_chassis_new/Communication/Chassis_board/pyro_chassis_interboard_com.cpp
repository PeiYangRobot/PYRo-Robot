/**
 * @file pyro_chassis_interboard_com.cpp
 * @brief 工程车底盘板间通信（UART7）
 *
 * 重构说明：去掉了全局 DataBoard 中转层，解析完上层帧后
 * 直接构造 engineer_cmd_t 调用 chassis 模块的 set_command()；
 * 回传数据从 chassis 模块的 get_ctx() 直接读取。
 */
#include "pyro_bsp_uart.h"
#include "pyro_crc.h"
#include "pyro_core_def.h"
#include "pyro_engineer_chassis.h"

#include <cstring>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "pyro_core_dma_heap.h"

using namespace pyro;

/*=================常量=================*/
static constexpr uint16_t FRAME_HEADER  = 0xFFA5;
static constexpr uint8_t  RX_QUEUE_SIZE = 10;
static constexpr uint32_t CALLBACK_OWNER = 0x01;
static constexpr uint32_t TIMEOUT_MS    = 500;  // 通信超时时间（ms）

/*==============通信帧定义================*/
#pragma pack(push,1)

// 上层板 -> 底盘
struct upper_to_lower_frame_t{
    uint16_t frame_header;       // 帧头 0xFFA5
    uint16_t enable;             // 使能开关
    float    vx;
    float    vy;
    float    wz;
    uint16_t magazine_pos;       // 矿仓目标档位 1~4
    uint16_t lift_control_mod;   // 摇臂模式 0自动 1手动
    uint16_t lift_mannual;       // 手动模式 0不动 1升起 2放下
    uint16_t lift_auto;          // 自动模式 0不动 1放下 2收起
    uint16_t lift_calib_trigger; // 校准触发
    uint16_t crc16;
};

// 底盘 -> 上层板
struct lower_to_upper_frame_t
{
    uint16_t frame_header;
    uint16_t chassis_online;     // 底盘状态 0正常 非0异常
    uint16_t magazine_pos;       // 矿仓当前档位
    uint16_t magazine_ready;     // 矿仓是否到位
    uint8_t  magazine_mask;      // 矿仓槽位状态（4bit）
    uint16_t crc16;
};
#pragma pack(pop)

/*===============内部变量=============*/
static uart_drv_t     *s_uart     = nullptr;
static QueueHandle_t   s_rx_queue = nullptr;
static TaskHandle_t    s_com_task = nullptr;

__attribute__((section(".dma_heap"))) static upper_to_lower_frame_t s_rx_frame;
__attribute__((section(".dma_heap"))) static lower_to_upper_frame_t s_tx_frame;

static TickType_t s_last_rx_tick = 0;
static bool       s_is_online    = false;

// 复用的 cmd 对象，避免每帧构造开销
static engineer_cmd_t s_cmd;

/*===========接收回调============*/
static bool rx_callback(uint8_t *buf, uint16_t size,
                        BaseType_t &xHigherPriortyTaskWoken)
{
    if(size == 0) return false;
    xQueueSendFromISR(s_rx_queue, buf, &xHigherPriortyTaskWoken);
    return true;
}

/*=========接收解析 → 直接下发命令============*/
static void rx_parse_and_publish()
{
    uint8_t rx_buf[sizeof(upper_to_lower_frame_t) + 10];
    if(xQueueReceive(s_rx_queue, rx_buf, 0) != pdTRUE) return;

    auto *frame = reinterpret_cast<upper_to_lower_frame_t *>(rx_buf);

    // 帧头校验
    if(frame->frame_header != FRAME_HEADER) return;
    // CRC 校验
    if(!verify_crc16_check_sum(rx_buf, sizeof(upper_to_lower_frame_t))) return;

    s_last_rx_tick = xTaskGetTickCount();
    if(!s_is_online) s_is_online = true;

    memcpy(&s_rx_frame, frame, sizeof(upper_to_lower_frame_t));

    /* ===== 直接构造 engineer_cmd_t ===== */

    // 1. 底盘速度
    s_cmd.chassis.vx = s_rx_frame.vx;
    s_cmd.chassis.vy = -s_rx_frame.vy;   // 注意原代码有取反
    s_cmd.chassis.wz = -s_rx_frame.wz;

    // 2. 使能模式
    s_cmd.mode = (s_rx_frame.enable != 0)
                     ? cmd_base_t::mode_t::ACTIVE
                     : cmd_base_t::mode_t::PASSIVE;

    // 3. 矿仓目标档位
    switch(s_rx_frame.magazine_pos){
        case 1: s_cmd.magazine.target_pos = magazine_pos_t::POS_1; break;
        case 2: s_cmd.magazine.target_pos = magazine_pos_t::POS_2; break;
        case 3: s_cmd.magazine.target_pos = magazine_pos_t::POS_3; break;
        case 4: s_cmd.magazine.target_pos = magazine_pos_t::POS_4; break;
        default: break;
    }

    // 4. 摇臂控制
    if(s_rx_frame.lift_control_mod == 0) {
        // 自动模式
        s_cmd.lift.mode = lift_mode_t::AUTO;
        switch(s_rx_frame.lift_auto) {
            case 0: s_cmd.lift.auto_action = lift_action_t::HOLD;    break;
            case 1: s_cmd.lift.auto_action = lift_action_t::DEPLOY;  break;
            case 2: s_cmd.lift.auto_action = lift_action_t::RETRACT; break;
            default: s_cmd.lift.auto_action = lift_action_t::HOLD;   break;
        }
        s_cmd.lift.manual.left_mod  = lift_manual_mod_t::HOLD;
        s_cmd.lift.manual.right_mod = lift_manual_mod_t::HOLD;
    } else {
        // 手动模式
        s_cmd.lift.mode = lift_mode_t::MANUAL;
        lift_manual_mod_t mod = lift_manual_mod_t::HOLD;
        switch(s_rx_frame.lift_mannual) {
            case 0: mod = lift_manual_mod_t::HOLD; break;
            case 1: mod = lift_manual_mod_t::UP;   break;
            case 2: mod = lift_manual_mod_t::DOWN; break;
            default: mod = lift_manual_mod_t::HOLD; break;
        }
        s_cmd.lift.manual.left_mod  = mod;
        s_cmd.lift.manual.right_mod = mod;
        s_cmd.lift.auto_action = lift_action_t::HOLD;
    }

    // 5. 摇臂校准触发（上升沿）
    static uint32_t last_calib_trigger = 0;
    if(s_rx_frame.lift_calib_trigger != 0 && last_calib_trigger == 0) {
        engineer_chassis_t::instance()->lift_start_calibrate(0);
        engineer_chassis_t::instance()->lift_start_calibrate(1);
    }
    last_calib_trigger = s_rx_frame.lift_calib_trigger;

    // 6. 直接下发给底盘模块
    engineer_chassis_t::instance()->set_command(s_cmd);
}

/*=========超时检测============*/
static void check_timeout()
{
    if(s_is_online &&
       (xTaskGetTickCount() - s_last_rx_tick > pdMS_TO_TICKS(TIMEOUT_MS)))
    {
        s_is_online = false;
        // 超时 → 构造一个全停止的安全 cmd 下发
        engineer_cmd_t safe_cmd;  // 默认构造：速度全0，mode=ACTIVE
        safe_cmd.mode = cmd_base_t::mode_t::PASSIVE;
        engineer_chassis_t::instance()->set_command(safe_cmd);
    }
}

/*=============发送回传================*/
static void tx_fill_and_send()
{
    auto &ctx = engineer_chassis_t::instance()->get_ctx();

    s_tx_frame.frame_header = FRAME_HEADER;

    // 1. 底盘在线状态：0=正常，非0=异常
    s_tx_frame.chassis_online = s_is_online ? 0 : 1;

    // 2. 矿仓当前档位（从实际角度换算）
    float cur_angle = ctx.data.current_magazine_angle;
    // 将角度归一化到 [0, 2π)
    while(cur_angle < 0.0f)        cur_angle += 6.2831853f;
    while(cur_angle >= 6.2831853f) cur_angle -= 6.2831853f;
    if     (cur_angle < 0.7854f)  s_tx_frame.magazine_pos = 1;  // [0, π/4)
    else if(cur_angle < 2.3562f)  s_tx_frame.magazine_pos = 2;  // [π/4, 3π/4)
    else if(cur_angle < 3.9270f)  s_tx_frame.magazine_pos = 3;  // [3π/4, 5π/4)
    else                          s_tx_frame.magazine_pos = 4;  // [5π/4, 2π)

    // 3. 矿仓是否到位
    s_tx_frame.magazine_ready = ctx.data.magazine_ready ? 1 : 0;

    // 4. 矿仓槽位状态
    s_tx_frame.magazine_mask = static_cast<uint8_t>(ctx.data.magazine_mask & 0x0F);

    append_crc16_check_sum(
        reinterpret_cast<uint8_t *>(&s_tx_frame),
        sizeof(lower_to_upper_frame_t));

    if(s_uart){
        s_uart->write(reinterpret_cast<uint8_t *>(&s_tx_frame),
                      sizeof(lower_to_upper_frame_t), 100);
    }
}

/* ============ 通信任务 ============ */
static void interboard_com_thread(void *argument)
{
    (void)argument;

    while(true)
    {
        rx_parse_and_publish();  // 接收解析 → 直接下发命令
        check_timeout();         // 超时检测
        tx_fill_and_send();      // 收集状态 → 发送回传
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ============ 对外初始化入口 ============ */
extern "C" void chassis_interboard_com_init()
{
    // 1. 获取 UART7 实例
    s_uart = &bsp_uart::get_uart7();

    // 2. 创建接收队列
    s_rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(upper_to_lower_frame_t));

    // 3. 注册接收回调
    s_uart->add_rx_event_callback(rx_callback, CALLBACK_OWNER);

    
    // 5. 创建通信任务
    xTaskCreate(interboard_com_thread,
                "interboard_com",
                1024,
                nullptr,
                configMAX_PRIORITIES - 3,
                &s_com_task);
}
