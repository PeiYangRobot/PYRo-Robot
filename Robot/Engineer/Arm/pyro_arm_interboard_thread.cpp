#include "pyro_core_config.h"
#include "pyro_arm_interboard_thread.h"
#include "pyro_bsp_uart.h"
#include "pyro_crc.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstring>
#include "pyro_databoard.h"
#include "pyro_core_dma_heap.h"

using namespace pyro;

// ==================== 常量 ====================
static constexpr uint16_t FRAME_HEADER = 0xFFA5;
static constexpr uint32_t CALLBACK_OWNER = 0xAA;
static constexpr uint32_t TASK_STACK_SIZE = 512;

// ==================== 帧结构 ====================
#pragma pack(push, 1)
struct upper_to_lower_frame_t {
    uint16_t frame_header;
    uint16_t enable;
    float vx;
    float vy;
    float wz;
    uint16_t magazine_pos;
    uint16_t lift_control_mod;
    uint16_t lift_mannual;
    uint16_t lift_auto;
    uint16_t lift_calib_trigger;
    uint16_t crc16;
};

struct lower_to_upper_frame_t {
    uint16_t frame_header;
    uint16_t chassis_online;
    uint16_t magazine_pos;
    uint16_t magazine_ready;
    uint8_t  magazine_mask;
    uint16_t crc16;
};
#pragma pack(pop)

// ==================== 外部全局 DataBoard ====================
extern pyro::databoard global_databoard;

// ==================== 静态变量 ====================
#ifdef INTERBOARD_UART
static pyro::uart_drv_t* s_uart = &INTERBOARD_UART;
#endif

// 使用 DMA 堆分配缓冲区，确保 DMA 可访问
__attribute__((section(".dma_heap"))) static uint8_t s_rx_buf[sizeof(lower_to_upper_frame_t)];
__attribute__((section(".dma_heap"))) static uint8_t s_tx_buf[sizeof(upper_to_lower_frame_t)];

static volatile bool s_rx_new_frame = false;

static uint32_t s_topic_vx;
static uint32_t s_topic_vy;
static uint32_t s_topic_wz;
static uint32_t s_topic_enable;
static uint32_t s_topic_magazine_pos;
static uint32_t s_topic_lift_control_mod;
static uint32_t s_topic_lift_auto;
static uint32_t s_topic_lift_mannual;
static uint32_t s_topic_lift_calib_trigger;
static uint32_t s_topic_chassis_online;
static uint32_t s_topic_magazine_pos_fb;
static uint32_t s_topic_magazine_ready;
static uint32_t s_topic_magazine_mask;

// ==================== 接收回调（ISR） ====================
bool rx_callback(uint8_t *data, uint16_t len, BaseType_t &xHigherPriorityTaskWoken) {
    if (len == sizeof(lower_to_upper_frame_t)) {
        UBaseType_t uxSaved = taskENTER_CRITICAL_FROM_ISR();
        memcpy((void*)s_rx_buf, data, len);
        s_rx_new_frame = true;
        taskEXIT_CRITICAL_FROM_ISR(uxSaved);
        return true;
    }
    return false;
}

// ==================== 发送函数 ====================
void send_frame(void) {
    upper_to_lower_frame_t *frame = reinterpret_cast<upper_to_lower_frame_t*>(s_tx_buf);
    TickType_t timestamp;
    genenral_data_t data;

    if (global_databoard.read(s_topic_enable, &data, timestamp) == topic::DATA_OK) {
        frame->enable = (uint16_t)data.data_ui;
    } else {
        frame->enable = 0;
    }

    if (global_databoard.read(s_topic_vx, &data, timestamp) == topic::DATA_OK) {
        frame->vx = data.data_f;
    } else {
        frame->vx = 0.0f;
    }
    if (global_databoard.read(s_topic_vy, &data, timestamp) == topic::DATA_OK) {
        frame->vy = data.data_f;
    } else {
        frame->vy = 0.0f;
    }
    if (global_databoard.read(s_topic_wz, &data, timestamp) == topic::DATA_OK) {
        frame->wz = data.data_f;
    } else {
        frame->wz = 0.0f;
    }

    if (global_databoard.read(s_topic_magazine_pos, &data, timestamp) == topic::DATA_OK) {
        frame->magazine_pos = (uint16_t)data.data_ui;
    } else {
        frame->magazine_pos = 0;
    }

    if (global_databoard.read(s_topic_lift_control_mod, &data, timestamp) == topic::DATA_OK) {
        frame->lift_control_mod = (uint16_t)data.data_ui;
    } else {
        frame->lift_control_mod = 0;
    }
    if (global_databoard.read(s_topic_lift_auto, &data, timestamp) == topic::DATA_OK) {
        frame->lift_auto = (uint16_t)data.data_ui;
    } else {
        frame->lift_auto = 0;
    }
    if (global_databoard.read(s_topic_lift_mannual, &data, timestamp) == topic::DATA_OK) {
        frame->lift_mannual = (uint16_t)data.data_ui;
    } else {
        frame->lift_mannual = 0;
    }
    if (global_databoard.read(s_topic_lift_calib_trigger, &data, timestamp) == topic::DATA_OK) {
        frame->lift_calib_trigger = (uint16_t)data.data_ui;
    } else {
        frame->lift_calib_trigger = 0;
    }

    frame->frame_header = FRAME_HEADER;
    append_crc16_check_sum(
        reinterpret_cast<uint8_t*>(frame),
        sizeof(upper_to_lower_frame_t)
    );

#ifdef INTERBOARD_UART
    if (s_uart) {
        // 阻塞轮询发送，确保发送完成后再修改缓冲区
        s_uart->write(
            reinterpret_cast<uint8_t*>(frame),
            sizeof(upper_to_lower_frame_t),
            100   // 超时 100ms
        );
    }
#endif
}

// ==================== 接收处理 ====================
void process_received_frame(void) {
    if (!s_rx_new_frame) return;

    taskENTER_CRITICAL();
    s_rx_new_frame = false;
    taskEXIT_CRITICAL();

    auto *frame = reinterpret_cast<lower_to_upper_frame_t*>(s_rx_buf);

    if (frame->frame_header != FRAME_HEADER) return;
    if (!verify_crc16_check_sum(s_rx_buf, sizeof(lower_to_upper_frame_t))) return;

    genenral_data_t data;
    data.data_ui = frame->chassis_online;
    global_databoard.write_topic(s_topic_chassis_online, data);

    data.data_ui = frame->magazine_pos;
    global_databoard.write_topic(s_topic_magazine_pos_fb, data);

    data.data_ui = frame->magazine_ready;
    global_databoard.write_topic(s_topic_magazine_ready, data);

    data.data_ui = frame->magazine_mask & 0x0F;
    global_databoard.write_topic(s_topic_magazine_mask, data);
}

// ==================== 主通信任务 ====================
void arm_interboard_task(void *arg) {
    (void)arg;

    while (1) {
        send_frame();
        process_received_frame();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== 初始化接口（任务入口） ====================
extern "C" void arm_interboard_init(void *argument) {
#ifdef INTERBOARD_UART
    // 注册接收回调
    s_uart->add_rx_event_callback(rx_callback, CALLBACK_OWNER);
    // 启动 DMA 接收
    s_uart->enable_rx_dma();
#endif

    // 等待 DataBoard 话题就绪并获取所有话题 ID
    while (global_databoard.get_topic_id("chassis_ctrl_enable") == 0xFFFFFFFF) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    s_topic_vx   = global_databoard.get_topic_id("chassis_ctrl_vx");
    s_topic_vy   = global_databoard.get_topic_id("chassis_ctrl_vy");
    s_topic_wz   = global_databoard.get_topic_id("chassis_ctrl_wz");
    s_topic_enable = global_databoard.get_topic_id("chassis_ctrl_enable");
    s_topic_magazine_pos = global_databoard.get_topic_id("chassis_ctrl_magazine_pos");
    s_topic_lift_control_mod = global_databoard.get_topic_id("chassis_ctrl_lift_control_mod");
    s_topic_lift_auto = global_databoard.get_topic_id("chassis_ctrl_lift_auto");
    s_topic_lift_mannual = global_databoard.get_topic_id("chassis_ctrl_lift_mannual");
    s_topic_lift_calib_trigger = global_databoard.get_topic_id("chassis_ctrl_lift_calib_trigger");
    s_topic_chassis_online = global_databoard.get_topic_id("chassis_feedback_online");
    s_topic_magazine_pos_fb = global_databoard.get_topic_id("chassis_feedback_magazine_pos");
    s_topic_magazine_ready = global_databoard.get_topic_id("chassis_feedback_magazine_ready");
    s_topic_magazine_mask = global_databoard.get_topic_id("chassis_feedback_magazine_mask");

    // 创建通信任务
    xTaskCreate(arm_interboard_task,
                "arm_interboard",
                TASK_STACK_SIZE,
                nullptr,
                configMAX_PRIORITIES - 3,
                nullptr);

    // 删除自身
    vTaskDelete(nullptr);
}