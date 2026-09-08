/***************************************************
 * CUIJ 调试任务 - 精简重写版（适配新 UART 库）
 * 串口：UART10（通过 BSP 获取）
 * 接收：DMA + IDLE 中断，固定缓冲区
 * 发送：阻塞轮询，避免 DMA BUSY
 * 命令：CUIJ, CUIJ READ, CUIJ WRITE
 * 新增：每秒心跳
 ***************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "pyro_databoard.h"
#include "pyro_bsp_uart.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
namespace pyro{
    extern databoard* global_databoard;
}


// ==================== 接收缓冲区 ====================
#define RX_BUF_SIZE 256
static uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_len = 0;
static volatile bool rx_new_data = false;

// CUIJ 调试串口实例（指针指向 BSP 返回的引用）
static pyro::uart_drv_t* cuij_uart = nullptr;

// ==================== DMA 接收回调（ISR 上下文） ====================
static bool cuij_rx_callback(uint8_t* data, uint16_t len, BaseType_t& xHigherPriorityTaskWoken) {
    if (len == 0) return false;
    uint16_t copy_len = (len > RX_BUF_SIZE) ? RX_BUF_SIZE : len;
    memcpy((void*)rx_buf, data, copy_len);
    rx_len = copy_len;
    rx_new_data = true;
    return true;
}

// ==================== 发送函数（阻塞轮询） ====================
 void cuij_send(const char* str) {
    if (!cuij_uart || !str) return;
    cuij_uart->write((uint8_t*)str, strlen(str), 100);
}

static void cuij_send_binary(const uint8_t* data, uint16_t len) {
    if (!cuij_uart || !data || len == 0) return;
    cuij_uart->write(data, len, 100);
}

// ==================== 命令处理 ====================

// CUIJ READ <topic>
static void cuij_cmd_read(const char* topic_name) {
    uint32_t id = pyro::global_databoard->get_topic_id(topic_name);
    if (id == 0xFFFFFFFF) {
        cuij_send("Topic not found.\n");
        return;
    }
    pyro::genenral_data_t data;
    TickType_t timestamp;
    auto status = pyro::global_databoard->read(id, &data, timestamp);
    if (status == pyro::topic::DATA_INVALID) {
        cuij_send("No valid data yet.\n");
        return;
    } else if (status != pyro::topic::DATA_OK) {
        cuij_send("Read error.\n");
        return;
    }
    uint8_t packet[6];
    packet[0] = 0xA5;
    memcpy(&packet[1], &data, 4);
    packet[5] = 0x5A;
    cuij_send_binary(packet, sizeof(packet));
}

// CUIJ WRITE <topic> <value>
static void cuij_cmd_write(const char* topic_name, const char* value_str) {
    uint32_t id = pyro::global_databoard->get_topic_id(topic_name);
    if (id == 0xFFFFFFFF) {
        cuij_send("Topic not found.\n");
        return;
    }
    char* endptr;

    int32_t ival = (int32_t)strtol(value_str, &endptr, 10);
    if (*endptr == '\0') {
        pyro::genenral_data_t data;
        data.data_si = ival;
        auto ret = pyro::global_databoard->write_topic(id, data);
        if (ret == pyro::topic::DATA_OK)
            cuij_send("Write OK.\n");
        else
            cuij_send("Write failed.\n");
        return;
    }

    float fval = strtof(value_str, &endptr);
    if (*endptr == '\0') {
        pyro::genenral_data_t data;
        data.data_f = fval;
        auto ret = pyro::global_databoard->write_topic(id, data);
        if (ret == pyro::topic::DATA_OK)
            cuij_send("Write OK.\n");
        else
            cuij_send("Write failed (maybe type mismatch).\n");
        return;
    }

    uint32_t uval = (uint32_t)strtoul(value_str, &endptr, 10);
    if (*endptr == '\0') {
        pyro::genenral_data_t data;
        data.data_ui = uval;
        auto ret = pyro::global_databoard->write_topic(id, data);
        if (ret == pyro::topic::DATA_OK)
            cuij_send("Write OK.\n");
        else
            cuij_send("Write failed.\n");
        return;
    }

    cuij_send("Invalid number format.\n");
}

// ==================== 解析入口 ====================
static void cuij_process_line(const char* line) {
    char* cmd = strtok((char*)line, " ");
    if (!cmd) return;

    if (strcmp(cmd, "CUIJ") != 0) {
        cuij_send("Unknown command. Use 'CUIJ ...'\n");
        return;
    }

    char* sub = strtok(nullptr, " ");
    if (!sub) {
        cuij_send("OK\n");
        return;
    }

    if (strcmp(sub, "READ") == 0) {
        char* topic = strtok(nullptr, " ");
        if (topic) cuij_cmd_read(topic);
        else cuij_send("Missing topic name.\n");
    }
    else if (strcmp(sub, "WRITE") == 0) {
        char* topic = strtok(nullptr, " ");
        char* val = strtok(nullptr, " ");
        if (topic && val) cuij_cmd_write(topic, val);
        else cuij_send("Missing arguments.\n");
    }
    else {
        // 未知子命令静默忽略
    }
}

// ==================== 主任务 ====================
extern "C" void pyro_debug_task(void* argument) {
    // 获取 UART10 实例
    cuij_uart = &pyro::bsp_uart::get_uart10();

    // 启用 DMA 接收并注册回调
    cuij_uart->enable_rx_dma();
    cuij_uart->add_rx_event_callback(cuij_rx_callback, 0xDEADBEEF);

    cuij_send("CUIJ Debug ready.\n");

    char line_buf[128];
    //TickType_t last_heartbeat_time = xTaskGetTickCount();  // 初始化心跳计时

    while (1) {
        // 处理接收到的数据
        if (rx_new_data) {
            char* ptr = (char*)rx_buf;
            uint16_t remain = rx_len;
            while (remain > 0) {
                char* nl = (char*)memchr(ptr, '\n', remain);
                if (nl) {
                    uint16_t line_len = nl - ptr + 1;
                    if (line_len > 0) {
                        uint16_t copy_len = (line_len > sizeof(line_buf)-1) ? sizeof(line_buf)-1 : line_len-1;
                        memcpy(line_buf, ptr, copy_len);
                        line_buf[copy_len] = '\0';
                        if (copy_len > 0 && line_buf[copy_len-1] == '\r') {
                            line_buf[copy_len-1] = '\0';
                        }
                        cuij_process_line(line_buf);
                    }
                    ptr += line_len;
                    remain -= line_len;
                } else {
                    break;
                }
            }
            rx_len = 0;
            rx_new_data = false;
        }

        // 心跳：每隔 1 秒发送一次
        /*
        TickType_t now = xTaskGetTickCount();
        if (now - last_heartbeat_time >= pdMS_TO_TICKS(1000)) {
            cuij_send("CUIJ heartbeat\r\n");
            last_heartbeat_time = now;
        }
        */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}