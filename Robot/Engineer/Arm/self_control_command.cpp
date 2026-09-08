#include "pyro_core_config.h"
#include "pyro_crc.h"
#include "pyro_uart_drv.h"
#include "pyro_bsp_uart.h"
#include "self_control_command.h"
#include "cmsis_os.h"
#include <cstring>
#include "pyro_core_dma_heap.h"

//==================== 新增：零力超时配置 ====================
#ifndef ZERO_FORCE_FLAG
#define ZERO_FORCE_FLAG 80.0
#endif
// 串口接收超时，单位ms，长时间没收到帧就触发零力
#define SELF_CTRL_RX_TIMEOUT_MS 200

#ifdef SELF_CTRL_UART
static pyro::uart_drv_t* self_control_uart = &SELF_CTRL_UART;
#endif

referee_datalink_frame_t self_control_command;
extern pyro::databoard global_databoard;
QueueSetHandle_t self_control_queue;
__attribute__((section(".dma_heap"))) static uint8_t self_control_buf[128];

float command_buf[6];           // 串口传入的命令
float filter_command[6];        // 经低通滤波器，最终的目标命令
static float alpha = 0.02f;
uint32_t self_axis_id[6];

// 新增静态变量：记录上一次成功收到有效帧的系统tick
static TickType_t last_valid_frame_tick = 0U;
// 零力标志位
static uint8_t self_zero_force_flag = 0U;

static bool self_control_callback(uint8_t *buf, uint16_t len, BaseType_t &xHigherPriorityTaskWoken)
{
    uint16_t i = 0;
    // 遍历缓冲区寻找帧头 0xA5 0x1E
    for (; i < len - 1; i++)
    {
        if (buf[i] == 0xA5 && buf[i + 1] == 0x1E)
        {
            // 从帧头起始位置计算剩余数据长度
            uint16_t remain_len = len - i;
            // 判断剩余数据是否足够一整帧
            if (remain_len >= sizeof(referee_datalink_frame_t))
            {
                // buf+i 指向帧头起始的完整数据块，送入队列
                xQueueSendFromISR(self_control_queue, buf + i, &xHigherPriorityTaskWoken);
                return true;
            }
            // 找到帧头但数据不足一帧，直接返回，等待下一次中断补全数据
            return false;
        }
    }
    // 整个缓冲区未找到帧头 0xA5 0x1E
    return false;
}

extern "C" void self_command_updata_thread(void *arg)
{
    // 初始化接收时间戳
    last_valid_frame_tick = xTaskGetTickCount();
    const TickType_t timeout_tick = pdMS_TO_TICKS(SELF_CTRL_RX_TIMEOUT_MS);

    for(;;)
    {
        if(xQueueReceive(self_control_queue, self_control_buf, 0)==pdPASS)
        {
            if(verify_crc8_check_sum(self_control_buf,sizeof(pyro::frame_header_t))&&
            verify_crc16_check_sum(self_control_buf, sizeof(referee_datalink_frame_t))&&
            ((referee_datalink_frame_t*)self_control_buf)->header.sof == 0xA5&&
            ((referee_datalink_frame_t*)self_control_buf)->cmd_id == static_cast<uint16_t>(pyro::cmd_id::CUSTOM_CONTROLLER)&&
            ((referee_datalink_frame_t*)self_control_buf)->header.data_length == 30)
            {
                memcpy(&self_control_command, self_control_buf, sizeof(referee_datalink_frame_t));
                memcpy(command_buf,&self_control_command.data[0],sizeof(float)*6);
                for(int i=0; i<6; i++)
                {
                    filter_command[i] = filter_command[i]*(1-alpha) + command_buf[i]*alpha;
                }
                // ✅收到有效帧，刷新时间戳，清除零力标志
                last_valid_frame_tick = xTaskGetTickCount();
                self_zero_force_flag = 0U;

                global_databoard.write_topic(self_axis_id[0],*((pyro::genenral_data_t*)&(filter_command[0])));
                global_databoard.write_topic(self_axis_id[1],*((pyro::genenral_data_t*)&(filter_command[1])));
                global_databoard.write_topic(self_axis_id[2],*((pyro::genenral_data_t*)&(filter_command[2])));
                global_databoard.write_topic(self_axis_id[3],*((pyro::genenral_data_t*)&(filter_command[3])));
                global_databoard.write_topic(self_axis_id[4],*((pyro::genenral_data_t*)&(filter_command[4])));
                global_databoard.write_topic(self_axis_id[5],*((pyro::genenral_data_t*)&(filter_command[5])));
            }
        }

        //====================新增超时判断逻辑====================
        TickType_t now_tick = xTaskGetTickCount();
        // FreeRTOS tick溢出兼容，直接用TickType_t减法
        if( (now_tick - last_valid_frame_tick) >= timeout_tick )
        {
            if(self_zero_force_flag != ZERO_FORCE_FLAG)
            {
                // 长时间没有收到合法串口帧，置零力标志
                self_zero_force_flag = ZERO_FORCE_FLAG;
                // 可选：把输出命令置0，防止保持旧的目标值
                for(int i=0;i<6;i++)
                {
                    filter_command[i] = 0.0f;
                    global_databoard.write_topic(self_axis_id[i],*((pyro::genenral_data_t*)&(filter_command[i])));
                }
            }
        }

        vTaskDelay(1);
    }
}

extern "C" void self_control_command_init(void *arg)
{
    self_control_queue = xQueueCreate(10, sizeof(referee_datalink_frame_t));
    // 仅当CMake定义SELF_CTRL_UART时执行串口初始化
#ifdef SELF_CTRL_UART
    // 注册回调
    self_control_uart->add_rx_event_callback(self_control_callback, 0xA501);
    //显式开启 DMA 接收
    self_control_uart->enable_rx_dma();
#endif
    self_axis_id[0] = global_databoard.get_topic_id("arm_ctrl_joint0");
    self_axis_id[1] = global_databoard.get_topic_id("arm_ctrl_joint1");
    self_axis_id[2] = global_databoard.get_topic_id("arm_ctrl_joint2");
    self_axis_id[3] = global_databoard.get_topic_id("arm_ctrl_joint3");
    self_axis_id[4] = global_databoard.get_topic_id("arm_ctrl_joint4");
    self_axis_id[5] = global_databoard.get_topic_id("arm_ctrl_joint5");
    xTaskCreate(self_command_updata_thread, "self_command_updata_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
    vTaskDelete(nullptr);
}