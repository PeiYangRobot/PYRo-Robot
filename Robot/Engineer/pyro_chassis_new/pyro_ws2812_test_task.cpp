/**
 * @file pyro_ws2812_test_task.cpp
 * @brief WS2812 灯带测试任务
 *
 * 使用方法：
 *   1. 修改下方 LED_NUM 为你的实际灯带颗数
 *   2. 在 pyro_mission_planner.cpp 里调用 ws2812_test_init()
 *   3. 确认 CubeMX 里对应 TIM 通道的 PWM + DMA 已配置为 800kHz
 *
 * 效果循环：
 *   自检 → 红绿蓝白 → 呼吸灯 → 彩虹 → 跑马灯 → 循环
 */

#include "ws2812_driver.h"
#include "FreeRTOS.h"
#include "task.h"

using namespace pyro;

/* ==================== 配置区 ==================== */

// TODO: 改成你的实际 LED 颗数
static constexpr uint16_t LED_NUM = 1;

// 任务参数
static constexpr uint16_t TASK_STACK_SIZE = 512;
static constexpr UBaseType_t TASK_PRIORITY = 3;  // 低优先级，不影响控制

// 每个效果持续时间（毫秒）
static constexpr uint32_t COLOR_TEST_INTERVAL = 1000;   // 红绿蓝白每色停留
static constexpr uint32_t BREATHING_DURATION  = 5000;   // 呼吸灯持续
static constexpr uint32_t RAINBOW_DURATION    = 5000;   // 彩虹持续
static constexpr uint32_t CHASE_DURATION      = 5000;   // 跑马灯持续

/* ==================== 任务句柄 ==================== */
static TaskHandle_t s_test_task = nullptr;

/* ==================== 效果实现 ==================== */

// 1. 基础颜色测试：红 → 绿 → 蓝 → 白
static void test_basic_colors(ws2812_t& led)
{
    // 红色
    led.set_all(255, 0, 0);
    led.update();
    vTaskDelay(pdMS_TO_TICKS(COLOR_TEST_INTERVAL));

    // 绿色
    led.set_all(0, 255, 0);
    led.update();
    vTaskDelay(pdMS_TO_TICKS(COLOR_TEST_INTERVAL));

    // 蓝色
    led.set_all(0, 0, 255);
    led.update();
    vTaskDelay(pdMS_TO_TICKS(COLOR_TEST_INTERVAL));

    // 白色
    led.set_all(255, 255, 255);
    led.update();
    vTaskDelay(pdMS_TO_TICKS(COLOR_TEST_INTERVAL));

    // 熄灭
    led.clear();
    led.update();
    vTaskDelay(pdMS_TO_TICKS(500));
}

// 2. 呼吸灯（红色）
static void test_breathing(ws2812_t& led)
{
    TickType_t start = xTaskGetTickCount();
    ws2812_color_t red = {255, 0, 0};

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(BREATHING_DURATION))
    {
        led.breathing(red, 5);  // step=5，呼吸速度
        led.update();
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    led.clear();
    led.update();
}

// 3. 彩虹效果
static void test_rainbow(ws2812_t& led)
{
    TickType_t start = xTaskGetTickCount();
    uint16_t hue = 0;

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(RAINBOW_DURATION))
    {
        led.rainbow(hue, 10);  // density=10，色相分布密度
        led.update();
        hue += 5;               // 色相滚动速度
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    led.clear();
    led.update();
}

// 4. 跑马灯（白色光点在黑色背景上移动）
static void test_chase(ws2812_t& led)
{
    TickType_t start = xTaskGetTickCount();
    ws2812_color_t white = {255, 255, 255};
    uint16_t pos = 0;

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(CHASE_DURATION))
    {
        led.clear();                    // 先全灭
        led.chase(white, pos, 1);      // 在pos位置点亮长度为2的光带
        led.update();
        pos = (pos + 1) % LED_NUM;     // 移动到下一个位置
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    led.clear();
    led.update();
}

/* ==================== 主任务 ==================== */
static void ws2812_test_thread(void *argument)
{
    (void)argument;

    auto& led = ws2812_t::get_instance();

    // 初始化
    status_t ret = led.init(LED_NUM);
    if (ret != PYRO_OK)
    {
        // 初始化失败，任务直接退出
        // 可以在这里加断点或串口打印排查
        vTaskDelete(nullptr);
        return;
    }

    // 先做自检：逐个点亮每颗LED
    
    while(true){
        led.update();
    }
    
}

/* ==================== 对外初始化入口 ==================== */
extern "C" void ws2812_test_init(void)
{
    xTaskCreate(
        ws2812_test_thread,
        "ws2812_test",
        TASK_STACK_SIZE,
        nullptr,
        TASK_PRIORITY,
        &s_test_task);
}
