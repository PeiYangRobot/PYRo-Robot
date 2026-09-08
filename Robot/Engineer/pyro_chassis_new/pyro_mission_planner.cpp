/**
 * @file pyro_mission_planner.cpp
 * @brief 任务规划器（总入口，按顺序创建所有任务）
 *
 * 重构说明：去掉了 DataBoard 依赖。板间通信直接调用 chassis 模块，
 * 不再需要全局 databoard 中转。
 */

#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "FreeRTOS.h"
#include "task.h"

// C 链接函数声明（给 FreeRTOS 任务用）
extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void chassis_interboard_com_init();
    extern void chassis_app_init();
    extern void ws2812_test_init();
}

/**
 * @brief 任务规划器入口
 *
 * CubeMX 配置的默认任务调用此函数
 * 按顺序初始化所有模块，完成后删除自身
 */
extern "C" void Start_mission_planner(void const *argument)
{
    // 1. 先创建初始化任务（最高优先级，最先跑）
    xTaskCreate(
        pyro_init_thread,
        "pyro_init",
        1024,
        nullptr,
        configMAX_PRIORITIES - 1,
        nullptr);

    // 等待底层驱动初始化完成
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. 初始化板间通信（内部会自己创建通信任务）
    chassis_interboard_com_init();

    // 3. 初始化底盘模块 Application
    chassis_app_init();
    // TODO: 4. 初始化矿仓模块
    // magazine_app_init();

    // 规划器任务完成使命，删除自己
    vTaskDelete(nullptr);
}
