#include "FreeRTOS.h"
#include "pyro_core_config.h"
#include "pyro_core_def.h"
#include "task.h"

extern "C"
{
extern void pyro_init_thread(void *argument);
extern void start_debug_task(void *arg);
extern void infantry1_board_com_init(void *argument);
extern void infantry1_chassis_init(void *argument);
extern void infantry1_gimbal_init(void *argument);
extern void infantry1_booster_init(void *argument);
extern void infantry_autoaim_init(void *argument);

void start_mission_planer_task(void const *argument)
{
    xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                configMAX_PRIORITIES - 1, nullptr);
    vTaskDelay(pdMS_TO_TICKS(20));


#if BOARD == GIMBAL_BOARD
    //云台线程
    xTaskCreate(infantry1_gimbal_init, "infantry1_gimbal_init", 512, nullptr,
                configMAX_PRIORITIES - 2, nullptr);
    //发射机构线程
    xTaskCreate(infantry1_booster_init, "infantry1_booster_init", 512, nullptr,
                configMAX_PRIORITIES - 2, nullptr);
    //PC通信线程
    xTaskCreate(infantry_autoaim_init, "infantry_autoaim_init", 512, nullptr,
                configMAX_PRIORITIES - 2, nullptr);
#endif
#if BOARD == CHASSIS_BOARD
    xTaskCreate(infantry1_chassis_init, "infantry1_chassis_init", 512, nullptr,
                configMAX_PRIORITIES - 2, nullptr);
#endif
    //板间通信线程
    xTaskCreate(infantry1_board_com_init, "pyro_board_com_init", 512, nullptr,
                configMAX_PRIORITIES - 2, nullptr);

#if DEBUG_MODE
    xTaskCreate(start_debug_task, "start_debug_task", 512, nullptr,
                configMAX_PRIORITIES - 3, nullptr);
#endif

    vTaskDelete(nullptr);
}
}
