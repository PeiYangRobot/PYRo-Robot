#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "FreeRTOS.h"
#include "task.h"

extern "C" {
    extern void pyro_init_thread(void *argument);
    extern void start_debug_task(void *arg);

    extern void infantry2_gimbal_init(void *argument);
    extern void infantry2_booster_init(void *argument);
    extern void infantry2_chassis_init(void *argument);

    void start_mission_planer_task(void const *argument) {
        xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        
#if BOARD == GIMBAL_BOARD
        xTaskCreate(infantry2_gimbal_init, "pyro_gimbal_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
        xTaskCreate(infantry2_booster_init, "pyro_booster_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
#elif BOARD == CHASSIS_BOARD
        xTaskCreate(infantry2_chassis_init, "pyro_chassis_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
#endif

        vTaskDelete(nullptr);
    }
}