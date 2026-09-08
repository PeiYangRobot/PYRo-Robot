#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "FreeRTOS.h"
#include "task.h"
extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void start_debug_task(void *arg);
    extern void sentry_gimbal_init(void *argument);
    extern void sentry_booster_init(void *argument);
    extern void sentry_chassis_init(void *argument);
    extern void board_comm_init(void *argument);


    void start_mission_planer_task(void const *argument)
    {

        xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);

        xTaskCreate(board_comm_init, "pyro_board_comm_init", 256, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);

#if BOARD == GIMBAL_BOARD
        xTaskCreate(sentry_gimbal_init, "pyro_gimbal_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
         xTaskCreate(sentry_booster_init, "pyro_booster_init", 512, nullptr,
                     configMAX_PRIORITIES - 2, nullptr);
#elif BOARD == CHASSIS_BOARD
        xTaskCreate(sentry_chassis_init, "pyro_chassis_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
#endif
        vTaskDelete(nullptr);

    }
}