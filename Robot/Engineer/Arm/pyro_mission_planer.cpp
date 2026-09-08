#include "cmsis_os.h"

extern "C"
{
    extern void pyro_init_thread(void *argument);//初始化
    extern void pyro_debug_task(void* argument);//调试
    extern void pyro_processing_thread(void* argument);//中间层
    extern void engineer_arm_init(void* args);//这一句是何意味,回去问问SCUM
    extern void self_control_command_init(void *arg);//自控
    extern void pyro_receive_thread(void *argument);
    extern void arm_interboard_init(void *db_ptr);
    void start_mission_planer_task(void const *argument)
    {
        pyro_init_thread(nullptr);
        vTaskDelay(10);
        xTaskCreate(pyro_debug_task, "pyro_debug_task", 1024, nullptr,
                    1, nullptr);
        vTaskDelay(20);
        xTaskCreate(pyro_processing_thread, "pyro_processing_thread", 2048, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelay(20);
        xTaskCreate(engineer_arm_init, "engineer_arm_init", 2048, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
        vTaskDelay(20);
        xTaskCreate(self_control_command_init, "self_control_command_init", 2048, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);  
        vTaskDelay(20);
        xTaskCreate(pyro_receive_thread, "pyro_receive_thread", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr); 
        vTaskDelay(20);
        xTaskCreate(arm_interboard_init, "arm_interboard_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);          
        vTaskDelete(nullptr);
    }
}