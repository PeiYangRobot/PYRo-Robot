#include "pyro_board_drv.h"
#include "pyro_module_base.h"
#include "pyro_wl_chassis.h"

using namespace pyro;

static TaskHandle_t board_com_task_handl    = nullptr;
static board_drv_t *board_drv_ptr           = nullptr;


static void chassis_feedback()
{
    auto &tx_data = board_drv_ptr->get_c2g_tx_data();
    if(wl_chassis_t::instance()->get_ctx().data.flag.chassis_is_align_ready)
    {
        tx_data.chassis_is_align_ready = 1;
    }
    else
    {
        tx_data.chassis_is_align_ready = 0;
    }
    if(wl_chassis_t::instance()->get_ctx().data.flag.chassis_is_rescuing)
    {
        tx_data.chassis_is_rescuing = 1;
    }
    else
    {
        tx_data.chassis_is_rescuing = 0;
    }
}

extern "C"
{
    void infantry1_board_com_thread(void *argument)
    {
        while (true)
        {
            chassis_feedback();
            board_drv_ptr->send_data();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    void infantry1_board_com_init(void *argument)
    {
        board_drv_ptr = &board_drv_t::get_instance(pyro::board_drv_t::role_t::CHASSIS,pyro::bsp_can::can3);
        board_drv_ptr->start_rx();

        xTaskCreate(infantry1_board_com_thread, "board_com_app", 256, nullptr,
                    configMAX_PRIORITIES - 3, &board_com_task_handl);

        vTaskDelete(nullptr);
    }
}