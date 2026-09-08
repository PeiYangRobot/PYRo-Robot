#include "pyro_bsp_uart.h"
#include "pyro_can_drv.h"
#include "pyro_bsp_can.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"
#include "pyro_supercap_drv.h"
#include "pyro_referee.h"
#include "pyro_vt03_rc_drv.h"

namespace pyro
{
extern "C"
{
    can_drv_t *can1_drv;
    can_drv_t *can2_drv;
    can_drv_t *can3_drv;
    ins_drv_t *ins_drv;
    ins_config_t ins_cfg;


    void pyro_init_thread(void *argument)
    {
        dwt_drv_t::init(480); // Initialize DWT at 550 MHz

        can1_drv = &bsp_can::get_can1();
        can2_drv = &bsp_can::get_can2();
        can3_drv = &bsp_can::get_can3();
        pyro::bsp_can::init_all();
        
        ins_drv = ins_drv_t::get_instance();
        ins_cfg.direct = ins_config_t::imu_direct_t::DIRECT_2;  // 根据实际安装方向选择 1-8
        ins_drv->init(ins_cfg);
       


#ifdef DR16_UART
        dr16_drv_t::instance().start();
        dr16_drv_t::instance().enable();
        DR16_UART.reset(100000, UART_WORDLENGTH_9B, UART_STOPBITS_2,
                UART_PARITY_EVEN);
        DR16_UART.enable_rx_dma();
#endif

#ifdef VT03_UART
        vt03_drv_t::instance().start();
        vt03_drv_t::instance().enable();
        VT03_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                UART_PARITY_NONE);
        VT03_UART.enable_rx_dma();
#endif

#ifdef REFEREE_UART
        REFEREE_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                           UART_PARITY_NONE);
        referee_drv_t::get_instance()->init();
#endif

#ifdef SUPERCAP_UART
        SUPERCAP_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                            UART_PARITY_NONE);
        supercap_drv_t::get_instance()->start_rx();
#endif

        vTaskDelete(nullptr);
    }
}
} // namespace pyro