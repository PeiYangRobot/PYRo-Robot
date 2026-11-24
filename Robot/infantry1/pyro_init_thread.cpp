#include "BMI088_driver.h"
#include "BMI088_middleware.h"
#include "pyro_bsp_can.h"
#include "pyro_bsp_spi.h"
#include "pyro_bsp_uart.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"
#include "pyro_referee.h"
#include "pyro_supercap_drv.h"
#include "pyro_stp23l_drv.h"
#include "pyro_uart_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_warning_drv.h"

namespace pyro
{
can_drv_t *can1_drv = nullptr;
can_drv_t *can2_drv = nullptr;
can_drv_t *can3_drv = nullptr;
ins_drv_t *ins_drv = nullptr;
referee_drv_t *referee_drv = nullptr;

extern "C"
{
void pyro_init_thread(void *argument)
{
    dwt_drv_t::init(480);
    bsp_can::init_all();
    pyro::rc_drv_t::init_virtual_rc();
    pyro::rc_drv_t::read().buttons.pause.init(true, true, true);
    warning_drv_t::instance().start();


    can1_drv = &bsp_can::get_can1();
    can2_drv = &bsp_can::get_can2();
    can3_drv = &bsp_can::get_can3();

    ins_drv = ins_drv_t::get_instance();
    ins_config_t ins_cfg;
    ins_cfg.calibrate = IMU_CALIBRATION_EN;
    ins_cfg.direct = ins_config_t::imu_direct_t::DIRECT_3;
    //区分出云台和底盘的imu偏置
    #if BOARD == GIMBAL_BOARD
        ins_cfg.gx_offset = 0.00275493739f;
        ins_cfg.gy_offset = 0.000510261336f;
        ins_cfg.gz_offset = 0.00282844482f;
    #endif
    #if BOARD == CHASSIS_BOARD
        ins_cfg.gx_offset = 0.00275493739f;
        ins_cfg.gy_offset = 0.000510261336f;
        ins_cfg.gz_offset = 0.00282844482f;
    #endif
    ins_cfg.g_norm = 9.83213902f;
    static bmi088_spi_t imu_device(
        bsp_spi::get_spi2(),
        {CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin},
        {CS1_GYRO_GPIO_Port, CS1_GYRO_Pin});
    static bmi088_drv_t imu(imu_device);
    ins_drv->init(imu, ins_cfg);

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

#ifdef STP23L_DOWN_UART
    // STP-23L protocol: 230400 baud, 8 data bits, 1 stop bit, no parity.
    // The module streams measurement packets automatically after power-up.
    STP23L_DOWN_UART.reset(230400, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                      UART_PARITY_NONE);
    auto &stp23l = stp23l_drv::down_get_instance();
    stp23l.init();
    STP23L_DOWN_UART.enable_rx_dma();
#endif

#ifdef STP23L_FRONT_UART
    // Second STP-23L on UART7; register its own parser before enabling RX.
    STP23L_FRONT_UART.reset(230400, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                         UART_PARITY_NONE);
    stp23l_drv::front_get_instance().init();
    STP23L_FRONT_UART.enable_rx_dma();
#endif

#ifdef REFEREE_UART
    REFEREE_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                       UART_PARITY_NONE);
    REFEREE_UART.enable_rx_dma();
    referee_drv = referee_drv_t::get_instance();
    referee_drv->init();
#endif

#ifdef SUPERCAP_UART
    SUPERCAP_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                        UART_PARITY_NONE);
    SUPERCAP_UART.enable_rx_dma();
    supercap_drv_t::get_instance()->start_rx();
#endif

    vTaskDelete(nullptr);
}
}
} // namespace pyro
