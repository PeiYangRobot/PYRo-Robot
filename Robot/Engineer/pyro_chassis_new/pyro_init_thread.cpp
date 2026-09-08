/**
 * @file pyro_init_thread.cpp
 * @brief 工程车底盘初始化线程
 *
 * 重构说明：去掉了全局 DataBoard 的创建和 topic 注册。
 * 板间通信直接调用 chassis 模块，不再需要中间数据层。
 */
#include "pyro_bsp_uart.h"
#include "pyro_bsp_can.h"
#include "pyro_dwt_drv.h"
#include "FreeRTOS.h"
#include "task.h"

namespace pyro{

/*==================全局驱动指针========================*/
// CAN 驱动指针
can_drv_t *can1_drv = nullptr;
can_drv_t *can2_drv = nullptr;
can_drv_t *can3_drv = nullptr;

extern "C"{
void pyro_init_thread(void *argument){
    (void)argument;

    // DWT 定时器初始化（480MHz 主频）
    dwt_drv_t::init(480);

    /*      UART 初始化          */
    bsp_uart::get_uart1().enable_rx_dma();
    bsp_uart::get_uart5().enable_rx_dma();
    status_t ret = bsp_uart::get_uart7().enable_rx_dma();
    bsp_uart::get_uart10().enable_rx_dma();

    /*       CAN 初始化            */
    // CAN1：麦轮电机 M3508*4
    can1_drv = &bsp_can::get_can1();
    can1_drv->init();
    can1_drv->start();

    // CAN2：后摇臂 DM 电机 + 功率计
    can2_drv = &bsp_can::get_can2();
    can2_drv->init();
    can2_drv->start();

    // CAN3：矿仓 GM6020
    can3_drv = &bsp_can::get_can3();
    can3_drv->init();
    can3_drv->start();

    // 初始化完成，删除自己
    vTaskDelete(nullptr);
}
}

}; // namespace pyro
