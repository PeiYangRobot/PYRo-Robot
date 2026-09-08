#include "pyro_board_comm.h"
#include "FreeRTOS.h"
#include "task.h"

namespace pyro
{

status_t board_comm_t::init()
{
    // ==== 订阅清单（按板子区分）====
    // 注意：现有 app 仍占用这些 ID，迁移前先保持注释，避免 register_rx_msg 冲突。
#if BOARD == CHASSIS_BOARD
    rx_subscription_t<g2c_msg_t>::get().register_on(bsp_can::can3);         // 收云台控制
    rx_subscription_t<imu2chassis_msg_t>::get().register_on(bsp_can::can3); // 收 IMU
#elif BOARD == GIMBAL_BOARD
    rx_subscription_t<c2g_msg_t>::get().register_on(bsp_can::can3);         // 收底盘数据
#endif
    // ==== 两板公共的外部设备订阅在此追加（不放进 #if 分支）====
    // rx_subscription_t<ext_dev_msg_t>::get().register_on(bsp_can::canX);

    return PYRO_OK;
}

status_t board_comm_t::start()
{
    xTaskCreate(comm_thread, "pyro_board_comm", 256, nullptr,
                configMAX_PRIORITIES - 1, nullptr);
    return PYRO_OK;
}

// 轮询循环按板子区分：只 poll 本板订阅的消息
void board_comm_t::comm_thread(void* arg)
{
    (void)arg;

    while (true)
    {
#if BOARD == CHASSIS_BOARD
        rx_subscription_t<g2c_msg_t>::get().poll();         // 收云台控制
        rx_subscription_t<imu2chassis_msg_t>::get().poll(); // 收 IMU
#elif BOARD == GIMBAL_BOARD
        rx_subscription_t<c2g_msg_t>::get().poll();         // 收底盘数据
#endif
        // 两板公共的外部设备 poll 放这里（不加 #if）
        // rx_subscription_t<ext_dev_msg_t>::get().poll();

        vTaskDelay(1); // 1kHz
    }
}

} // namespace pyro

extern "C" void board_comm_init(void* argument)
{
    (void)argument;

    pyro::board_comm_t::instance().init();
    pyro::board_comm_t::instance().start();
    vTaskDelete(nullptr);
}
