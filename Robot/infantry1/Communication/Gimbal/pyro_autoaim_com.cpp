#include "pyro_autoaim_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_wl_gimbal.h"
#include "pyro_wl_booster.h"


using namespace pyro;


static TaskHandle_t autoaim_app_task_handle = nullptr;
static pyro::autoaim_drv_t *autoaim_drv_ptr = nullptr;

static void process_pc_target_data(const pyro::autoaim_drv_t::rx_data_t &rx_data);
static void update_and_send_feedback();

extern "C"
{
    /**
     * @brief 自动瞄准应用层主循环线程
     */
    void infantry_autoaim_app_thread(void *argument)
    {
        // 延时等待底层设备初始化完成
        vTaskDelay(pdMS_TO_TICKS(500));

        while (true)
        {
            // 1. 检查 PC 视觉是否在线
            if (autoaim_drv_ptr->check_online())
            {
                // 获取 PC 下发的目标数据
                const auto &rx_data = autoaim_drv_ptr->get_target_data();

                // 处理接收到的数据，将目标角度和射击指令下发给模块
                process_pc_target_data(rx_data);
            }

            // 2. 无论是否收到数据，都以固定频率向 PC 发送当前云台姿态和弹速
            update_and_send_feedback();

            // 3. 延时让出 CPU
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    /**
     * @brief 自动瞄准应用层初始化函数 (由 FreeRTOS 启动时调用)
     */
    void infantry_autoaim_init(void *argument)
    {
        // 1. 获取底层驱动实例
        autoaim_drv_ptr = &pyro::autoaim_drv_t::get_instance();

        // 2. 启动驱动层的接收和解析任务
        autoaim_drv_ptr->start_rx();

        // 3. 创建应用层业务线程
        xTaskCreate(infantry_autoaim_app_thread, "autoaim_app_thread", 256, nullptr,
                    configMAX_PRIORITIES - 3, &autoaim_app_task_handle);

        // 4. 初始化完成，删除自身
        vTaskDelete(nullptr);
    }
}


// 解析并处理来自 PC 的自瞄控制数据
static void process_pc_target_data(const pyro::autoaim_drv_t::rx_data_t &rx_data)
{
    // 似乎云台和发射机构应用线程那里直接读取并解析就行
    // 这里不需要添加逻辑
}

/**
 * @brief 获取当前 MCU 状态，并将其发送给 PC
 */
static void update_and_send_feedback()
{
    // 1. 获取待发送数据的引用
    auto &tx_data = autoaim_drv_ptr->get_tx_data();

    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    // 2. 获取当前的运行上下文
    auto gimbal_ctx  = pyro::wl_gimbal_t::instance()->get_ctx();
    auto booster_ctx = pyro::wl_booster_t::instance()->get_ctx();

    //接下来向其中填充pc需要的自瞄数据
    tx_data.currentPitch  = gimbal_ctx.data.imu.pitch;
    tx_data.currentYaw    = gimbal_ctx.data.imu.yaw;
    tx_data.autoAimMode;
    tx_data.enemyColor;
    tx_data.initialSpeed;
    tx_data.robotState;
    tx_data.selfVelocityAngle;
    tx_data.selfVelocityMagnitude;
    tx_data.shootDelay;
    tx_data.stopRecord;

    // 6. 触发底层 DMA 发送
    autoaim_drv_ptr->send_data();
}