#ifndef __PYRO_AUTOAIM_DRV_H__
#define __PYRO_AUTOAIM_DRV_H__

/* Includes ------------------------------------------------------------------*/
#include "pyro_task.h"
#include "pyro_uart_drv.h"
#include "pyro_core_def.h"
#include "message_buffer.h"

namespace pyro
{

class autoaim_drv_t
{
  public:
/* Public Nested Types (User Data Payload) -------------------------------*/
#pragma pack(push, 1)

    /**
     * @brief [MCU -> PC] Feedback Data (Formerly OutputData)
     */
    struct tx_data_t
    {
        float   currentYaw;                 //当前yaw
        float   currentPitch;               //当前pitch
        float   selfVelocityMagnitude;      //未知
        float   selfVelocityAngle;          //未知
        float   initialSpeed;               //未知
        uint8_t shootDelay;                 //当前射弹延迟

        uint8_t robotState  : 5;            //未知
        uint8_t stopRecord  : 1;            //未知
        uint8_t autoAimMode : 1;            //未知
        uint8_t enemyColor  : 1;            //未知
    };

    /**
     * @brief [PC -> MCU] Command Data (Formerly InputData)
     */
    struct rx_data_t
    {
        float   targetYaw;                   //目标yaw
        float   targetYawSpeed;              //目标yaw_radps
        float   targetYawAcceleration;       //目标yaw角加速度
        float   targetPitch;                 //目标pitch
        float   targetPitchSpeed;            //目标pitch_radps
        float   targetPitchAcceleration;     //目标pitch角加速度

        uint8_t fireCommand  : 1;            //是否开火
        uint8_t is_single_shot : 1;          //是否是单发
        uint8_t targetId     : 6;            //未知
        uint8_t aimState;                    //未知
    };

#pragma pack(pop)

#ifdef AUTOAIM_UART
    /* Public Methods --------------------------------------------------------*/
    static autoaim_drv_t &get_instance();
#endif

    /**
     * @brief Starts the internal task and enables communication.
     */
    void start_rx() const;

    /**
     * @brief 获取待发送数据的引用，用于直接修改发送内容。
     * @return 内部待发送数据结构体的引用
     */
    tx_data_t &get_tx_data();

    /**
     * @brief 将内部的 tx_data 数据打包，自动计算 CRC 和换行符并发送。
     */
    status_t send_data() const;

    /**
     * @brief Gets the latest target data from PC.
     */
    [[nodiscard]] const rx_data_t &get_target_data() const;

    /**
     * @brief Checks connection status with PC.
     */
    [[nodiscard]] bool check_online() const;

    /**
     * @brief 获取最新的 PC 视觉通信间隔 (ms)
     */
    [[nodiscard]] float get_comm_interval() const;

  private:
    /**
     * @brief Constructor.
     * @param uart_handle Pointer to the existing PYRO UART driver instance.
     */
    explicit autoaim_drv_t(uart_drv_t *uart_handle);

    /**
     * @brief Destructor. Stops task and frees resources.
     */
    ~autoaim_drv_t();

    /* Private Task Implementation (Composition) -----------------------------*/
    class autoaim_task_t final : public task_base_t
    {
      public:
        // Priority set to NORMAL as Vision target tracking is time-sensitive
        explicit autoaim_task_t(autoaim_drv_t *owner_ptr)
            : task_base_t("autoaim_task", 128, 256, priority_t::NORMAL),
              _owner(owner_ptr)
        {
        }

      protected:
        status_t init() override;
        void run_loop() override;

      private:
        autoaim_drv_t *_owner;
    };

/* Private Protocol Types ------------------------------------------------*/
#pragma pack(push, 1)

    struct frame_header_t
    {
        uint8_t sof; // 0xA5
    };

    struct frame_tailer_t
    {
        uint16_t crc16;
        uint8_t end; // '\n'
    };

    struct rx_frame_tailer_t
    {
        uint16_t crc16;
    };

    struct tx_packet_t
    {
        frame_header_t header;
        tx_data_t data;
        frame_tailer_t tailer;
    };

    struct rx_packet_t
    {
        frame_header_t header;
        rx_data_t data;
        rx_frame_tailer_t tailer;
    };

#pragma pack(pop)

    /* Private Members -------------------------------------------------------*/
    uart_drv_t *_uart_drv;
    autoaim_task_t *_task;   // The internal task instance
    tx_packet_t *_tx_buffer; // DMA buffer
    MessageBufferHandle_t _rx_msg_buf;

    tx_data_t _tx_payload{}; // 缓存用户修改的待发数据
    rx_data_t _latest_target{};
    bool _is_online;

    // --- 新增：通信间隔与时间戳变量 ---
    float _comm_interval_ms{0.0f};
    float _last_rx_time_ms{0.0f};

    static constexpr uint8_t FRAME_SOF = 0xA5;

    /* Private Methods (Logic) -----------------------------------------------*/
    void init_impl();
    void run_loop_impl();

    bool rx_callback(const uint8_t *p_data, uint16_t size,
                     BaseType_t& xHigherPriorityTaskWoken) const;

    static status_t error_check(const rx_packet_t *buf);
    void unpack(const rx_packet_t *buf);
};

}

#endif // __PYRO_AUTOAIM_DRV_H__