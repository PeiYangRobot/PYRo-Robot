#ifndef PYRO_BOARD_DRV_H
#define PYRO_BOARD_DRV_H

#include "pyro_can_drv.h"
#include "pyro_bsp_can.h"
#include "pyro_core_def.h"
#include "pyro_task.h"
#include <cstdint>

namespace pyro
{

class board_drv_t
{
public:
    enum class role_t
    {
        GIMBAL,
        CHASSIS
    };

    /* ================== 通信协议结构体定义 ================== */
#pragma pack(push, 1)

    /**
     * @brief [周期数据区] 云台 -> 底盘
     */
    struct g2c_data_t
    {
        uint32_t mode             : 2;//0下力，1手动(新遥控器下废除)，2平衡

        int32_t vx                : 6;
        int32_t w                 : 6;
        int32_t imu_yaw_radps_100 : 16;

        uint32_t delta_leg        : 2;//0不变，1增大，2减小
        uint32_t step_mode        : 1;
        uint32_t spining          : 1;
        uint32_t rescue           : 1;
        uint32_t jump             : 1;
    };

    /**
     * @brief [周期数据区] 底盘 -> 云台
     */
    struct c2g_data_t
    {

        uint32_t chassis_is_align_ready : 1; //机体姿态对齐的标志位,只供给云盘读取、
        uint32_t chassis_is_rescuing    : 1; //机体姿态自救的标志位,只供给云盘读取、
        uint32_t bullet_speed100        : 13;
        uint32_t current_heat           : 9;
        uint32_t heat_limit             : 9;
        uint32_t cooling_rate           : 5;

    };

    /* ---------------------------------------------------- */
    /* [独立事件区] 扩充事件时，新建结构体并分配独立 ID 即可  */
    /* ---------------------------------------------------- */

    // 事件 A：底盘发弹测速


#pragma pack(pop)
    /* ======================================================= */

    // 周期数据基准 ID
    static constexpr uint32_t G2C_BASE_ID     = 0x011;
    static constexpr uint32_t C2G_BASE_ID     = 0x015;

    // 独立事件基准 ID


    // 周期帧数计算
    static constexpr uint8_t G2C_FRAME_CNT    = (sizeof(g2c_data_t) + 7) / 8;
    static constexpr uint8_t C2G_FRAME_CNT    = (sizeof(c2g_data_t) + 7) / 8;

    /**
     * @brief 获取单例实例
     * @param role 本机角色
     * @param can_ch 使用的 CAN 通道（来自 bsp_can::which_can）
     */
    static board_drv_t& get_instance(role_t role = role_t::GIMBAL,
                                     bsp_can::which_can can_ch = bsp_can::can1);

    void start_rx() const;   // 启动接收后台任务

    // ---- 周期数据交互接口 ----
    g2c_data_t& get_g2c_tx_data();
    c2g_data_t& get_c2g_tx_data();
    [[nodiscard]] const g2c_data_t& get_g2c_rx_data() const;
    [[nodiscard]] const c2g_data_t& get_c2g_rx_data() const;
    status_t send_data() const;   // 发送本机周期数据

    // ---- 独立事件交互接口 ----
    template <typename T>
    status_t send_event(uint32_t event_base_id, const T& data) const
    {
        return send_event_raw(event_base_id, &data, sizeof(T));
    }

    template <typename T>
    bool read_event(uint32_t event_base_id, T& data_out) const
    {
        return read_event_raw(event_base_id, &data_out, sizeof(T));
    }

    [[nodiscard]] bool check_online() const;
    [[nodiscard]] role_t get_role() const { return _role; }

private:
    // 构造/析构私有
    explicit board_drv_t(role_t role, bsp_can::which_can can_ch);
    ~board_drv_t();

    // 底层 raw 接口
    status_t send_event_raw(uint32_t event_base_id, const void* data, size_t size) const;
    bool read_event_raw(uint32_t event_base_id, void* data_out, size_t size) const;

    // 内部任务类
    class board_task_t final : public task_base_t
    {
    public:
        explicit board_task_t(board_drv_t* owner);
    protected:
        status_t init() override;
        void run_loop() override;
    private:
        board_drv_t* _owner;
    };

    void init_impl();        // 订阅 CAN ID
    void run_loop_impl();    // 后台轮询

    // ---- 成员变量 ----
    role_t _role;
    bsp_can::which_can _can_ch;
    board_task_t* _task;

    g2c_data_t _g2c_tx_payload{};
    c2g_data_t _c2g_tx_payload{};
    g2c_data_t _latest_g2c_rx{};
    c2g_data_t _latest_c2g_rx{};

    bool _is_online;
    float _last_rx_time_ms;

    // ---- 手动管理的接收缓冲区 ----
    static constexpr uint8_t MAX_RX_BUFFERS = 16;
    can_msg_buffer_t* _rx_buffers[MAX_RX_BUFFERS];
    uint8_t _rx_buf_count;
};

} // namespace pyro

#endif // PYRO_BOARD_DRV_H