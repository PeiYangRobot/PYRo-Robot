#ifndef PYRO_BOARD_COMM_H
#define PYRO_BOARD_COMM_H

#include "pyro_board_msg.h"
#include "pyro_board_comm_config.h"
#include "pyro_bsp_can.h"
#include "pyro_core_def.h"
#include "FreeRTOS.h"
#include "task.h"
#include <array>
#include <cstdint>
#include <cstring>

namespace pyro
{

/**
 * @brief 每个消息类型一个"订阅槽"：can_msg_buffer_t + 解码缓存 + 新鲜标志。
 *        按类型静态唯一（get() 返回同一个实例）。
 */
template <typename T>
class rx_subscription_t
{
public:
    static rx_subscription_t& get()
    {
        static rx_subscription_t instance;
        return instance;
    }

    void register_on(bsp_can::which_can bus)
    {
        bsp_can::get_can(bus)->register_rx_msg(&_buffer);
    }

    /** @brief 若缓冲区有新数据，则解码到缓存并返回 true。 */
    bool poll()
    {
        if (!_buffer.is_fresh())
            return false;

        std::array<uint8_t, 8> raw{};
        if (!_buffer.get_data(raw))
            return false;
        std::memcpy(&_cache, raw.data(), sizeof(T));
        _buffer.mark_read();
        _fresh = true;
        return true;
    }

    /** @brief 读取最新解码值；返回自上次读取后是否有更新。 */
    bool read(T& out)
    {
        if (!_fresh)
            return false;

        taskENTER_CRITICAL();
        out    = _cache;
        _fresh = false;
        taskEXIT_CRITICAL();
        return true;
    }

    /** @brief 距上次收到数据是否已超时（失联判定）。从未收到过数据也视为超时。 */
    bool is_stale(TickType_t timeout_ms) const
    {
        return (xTaskGetTickCount() - _buffer.get_last_update_time()) >= pdMS_TO_TICKS(timeout_ms);
    }

private:
    rx_subscription_t() : _buffer(T::ID) {}

    can_msg_buffer_t _buffer;
    T _cache{};
    volatile bool _fresh = false;
};

/**
 * @brief 版间/外部 CAN 通信管理类（单例）。
 *        负责注册 RX、启动通信线程、统一轮询解码；
 *        对外提供类型化的 send<T> / read<T> 接口。
 */
class board_comm_t
{
public:
    static board_comm_t& instance()
    {
        static board_comm_t instance;
        return instance;
    }

    status_t init();   // 按 BOARD 注册本板的订阅清单
    status_t start();  // 创建通信线程

    /** @brief 发送接口（任意模块按需调用）。T 需带 static constexpr ID / BUS。 */
    template <typename T>
    status_t send(const T& msg) const;

    /** @brief 读取接口（任意模块调用；只读本板订阅过的类型，未订阅时恒返回 false）。 */
    template <typename T>
    bool read(T& out) const
    {
        return rx_subscription_t<T>::get().read(out);
    }

    /** @brief 失联判定（转发给对应订阅槽）。T 需为本板订阅过的类型。 */
    template <typename T>
    bool is_stale(TickType_t timeout_ms) const
    {
        return rx_subscription_t<T>::get().is_stale(timeout_ms);
    }

private:
    board_comm_t() = default;

    static void comm_thread(void* arg);
};

// ---- 发送实现（inline）：结构体 memcpy 到 8 字节 → 按 T::BUS 发送 ----
template <typename T>
status_t board_comm_t::send(const T& msg) const
{
    static_assert(sizeof(T) <= 8, "msg must fit in one CAN frame (8 bytes)");

    can_drv_t* can = bsp_can::get_can(T::BUS);
    if (nullptr == can)
        return PYRO_PARAM_ERROR;

    uint8_t data[8]{};
    std::memcpy(data, &msg, sizeof(T));
    return can->send_msg(T::ID, data);
}

} // namespace pyro

#endif
