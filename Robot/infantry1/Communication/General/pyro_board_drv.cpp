/**
 * @file pyro_board_drv.cpp
 * @brief 板间通信驱动实现，使用 bsp_can 直接驱动底层 CAN。
 */

#include "pyro_board_drv.h"
#include "pyro_bsp_can.h"
#include "pyro_dwt_drv.h"
#include <cstring>

namespace pyro
{


board_drv_t::board_task_t::board_task_t(board_drv_t* owner)
    : task_base_t("board_drv_task", 128, 256, priority_t::NORMAL), // 参数根据实际调整
      _owner(owner)
{
}

// ==================== 内部任务 ====================
status_t board_drv_t::board_task_t::init()
{
    if (_owner) {
        _owner->init_impl();
        return PYRO_OK;
    }
    return PYRO_ERROR;
}

void board_drv_t::board_task_t::run_loop()
{
    if (_owner) {
        _owner->run_loop_impl();
    }
}

// ==================== 构造/析构 ====================
board_drv_t::board_drv_t(role_t role, bsp_can::which_can can_ch)
    : _role(role), _can_ch(can_ch), _task(nullptr),
      _is_online(false), _last_rx_time_ms(0.0f), _rx_buf_count(0)
{
    for (auto& p : _rx_buffers) p = nullptr;
    _task = new board_task_t(this);
}

board_drv_t::~board_drv_t()
{
    // 释放所有接收缓冲区（必须在 can_drv_t 析构之前）
    for (uint8_t i = 0; i < _rx_buf_count; ++i) {
        delete _rx_buffers[i];
        _rx_buffers[i] = nullptr;
    }
    if (_task) {
        _task->stop();
        delete _task;
    }
}

// ==================== 单例 ====================
board_drv_t& board_drv_t::get_instance(role_t role, bsp_can::which_can can_ch)
{
    static board_drv_t instance(role, can_ch);
    return instance;
}

// ==================== 初始化：订阅 CAN ID ====================
void board_drv_t::init_impl()
{
    auto* can_drv = bsp_can::get_can(_can_ch);
    if (!can_drv) return;

    uint8_t cnt = 0;
    auto add_buffer = [&](uint32_t id) {
        if (cnt >= MAX_RX_BUFFERS) return;
        auto* buf = new can_msg_buffer_t(id);
        if (can_drv->register_rx_msg(buf) == PYRO_OK) {
            _rx_buffers[cnt++] = buf;
        } else {
            delete buf;
        }
    };

    if (_role == role_t::CHASSIS) {
        // 底盘：订阅云台周期数据 (G2C)
        for (uint8_t i = 0; i < G2C_FRAME_CNT; ++i)
            add_buffer(G2C_BASE_ID + i);
    } else { // GIMBAL
        // 云台：订阅底盘周期数据 (C2G)
        for (uint8_t i = 0; i < C2G_FRAME_CNT; ++i)
            add_buffer(C2G_BASE_ID + i);


    }
    _rx_buf_count = cnt;
}

void board_drv_t::start_rx() const
{
    if (_task) _task->start();
}

// ==================== 后台轮询 ====================
void board_drv_t::run_loop_impl()
{
    while (true) {
        std::array<uint8_t, 8> raw{};
        float current_time = dwt_drv_t::get_timeline_ms();

        // 遍历所有接收缓冲区
        for (uint8_t i = 0; i < _rx_buf_count; ++i) {
            auto* buf = _rx_buffers[i];
            if (!buf || !buf->is_fresh()) continue;

            buf->get_data(raw);
            uint32_t id = buf->get_id();

            // 根据角色处理周期数据
            if (_role == role_t::CHASSIS) {
                if (id >= G2C_BASE_ID && id < G2C_BASE_ID + G2C_FRAME_CNT) {
                    uint8_t idx = id - G2C_BASE_ID;
                    uint8_t* dst = reinterpret_cast<uint8_t*>(&_latest_g2c_rx) + idx * 8;
                    uint8_t len = std::min(8, (int)sizeof(g2c_data_t) - idx * 8);
                    memcpy(dst, raw.data(), len);
                    _is_online = true;
                    _last_rx_time_ms = current_time;
                }
                // 底盘不处理事件（事件由 read_event 主动拉取）
            } else { // GIMBAL
                if (id >= C2G_BASE_ID && id < C2G_BASE_ID + C2G_FRAME_CNT) {
                    uint8_t idx = id - C2G_BASE_ID;
                    uint8_t* dst = reinterpret_cast<uint8_t*>(&_latest_c2g_rx) + idx * 8;
                    uint8_t len = std::min(8, (int)sizeof(c2g_data_t) - idx * 8);
                    memcpy(dst, raw.data(), len);
                    _is_online = true;
                    _last_rx_time_ms = current_time;
                }
                // 事件数据由 read_event_raw 处理，此处不处理
            }
            buf->mark_read();
        }

        // 在线超时检测
        if (_is_online && (current_time - _last_rx_time_ms > 100.0f))
            _is_online = false;

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ==================== 周期数据发送 ====================
status_t board_drv_t::send_data() const
{
    auto* can_drv = bsp_can::get_can(_can_ch);
    if (!can_drv) return PYRO_ERROR;

    if (_role == role_t::GIMBAL) {
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&_g2c_tx_payload);
        for (uint8_t i = 0; i < G2C_FRAME_CNT; ++i) {
            uint8_t data[8] = {0};
            uint8_t len = std::min(8, (int)sizeof(g2c_data_t) - i * 8);
            memcpy(data, ptr + i * 8, len);
            if (can_drv->send_msg(G2C_BASE_ID + i, data) != PYRO_OK)
                return PYRO_ERROR;
        }
    } else { // CHASSIS
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&_c2g_tx_payload);
        for (uint8_t i = 0; i < C2G_FRAME_CNT; ++i) {
            uint8_t data[8] = {0};
            uint8_t len = std::min(8, (int)sizeof(c2g_data_t) - i * 8);
            memcpy(data, ptr + i * 8, len);
            if (can_drv->send_msg(C2G_BASE_ID + i, data) != PYRO_OK)
                return PYRO_ERROR;
        }
    }
    return PYRO_OK;
}

// ==================== 独立事件发送 ====================
status_t board_drv_t::send_event_raw(uint32_t event_base_id, const void* data, size_t size) const
{
    auto* can_drv = bsp_can::get_can(_can_ch);
    if (!can_drv || !data) return PYRO_ERROR;

    const uint8_t* ptr = static_cast<const uint8_t*>(data);
    uint8_t frame_cnt = (size + 7) / 8;
    for (uint8_t i = 0; i < frame_cnt; ++i) {
        uint8_t tx_data[8] = {0};
        uint8_t len = std::min(8, (int)size - i * 8);
        memcpy(tx_data, ptr + i * 8, len);
        if (can_drv->send_msg(event_base_id + i, tx_data) != PYRO_OK)
            return PYRO_ERROR;
    }
    return PYRO_OK;
}

// ==================== 独立事件读取 ====================
bool board_drv_t::read_event_raw(uint32_t event_base_id, void* data_out, size_t size) const
{
    if (!data_out) return false;
    uint8_t* dst = static_cast<uint8_t*>(data_out);
    uint8_t frame_cnt = (size + 7) / 8;
    bool got_any = false;

    for (uint8_t i = 0; i < _rx_buf_count; ++i) {
        auto* buf = _rx_buffers[i];
        if (!buf) continue;
        uint32_t id = buf->get_id();
        if (id < event_base_id || id >= event_base_id + frame_cnt)
            continue;
        if (buf->is_fresh()) {
            std::array<uint8_t, 8> raw;
            buf->get_data(raw);
            uint8_t idx = id - event_base_id;
            uint8_t len = std::min(8, (int)size - idx * 8);
            memcpy(dst + idx * 8, raw.data(), len);
            buf->mark_read();
            got_any = true;
        }
    }
    return got_any;
}

// ==================== 周期数据访问器 ====================
board_drv_t::g2c_data_t& board_drv_t::get_g2c_tx_data()
{
    return _g2c_tx_payload;
}

board_drv_t::c2g_data_t& board_drv_t::get_c2g_tx_data()
{
    return _c2g_tx_payload;
}

const board_drv_t::g2c_data_t& board_drv_t::get_g2c_rx_data() const
{
    return _latest_g2c_rx;
}

const board_drv_t::c2g_data_t& board_drv_t::get_c2g_rx_data() const
{
    return _latest_c2g_rx;
}

bool board_drv_t::check_online() const
{
    return _is_online;
}

} // namespace pyro