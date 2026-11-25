/**
 * @file pyro_autoaim_drv.cpp
 * @brief Implementation of Autoaim Driver using Composition.
 *
 * @date 2026-04-06
 */

#include "pyro_autoaim_drv.h"

#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"
#include "pyro_core_dma_heap.h"
#include "pyro_crc.h"
#include "pyro_dwt_drv.h" // 新增：引入高精度定时器头文件
#include <cstring>

namespace pyro
{

/* ========================================================================== */
/* Inner Task Implementation                                                  */
/* ========================================================================== */

status_t autoaim_drv_t::autoaim_task_t::init()
{
    if (_owner)
    {
        _owner->init_impl();
        return PYRO_OK;
    }
    return PYRO_ERROR;
}

void autoaim_drv_t::autoaim_task_t::run_loop()
{
    if (_owner)
    {
        _owner->run_loop_impl();
    }
}

/* ========================================================================== */
/* Driver Implementation                                                      */
/* ========================================================================== */
#ifdef AUTOAIM_UART
/* instance ------------------------------------------------------------------*/
autoaim_drv_t &autoaim_drv_t::get_instance()
{
    // Use the predefined UART handle for autoaim
    static autoaim_drv_t instance(&AUTOAIM_UART);
    return instance;
}
#endif

/* Constructor & Destructor --------------------------------------------------*/
autoaim_drv_t::autoaim_drv_t(uart_drv_t *uart_handle)
    : _uart_drv(uart_handle), _task(nullptr), _tx_buffer(nullptr),
      _rx_msg_buf(nullptr), _is_online(false), _comm_interval_ms(0.0f), _last_rx_time_ms(0.0f)
{
    // 初始化接收与发送缓存
    memset(&_latest_target, 0, sizeof(_latest_target));
    memset(&_tx_payload, 0, sizeof(_tx_payload));

    // 1. Allocate DMA-capable TX buffer
    constexpr size_t buf_size = sizeof(tx_packet_t);
    _tx_buffer = static_cast<tx_packet_t *>(pvPortDmaMalloc(buf_size));

    if (_tx_buffer)
    {
        memset(_tx_buffer, 0, buf_size);
    }

    // 2. Instantiate the internal task
    _task = new autoaim_task_t(this);
}

autoaim_drv_t::~autoaim_drv_t()
{
    if (_task)
    {
        _task->stop();
        delete _task;
        _task = nullptr;
    }

    if (_uart_drv)
    {
        _uart_drv->remove_rx_event_callback(reinterpret_cast<uint32_t>(this));
    }

    if (_tx_buffer)
    {
        vPortFree(_tx_buffer);
        _tx_buffer = nullptr;
    }

    if (_rx_msg_buf)
    {
        vMessageBufferDelete(_rx_msg_buf);
        _rx_msg_buf = nullptr;
    }
}

/* Public Control Methods ----------------------------------------------------*/
void autoaim_drv_t::start_rx() const
{
    if (_task && _uart_drv && _tx_buffer)
    {
        _task->start();
    }
}

/* Logic Implementation (Private) --------------------------------------------*/

void autoaim_drv_t::init_impl()
{
    // 1. Create Message Buffer (Capacity for ~4 frames + overhead)
    if (_rx_msg_buf == nullptr)
    {
        _rx_msg_buf = xMessageBufferCreate(128);
    }

    if (_rx_msg_buf == nullptr)
        return;

    // 2. Register RX ISR Callback
    _uart_drv->add_rx_event_callback(
        [this](const uint8_t *p, const uint16_t size,
               BaseType_t &task_woken) -> bool
        { return this->rx_callback(p, size, task_woken); },
        reinterpret_cast<uint32_t>(this));
}

void autoaim_drv_t::run_loop_impl()
{
    while (true)
    {
        static rx_packet_t pkt;
        static size_t xReceivedBytes;

        // Wait for connection
        if (xMessageBufferReceive(_rx_msg_buf, &pkt, sizeof(pkt),
                                  portMAX_DELAY) == sizeof(rx_packet_t))
        {
            _is_online = true;
            _last_rx_time_ms = dwt_drv_t::get_timeline_ms(); // 首次连接记录起始时间
        }

        while (_is_online)
        {
            // Block until data arrives (PC vision frequency is usually high, 50ms Timeout is safe)
            xReceivedBytes =
                xMessageBufferReceive(_rx_msg_buf, &pkt, sizeof(pkt), pdMS_TO_TICKS(50));

            if (xReceivedBytes == sizeof(rx_packet_t))
            {
                if (error_check(&pkt) == PYRO_OK)
                {
                    unpack(&pkt);

                    // --- 新增：计算并更新通信间隔 ---
                    float current_time = dwt_drv_t::get_timeline_ms();
                    if (_last_rx_time_ms > 0.0f)
                    {
                        _comm_interval_ms = current_time - _last_rx_time_ms;
                    }
                    _last_rx_time_ms = current_time;
                }
            }
            else if (xReceivedBytes == 0)
            {
                // Timeout -> PC Offline or Vision dropped
                _is_online = false;
                _comm_interval_ms = 0.0f; // 离线时通信间隔归零
                _last_rx_time_ms = 0.0f;  // 重置时间戳防止重新上线时出现异常大的间隔
            }
        }
    }
}

/* ISR Callback --------------------------------------------------------------*/
bool autoaim_drv_t::rx_callback(const uint8_t *p_data, const uint16_t size,
                                 BaseType_t &xHigherPriorityTaskWoken) const
{
    // Check Frame Start and Exact Length based on old PC com logic
    if (size == sizeof(rx_packet_t) && p_data[0] == FRAME_SOF)
    {
        xMessageBufferSendFromISR(_rx_msg_buf, p_data, sizeof(rx_packet_t),
                                  &xHigherPriorityTaskWoken);
        return true;
    }
    return false;
}

/* Protocol Helpers ----------------------------------------------------------*/
status_t autoaim_drv_t::error_check(const rx_packet_t *buf)
{
    // CRC16 Check: Matches your old code append logic which ignores the last byte '\n'
    if (!verify_crc16_check_sum(reinterpret_cast<uint8_t const *>(buf),
                                sizeof(rx_packet_t)))
    {
        return PYRO_ERROR;
    }
    return PYRO_OK;
}

void autoaim_drv_t::unpack(const rx_packet_t *buf)
{
    memcpy(&_latest_target, &buf->data, sizeof(rx_data_t));
}

/* Transmission --------------------------------------------------------------*/
autoaim_drv_t::tx_data_t &autoaim_drv_t::get_tx_data()
{
    // 直接返回内部暂存区引用供外部读写
    return _tx_payload;
}

status_t autoaim_drv_t::send_data() const
{
    if (!_tx_buffer || !_uart_drv)
        return PYRO_ERROR;

    // 1. 填充帧头 SOF
    _tx_buffer->header.sof = FRAME_SOF;

    // 2. 将外部修改好的 _tx_payload 拷贝入 DMA 内存池
    memcpy(&_tx_buffer->data, &_tx_payload, sizeof(tx_data_t));

    // 3. 填充换行符，必须在 CRC 之前设置以符合旧版解包逻辑
    _tx_buffer->tailer.end = '\n';

    // 4. 计算 CRC16，注意：长度计算排除最后的 '\n'
    append_crc16_check_sum(reinterpret_cast<uint8_t *>(_tx_buffer),
                           sizeof(tx_packet_t) - 1);

    // 5. 触发 DMA 发送
    return _uart_drv->write(reinterpret_cast<uint8_t *>(_tx_buffer),
                            sizeof(tx_packet_t));
}

/* Getters -------------------------------------------------------------------*/
const autoaim_drv_t::rx_data_t &autoaim_drv_t::get_target_data() const
{
    return _latest_target;
}

bool autoaim_drv_t::check_online() const
{
    return _is_online;
}

// --- 新增：获取通信间隔接口实现 ---
float autoaim_drv_t::get_comm_interval() const
{
    return _comm_interval_ms;
}

} // namespace pyro