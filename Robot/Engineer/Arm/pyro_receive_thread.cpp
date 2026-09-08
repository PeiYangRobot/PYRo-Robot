/**
 * @file pyro_receive_thread.cpp
 * @brief 遥控器接收任务，直接使用新版 RC 驱动（无 rc_hub）
 * @note global_databoard 是实例，使用 . 操作符
 * @note vx, vy, wz 为 FLOAT 类型
 */
#include "pyro_core_config.h"
#include "pyro_bsp_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pyro_databoard.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_core.h"   // sw_pos_t
#include "pyro_rw_lock.h"   // read_scope_lock

extern pyro::databoard global_databoard;

// 缓存话题 ID
static uint32_t s_topic_vx      = 0;
static uint32_t s_topic_vy      = 0;
static uint32_t s_topic_wz      = 0;
static uint32_t s_topic_enable  = 0;
static uint32_t s_topic_chassis_ctrl_lift_auto = 0;  // 仍保留，但固定写0
static uint32_t s_topic_arm_ctrl_mode      = 0;
static uint32_t s_topic_tg1_start          = 0;
static uint32_t s_topic_tg1_choose         = 0;

extern "C" void pyro_receive_thread(void *argument)
{
    // 等待 DataBoard 话题创建完成
    while (global_databoard.get_topic_id("chassis_ctrl_vx") == 0xFFFFFFFF) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 获取话题 ID
    s_topic_vx     = global_databoard.get_topic_id("chassis_ctrl_vx");
    s_topic_vy     = global_databoard.get_topic_id("chassis_ctrl_vy");
    s_topic_wz     = global_databoard.get_topic_id("chassis_ctrl_wz");
    s_topic_enable = global_databoard.get_topic_id("chassis_ctrl_enable");
    s_topic_chassis_ctrl_lift_auto = global_databoard.get_topic_id("chassis_ctrl_lift_auto");
    s_topic_arm_ctrl_mode = global_databoard.get_topic_id("arm_ctrl_mode");
    s_topic_tg1_start    = global_databoard.get_topic_id("arm_ctrl_tg1_start");
    s_topic_tg1_choose   = global_databoard.get_topic_id("arm_ctrl_tg1_choose");

    pyro::genenral_data_t data;

    while (1) {
#ifdef RC
        // 默认值
        float vx = 0.0f, vy = 0.0f, wz = 0.0f;
        uint32_t enable = 0;
        uint32_t arm_ctrl_mode = 0;
        uint32_t tg1_start = 0;
        uint32_t tg1_choose = 0;

#ifdef DR16_UART
        auto& dr16 = pyro::dr16_drv_t::instance();
        // 检查遥控器是否在线，如果在线则读取数据
        if (dr16.check_online())
        {
            pyro::read_scope_lock lock(dr16.get_lock());
            const auto& vrc = pyro::rc_drv_t::read();

            // 摇杆映射（乘以3放大）
            vx = vrc.axes.ly * 3.0f;
            vy = vrc.axes.lx * 3.0f;
            wz = vrc.axes.rx * 3.0f;

            // 左拨杆向上 → 使能
            enable = (vrc.switches.left.current_pos == pyro::sw_pos_t::UP) ? 1 : 0;

            // ========== 机械臂控制模式映射 ==========
            // 优先判断左拨杆向下：如果左拨杆向下，arm_ctrl_mode = 0，且覆盖所有后续逻辑
            if (vrc.switches.left.current_pos == pyro::sw_pos_t::DOWN)
            {
                arm_ctrl_mode = 0;
                // 不执行右拨杆逻辑
            }
            else
            {
                // 左拨杆非向下，根据右拨杆设置
                if (vrc.switches.right.current_pos == pyro::sw_pos_t::UP)
                {
                    arm_ctrl_mode = 1;
                }
                else if (vrc.switches.right.current_pos == pyro::sw_pos_t::MID)
                {
                    arm_ctrl_mode = 3;
                    tg1_start = 0;          // 居中时 start 为0
                }
                else if (vrc.switches.right.current_pos == pyro::sw_pos_t::DOWN)
                {
                    arm_ctrl_mode = 3;
                    tg1_choose = 2;
                    tg1_start = 1;
                }
            }
        }
#endif

#ifdef VT03_UART
        // VT03 备用（如果 DR16 离线，可启用，但此处未使用，保留原有逻辑）
        auto& vt03 = pyro::vt03_drv_t::instance();
        if (vt03.check_online())
        {
            pyro::read_scope_lock lock(vt03.get_lock());
            const auto& vrc = pyro::rc_drv_t::read();
            // 简单映射，具体可根据需求扩展
            vx = vrc.axes.ly;
            vy = vrc.axes.lx;
            wz = vrc.axes.rx;
            enable = (vrc.switches.gear.current_pos == pyro::sw_pos_t::UP) ? 1 : 0;
            // 注意：VT03 分支未实现机械臂模式映射，可暂时保持默认值
        }
#endif
        // 若都离线，则保持默认值（vx/vy/wz=0，enable=0，arm_ctrl_mode=0等）

        // 写入 DataBoard
        // 速度

        data.data_f = vx;
        global_databoard.write_topic(s_topic_vx, data);
        data.data_f = vy;
        global_databoard.write_topic(s_topic_vy, data);
        data.data_f = wz;
        global_databoard.write_topic(s_topic_wz, data);

        // 使能
        data.data_ui = enable;
        global_databoard.write_topic(s_topic_enable, data);

        // chassis_ctrl_lift_auto 固定为 0
        data.data_ui = 0;
        global_databoard.write_topic(s_topic_chassis_ctrl_lift_auto, data);

        // 机械臂控制模式及一键操作指令
        data.data_ui = arm_ctrl_mode;
        global_databoard.write_topic(s_topic_arm_ctrl_mode, data);

        data.data_ui = tg1_start;
        global_databoard.write_topic(s_topic_tg1_start, data);

        data.data_ui = tg1_choose;
        global_databoard.write_topic(s_topic_tg1_choose, data);
#endif  // RC
        // 100Hz 更新
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}