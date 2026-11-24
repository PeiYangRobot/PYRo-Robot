#include "pyro_board_drv.h"
#include "pyro_module_base.h"
#include "pyro_rc_base_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_wl_gimbal.h"

using namespace pyro;

//底盘部分
constexpr uint32_t EVENT_BIT_STEPCLIMB                       = (1 << 0);     // - 左上按钮双击 上台阶
constexpr uint32_t EVENT_BIT_SPINING_TOGGLE                  = (1 << 1);     // - pause键     小陀螺
constexpr uint32_t EVENT_BIT_SELF_RESCUE                     = (1 << 2);     // - pause键长按 进行自救
constexpr uint32_t EVENT_BIT_JUMPER                          = (1 << 3);     // - 左上按钮键     跳跃

static TaskHandle_t board_com_task_handl    = nullptr;
static board_drv_t *board_drv_ptr           = nullptr;


void chassis_vt03cmd(uint32_t notify)
{
    auto &tx_data = board_drv_ptr->get_g2c_tx_data();

    //pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc              = pyro::rc_drv_t::read();
    //判断当前模式
    if(vt03_drv_t::instance().check_online())
    {
        //更新一些基础反馈
        tx_data.imu_yaw_radps_100 = (int)(pyro::wl_gimbal_t::instance()->get_ctx().data.imu.gyro[0] * 100);

        //更新指令信息
        if(vrc.switches.gear.current_pos == pyro::sw_pos_t::UP)
        {
            tx_data.mode      = 0;
            tx_data.step_mode = 0;
            tx_data.spining   = 0;
            tx_data.delta_leg = 0;
            tx_data.vx        = 0;
            tx_data.w         = 0;
        }
        else if(vrc.switches.gear.current_pos == pyro::sw_pos_t::MID)
        {
            tx_data.mode      = 1;
            tx_data.step_mode = 0;
            tx_data.delta_leg = 0;
            tx_data.vx        = vrc.axes.rx * 31.0f;
            tx_data.w         = vrc.axes.ry * 31.0f;

            if(notify & EVENT_BIT_SPINING_TOGGLE)
            {
                tx_data.spining   = 1;
            }
            else
            {
                tx_data.spining   = 0;
            }
        }
        else if(vrc.switches.gear.current_pos == pyro::sw_pos_t::DOWN)
        {
            tx_data.mode      = 2;
            tx_data.step_mode = (notify & EVENT_BIT_STEPCLIMB);
            tx_data.vx        = vrc.axes.ry * 31.0f;
            tx_data.w         = vrc.axes.rx * 31.0f;

            if(notify & EVENT_BIT_SPINING_TOGGLE)
            {
                tx_data.spining   = 1;
            }
            else
            {
                tx_data.spining   = 0;
            }
            if(notify & EVENT_BIT_SELF_RESCUE)
            {
                tx_data.rescue    = 1;
            }
            else
            {
                tx_data.rescue    = 0;
            }
            if(notify & EVENT_BIT_JUMPER)
            {
                tx_data.jump    = 1;
            }
            else
            {
                tx_data.jump    = 0;
            }

            //此时右摇杆水平方向控制腿长
                        //此时右摇杆水平方向控制腿长
            if(abs(vrc.axes.rx) <= 0.1f)
            {
                //死区
                tx_data.delta_leg = 0;
            }
            else
            {
                tx_data.delta_leg = vrc.axes.rx > 0 ? 1 : 2;
            }
        }
    }
    else
    {
        tx_data.mode      = 0;
        tx_data.step_mode = 0;
        tx_data.spining   = 0;
        tx_data.delta_leg = 0;
        tx_data.vx        = 0;
        tx_data.w         = 0;
    }

}

extern "C"
{
    void infantry1_board_com_thread(void *argument)
    {
        while (true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);
            chassis_vt03cmd(notify_val);
            board_drv_ptr->send_data();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    void infantry1_board_com_init(void *argument)
    {


        board_drv_ptr = &board_drv_t::get_instance(pyro::board_drv_t::role_t::GIMBAL,pyro::bsp_can::can1);
        board_drv_ptr->start_rx();

        xTaskCreate(infantry1_board_com_thread, "board_com_app", 256, nullptr,
                    configMAX_PRIORITIES - 2, &board_com_task_handl);

        auto &vrc = pyro::rc_drv_t::read();
        //这里添加要订阅的按键
        pyro::btn_broker::subscribe(&vrc.buttons.fn_l, pyro::btn_event_t::DOUBLE_CLICK,
                            board_com_task_handl, EVENT_BIT_STEPCLIMB);
        pyro::btn_broker::subscribe(&vrc.buttons.pause, pyro::btn_event_t::SINGLE_CLICK,
                            board_com_task_handl, EVENT_BIT_SPINING_TOGGLE);
        pyro::btn_broker::subscribe(&vrc.buttons.pause, pyro::btn_event_t::LONG_PRESS_START,
                            board_com_task_handl, EVENT_BIT_SELF_RESCUE);
        pyro::btn_broker::subscribe(&vrc.buttons.fn_l, pyro::btn_event_t::SINGLE_CLICK,
                            board_com_task_handl, EVENT_BIT_JUMPER);

        vTaskDelete(nullptr);
    }
}