#include "pyro_can_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_core.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_wl_gimbal.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_bsp_can.h"
#include "pyro_dji_motor_drv.h"
#include "gimbal_config.h"
#include "pyro_board_drv.h"
#include "pyro_autoaim_drv.h"


using namespace pyro;

namespace
{
class pitch_motor_t final : public dm_motor_drv_t
{
  public:
    pitch_motor_t(uint32_t tx_id, uint32_t rx_id, bsp_can::which_can bus)
        : dm_motor_drv_t(tx_id, rx_id, bus)
    {
        _online = false;
    }

    status_t update_feedback() override
    {
        taskENTER_CRITICAL();
        const status_t status = dm_motor_drv_t::update_feedback();
        if (status == PYRO_OK)
        {
            _last_feedback_tick = _feedback_msg->get_last_update_time();
            _has_feedback = true;
            _feedback_msg->mark_read();
        }
        taskEXIT_CRITICAL();

        _online = _has_feedback &&
            xTaskGetTickCount() - _last_feedback_tick < pdMS_TO_TICKS(1000);
        return status;
    }

  private:
    TickType_t _last_feedback_tick = 0;
    bool _has_feedback = false;
};
} // namespace

//云台部分
constexpr uint32_t EVENT_BIT_AUTOAIM_TOGGLE_GIMBAL      = (1 << 0);        //左上单击切换自瞄




static TaskHandle_t           gimbal_task_handle = nullptr;
static pyro::wl_gimbal_t      *wl_gimbal_ptr     = nullptr;
static pyro::wl_gimbal_cmd_t  *wl_gimbal_cmd_ptr = nullptr;
static pyro::wl_gimbal_deps_t *wl_gimbal_deps    = nullptr;
static pyro::board_drv_t      *board_ptr         = nullptr;
static pyro::autoaim_drv_t    *autoaim_ptr       = nullptr;

static virtual_rc_t vrc_t;






static void motor_deps_init();

static void gimbal_vt03cmd(virtual_rc_t vrc, uint32_t notify);


static void gimbal_dr16cmd(uint32_t notify){};
static void booster_dr16cmd(uint32_t notify){};



extern "C"
{
    void wl_gimbal_thread(void *argument)
    {
        while(true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);


            if(board_ptr->check_online())
            {
                const auto& c2g_data = board_ptr->get_c2g_rx_data();
                wl_gimbal_cmd_ptr->chassis_is_ready = c2g_data.chassis_is_align_ready;
                wl_gimbal_cmd_ptr->chassis_is_rescuing = c2g_data.chassis_is_rescuing;
            }
            else
            {
                wl_gimbal_cmd_ptr->chassis_is_ready = true;
                wl_gimbal_cmd_ptr->chassis_is_rescuing = false;
            }


            if (vt03_drv_t::instance().check_online())
            {
                pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
                vrc_t = pyro::rc_drv_t::read();
                gimbal_vt03cmd(vrc_t, notify_val);
            }
            else if(dr16_drv_t::instance().check_online())
            {
                gimbal_dr16cmd(notify_val);
                booster_dr16cmd(notify_val);
            }
            else
            {
                wl_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
                wl_gimbal_cmd_ptr->state_cmd = pyro::MotionState::Relax;
            }

            wl_gimbal_ptr->set_command(*wl_gimbal_cmd_ptr);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    void infantry1_gimbal_init(void *argument)
    {

        vTaskDelay(10);

        wl_gimbal_cmd_ptr = new pyro::wl_gimbal_cmd_t();
        wl_gimbal_ptr     = pyro::wl_gimbal_t::instance();

        motor_deps_init();
        wl_gimbal_ptr->configure(*wl_gimbal_deps);
        wl_gimbal_ptr->start();

        autoaim_ptr = &pyro::autoaim_drv_t::get_instance();


        board_ptr = &pyro::board_drv_t::get_instance(pyro::board_drv_t::role_t::GIMBAL,pyro::bsp_can::can1);

        auto &vrc = rc_drv_t::read();

        xTaskCreate(wl_gimbal_thread, "infantry_gimbal_thread", 256,
                    nullptr,configMAX_PRIORITIES - 1, &gimbal_task_handle);

        //订阅
        pyro::btn_broker::subscribe(&vrc.buttons.fn_l, pyro::btn_event_t::SINGLE_CLICK,
                            gimbal_task_handle, EVENT_BIT_AUTOAIM_TOGGLE_GIMBAL);



        vTaskDelete(nullptr);
    }
}



void gimbal_vt03cmd(virtual_rc_t vrc, uint32_t notify)
{
    auto autoaim_cmd    = autoaim_ptr->get_target_data();
    //判断当前模式
    if(vrc.switches.gear.current_pos == pyro::sw_pos_t::UP)
    {
        wl_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
        wl_gimbal_cmd_ptr->state_cmd = pyro::MotionState::Relax;

    }
    else if(vrc.switches.gear.current_pos == pyro::sw_pos_t::DOWN ||
            vrc.switches.gear.current_pos == pyro::sw_pos_t::MID)
    {
        wl_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;

        static bool if_autoaim = false;
        if(notify & EVENT_BIT_AUTOAIM_TOGGLE_GIMBAL)
        {
            if_autoaim = !if_autoaim;
        }

        if(if_autoaim && autoaim_ptr->check_online())
        {
            //自瞄情况
            wl_gimbal_cmd_ptr->state_cmd   = pyro::MotionState::Auto;
            wl_gimbal_cmd_ptr->pitchVel    = 0;
            wl_gimbal_cmd_ptr->yawVel      = 0;
            wl_gimbal_cmd_ptr->targetPitch = autoaim_cmd.targetPitch;
            wl_gimbal_cmd_ptr->targetYaw   = autoaim_cmd.targetYaw;
        }
        else
        {
            //手动情况
            wl_gimbal_cmd_ptr->state_cmd   = pyro::MotionState::Manual;

            wl_gimbal_cmd_ptr->targetPitch = autoaim_cmd.targetPitch;
            wl_gimbal_cmd_ptr->targetYaw   = autoaim_cmd.targetYaw;

            float pitchInput =vrc.axes.ly+vrc.mouse_axes.y*100.0f;
            if(pitchInput > 1.0f)
            {
                pitchInput=1.0f;
            }
            else if(pitchInput < -1.0f)
            {
                pitchInput=-1.0f;
            }
            wl_gimbal_cmd_ptr->pitchVel = -pitchInput * 2.0f;

            float yawInput =vrc.axes.lx+vrc.mouse_axes.x*100.0f;
            if(yawInput > 1.0f)
            {
                yawInput=1.0f;
            }
            else if(yawInput < -1.0f)
            {
                yawInput=-1.0f;
            }
            wl_gimbal_cmd_ptr->yawVel = yawInput*3.0f;
        }

    }
}



void motor_deps_init()
{
    wl_gimbal_deps = new pyro::wl_gimbal_deps_t();

    // 初始化电机
    wl_gimbal_deps->motor_deps.pitch =
        new pitch_motor_t(0x01, 0x00, pyro::bsp_can::can2);
    wl_gimbal_deps->motor_deps.yaw =
        new pyro::dji_gm_6020_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_5, pyro::bsp_can::can1,
            pyro::dji_gm_6020_motor_drv_t::CURRENT_MODE);
    wl_gimbal_deps->motor_deps.pitch->set_position_range(-PI,PI); // 设定位置限位 (rad)
    wl_gimbal_deps->motor_deps.pitch->set_rotate_range(-30.0f,30.0f); // 设定速度限位 (rad/s)
    wl_gimbal_deps->motor_deps.pitch->set_torque_range(-10.0f,10.0f); // 设定扭矩限位 (N.m)
    wl_gimbal_deps->pid_deps.pitch_spd =
        new pyro::pid_t(DM_SPD_PITCH_KP, 0.0f, DM_SPD_PITCH_KD, 0.0f, 7.0f);
    wl_gimbal_deps->pid_deps.pitch_pos =
        new pyro::pid_t(DM_POS_PITCH_KP, 0.0f, DM_POS_PITCH_KD, 0.0f, 10.0f);
    wl_gimbal_deps->pid_deps.yaw_pos =
        new pyro::pid_t(YAW_POS_PID_KP, YAW_POS_PID_KI, YAW_POS_PID_KD, 10.0f, 20.0f,
            20, 1, 10, 1, 4);
    wl_gimbal_deps->pid_deps.yaw_spd =
        new pyro::pid_t(YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD, 0.0f, 3.0f,
            20, 1, 10, 1, 4);

    // 设置 MIT 模式下的阻抗参数 (若使用串级PID输出扭矩，Kp和Kd必须设为0)
    // wl_gimbal_deps->motor_deps.pitch->set_runtime_kp(DM_MOT_PITCH_KP);
    // wl_gimbal_deps->motor_deps.pitch->set_runtime_kd(DM_MOT_PITCH_KD);
    wl_gimbal_deps->motor_deps.pitch->set_runtime_kp(0);
    wl_gimbal_deps->motor_deps.pitch->set_runtime_kd(0);
}