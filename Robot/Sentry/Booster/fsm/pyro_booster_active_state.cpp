#include "pyro_sentry_booster.h"
#include "booster_config.h"
#include "pyro_dwt_drv.h"

namespace pyro
{

void sentry_booster_t::fsm_active_t::on_enter(owner *owner)
{
    owner->_ctx.motor.fric_left->enable();
    
    owner->_ctx.motor.fric_right->enable();
    owner->_ctx.motor.trigger->enable();

    
    change_state(&_state_stay);
}

void sentry_booster_t::fsm_active_t::on_execute(owner *owner)
{
    // if(owner->_ctx.cmd->fric_on){
    //     if(_active_state == &_state_stay){
    //         change_state(&_state_wait);
    //     }

    //     owner->_ctx.cmd->fric_target_speed = BOOSTER_SHOOT_FRIC_SPEED;
        
    // }
    if(!owner->_ctx.cmd->fric_on){

        change_state(&_state_stay);
    }
     // 3. 拨弹盘堵转判断
    // 通过拨盘电机的速度和扭矩判断是否堵转
    constexpr float STALL_TIME_THRESHOLD   = 400.0f; // 堵转时间阈值
    constexpr float STALL_TORQUE_THRESHOLD = 3.0f;   // 堵转扭矩阈值
    constexpr float STALL_SPEED_THRESHOLD  = 0.4f;  // 堵转速度阈值

    static float stall_start_time          = 0.0f;
    if (abs(owner->_ctx.data.current_data.trigger_spd) < STALL_SPEED_THRESHOLD &&
        abs(owner->_ctx.data.current_data.trigger_torque) > STALL_TORQUE_THRESHOLD)
    {
        if (stall_start_time == 0.0f)
        {
            stall_start_time = dwt_drv_t::get_timeline_ms();
        }
        else
        {
            const float elapsed_time =
                dwt_drv_t::get_timeline_ms() - stall_start_time;
            if (elapsed_time >= STALL_TIME_THRESHOLD)
            {
                // 进入堵转状态
                change_state(&_state_stall);
                if (_active_state == &_state_stall)
                {
                    reset();
                }
                stall_start_time = 0.0f; // 重置堵转计时
            }
        }
    }
    else
    {
        stall_start_time = 0.0f; // 重置堵转计时
    }

    
    
    //owner->_solve();
    owner->_fric_control();
    owner->_send_fric_command();
    

}

void sentry_booster_t::fsm_active_t::on_exit(owner *owner)
{
}

} // namespace pyro