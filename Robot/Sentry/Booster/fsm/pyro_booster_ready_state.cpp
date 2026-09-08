#include "pyro_sentry_booster.h"
#include "booster_config.h"

float llspd = 0.0f;
float lrspd =  0.0f;
namespace pyro
{

void sentry_booster_t::fsm_active_t::fsm_ready_t::on_enter(owner *owner)
{
    change_state(&_singgle_shoot);
}

void sentry_booster_t::fsm_active_t::fsm_ready_t::on_execute(owner *owner)
{
    if(!owner->_ctx.cmd->fric_on){
        owner->_ctx.data.target_data.trigger_spd = 0.0f;
        request_switch(&owner->_state_active._state_stay);
    }

    //owner->_ctx.cmd->fric_target_speed = BOOSTER_SHOOT_FRIC_SPEED*1.3f;//有裁判系统后速度换成计算值
        if( owner->_ctx.data.current_data.left_spd < BOOSTER_SHOOT_FRIC_RADPS*0.5f || 
            owner->_ctx.data.current_data.right_spd > -BOOSTER_SHOOT_FRIC_RADPS*0.5f){
                llspd = owner->_ctx.data.current_data.left_spd;
                lrspd = owner->_ctx.data.current_data.right_spd;
            request_switch(&owner->_state_active._state_wait);
            
        }

        if(owner->_ctx.cmd->multi_shoot){
            change_state(&_multi_shoot);
        }
        // owner->_trigger_control();
        // owner->_send_trig_command();
//        owner->_fric_control();
//        owner->_send_fric_command();
}

void sentry_booster_t::fsm_active_t::fsm_ready_t::on_exit(owner *owner)
{
    owner->_ctx.data.singgle_shoot = false;
    owner->_ctx.data.target_data.trigger_spd = 0.0f;
    owner->_ctx.data.target_data.trigger_equal_pos = owner->_ctx.data.current_data.trigger_equal_pos;
    owner->_ctx.data.current_fire_count = owner->_ctx.cmd->fire_count;
}

} // namespace pyro
