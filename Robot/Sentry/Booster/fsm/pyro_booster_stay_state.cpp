#include "pyro_sentry_booster.h"
#include "booster_config.h"

namespace pyro
{

void sentry_booster_t::fsm_active_t::state_stay_t::enter(owner *owner)
{

}

void sentry_booster_t::fsm_active_t::state_stay_t::execute(owner *owner)
{

    //owner->_ctx.motor.fric_left->send_torque(0);
    //owner->_ctx.motor.fric_right->send_torque(0);
   // owner->_ctx.motor.trigger->send_torque(0);

   // owner->_ctx.cmd->fric_target_speed = 0.0f;
    owner->_ctx.data.singgle_shoot = false;
    //owner->_ctx.current_fire_count = owner->_ctx.cmd->fire_count;
    //owner->_ctx.cmd->fric_on = false;
    //owner->_ctx.target_data.trigger_pos=  owner->_ctx.current_data.trigger_pos;


    if(owner->_ctx.cmd->fric_on){
        request_switch(&owner->_state_active._state_wait);
    }
        
        owner->_trigger_control();
        owner->_send_trig_command();       //不知道在不发单是要不要上力
        //owner->_ctx.motor.trigger->send_torque(0);

}

void sentry_booster_t::fsm_active_t::state_stay_t::exit(owner *owner)
{
}

} // namespace pyro


