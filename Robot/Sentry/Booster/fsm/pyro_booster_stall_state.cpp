#include "pyro_sentry_booster.h"
#include "booster_config.h"

namespace pyro
{

void sentry_booster_t::fsm_active_t::state_stall_t::enter(owner *owner)
{

}

void sentry_booster_t::fsm_active_t::state_stall_t::execute(owner *owner)
{
    
    //owner->_ctx.motor.fric_left->send_torque(0);
    //owner->_ctx.motor.fric_right->send_torque(0);
    owner->_ctx.cmd->fric_on = false;
    owner->_ctx.data.singgle_shoot = false;
    //owner->_ctx.target_data.trigger_pos=  owner->_ctx.current_data.trigger_pos;
    //owner->_ctx.current_fire_count = owner->_ctx.cmd->fire_count;

    owner->_ctx.motor.trigger->send_torque(0);

    //owner->_ctx.target_data.trigger_spd = 0.0f;
    
    if(owner->_ctx.cmd->fric_on){
        request_switch(&owner->_state_active._state_wait);
    }
        owner->_trigger_control();
        owner->_send_trig_command();
    
    //owner->_ctx.motor.trigger->send_torque(0);

}

void sentry_booster_t::fsm_active_t::state_stall_t::exit(owner *owner)
{
}

} // namespace pyro