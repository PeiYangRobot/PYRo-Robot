#include "pyro_sentry_booster.h"
#include "booster_config.h"

namespace pyro
{

void sentry_booster_t::fsm_active_t::fsm_ready_t::multi_shoot_t::enter(owner *owner)
{

}

void sentry_booster_t::fsm_active_t::fsm_ready_t::multi_shoot_t::execute(owner *owner)
{
    if(owner->_ctx.cmd->multi_shoot == false){
        request_switch(&owner->_state_active._state_ready._singgle_shoot);
        return;
    }

    if (owner->_ctx.cmd->fire_count <= owner ->_ctx.data.current_fire_count +3)
    {
        owner->_ctx.cmd->fire_count+=3;
    }

        owner->_trigger_control();
        owner->_send_trig_command();
    //owner->_ctx.motor.fric_left->send_torque(0);
    //owner->_ctx.motor.fric_right->send_torque(0);
    //owner->_ctx.motor.trigger->send_torque(0);

    //owner->_ctx.cmd->fric_target_speed = 0.0f;
    


    
    //owner->_ctx.motor.trigger
}

void sentry_booster_t::fsm_active_t::fsm_ready_t::multi_shoot_t::exit(owner *owner)
{
    owner->_ctx.data.target_data.trigger_equal_pos = owner->_ctx.data.current_data.trigger_equal_pos;
    owner->_ctx.data.current_fire_count = owner->_ctx.cmd->fire_count;
}

} // namespace pyro