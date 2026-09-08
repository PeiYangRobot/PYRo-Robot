#include "pyro_sentry_booster.h"
#include "booster_config.h"


namespace pyro
{

void sentry_booster_t::fsm_active_t::state_wait_t::enter(owner *owner)
{

}

void sentry_booster_t::fsm_active_t::state_wait_t::execute(owner *owner)
{
    owner->_ctx.data.singgle_shoot = false;
    //owner->_ctx.current_fire_count = owner->_ctx.cmd->fire_count;    

    //owner->_ctx.target_data.trigger_pos=  owner->_ctx.current_data.trigger_pos;


        if( owner->_ctx.data.current_data.left_spd > BOOSTER_SHOOT_FRIC_RADPS*0.8f && 
            owner->_ctx.data.current_data.right_spd < -BOOSTER_SHOOT_FRIC_RADPS*0.8f){

            request_switch(&owner->_state_active._state_ready);
            
        }


        owner->_trigger_control();
        owner->_send_trig_command();
        //owner->_ctx.motor.trigger->send_torque(0);
//        owner->_fric_control();
//        owner->_send_fric_command();
}

void sentry_booster_t::fsm_active_t::state_wait_t::exit(owner *owner)
{
}

} // namespace pyro


