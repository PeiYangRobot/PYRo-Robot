#include "pyro_infantry2_booster.h"

namespace pyro {

void infantry2_booster_t::fsm_active_t::state_ready_t::enter(owner *owner) {
    //防跳变
    owner->_ctx.data.target_trigger_rad = owner->_ctx.data.current_trigger_rad;
    owner->_ctx.data.target_trigger_radps = 0;
}

void infantry2_booster_t::fsm_active_t::state_ready_t::execute(owner *owner) {
    if(!owner->_ctx.cmd->is_fric_on || !_is_fric_ready(&owner->_ctx))
        request_switch(&owner->_active_state._waiting_state);
    else {
        if(owner->_ctx.cmd->fire_licence &&
                (owner->_ctx.data.notify_ev & infantry2_booster_t::EVENT_BIT_SINGLE_SHOOT)) {
            if(owner->_ctx.data.is_calibrated) {
                request_switch(&owner->_active_state._single_state);
                owner->_ctx.data.notify_ev &= ~infantry2_booster_t::EVENT_BIT_SINGLE_SHOOT; // 消费
            }
            else {
                request_switch(&owner->_active_state._cali_reverse_state);
            }
        }
    
        if(owner->_ctx.cmd->fire_licence && owner->_ctx.cmd->continue_shoot)
            request_switch(&owner->_active_state._continue_state);
    }
    
    _trigger_control(&owner->_ctx);
    _send_trigger_command(&owner->_ctx);
}

void infantry2_booster_t::fsm_active_t::state_ready_t::exit(owner *owner) {}

} // namespace pyro