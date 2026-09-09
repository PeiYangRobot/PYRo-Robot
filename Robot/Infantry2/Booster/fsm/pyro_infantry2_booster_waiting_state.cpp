#include "pyro_infantry2_booster.h"

namespace pyro {

void infantry2_booster_t::fsm_active_t::state_waiting_t::enter(owner *owner) {
    //防跳变
    owner->_ctx.data.target_trigger_rad = owner->_ctx.data.current_trigger_rad;
    owner->_ctx.data.target_trigger_radps = 0;
}

void infantry2_booster_t::fsm_active_t::state_waiting_t::execute(owner *owner) {
    if(owner->_ctx.cmd->is_fric_on && _is_fric_ready(&owner->_ctx))
        request_switch(&owner->_active_state._ready_state);

    _trigger_control(&owner->_ctx);
    _send_trigger_command(&owner->_ctx);
}

void infantry2_booster_t::fsm_active_t::state_waiting_t::exit(owner *owner) {}

} // namespace pyro