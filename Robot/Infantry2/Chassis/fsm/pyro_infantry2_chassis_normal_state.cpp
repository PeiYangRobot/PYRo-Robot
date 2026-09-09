#include "pyro_infantry2_chassis.h"

namespace pyro {

void infantry2_chassis_t::fsm_active_t::state_normal_t::enter(owner *owner) {}

void infantry2_chassis_t::fsm_active_t::state_normal_t::execute(owner *owner) {
    owner->_ctx.data.target_states =
        _kinematics.solve(owner->_ctx.cmd->vx, owner->_ctx.cmd->vy,
            owner->_ctx.cmd->wz, owner->_ctx.data.current_states);

    _chassis_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void infantry2_chassis_t::fsm_active_t::state_normal_t::exit(owner *owner) {}

} // namespace pyro