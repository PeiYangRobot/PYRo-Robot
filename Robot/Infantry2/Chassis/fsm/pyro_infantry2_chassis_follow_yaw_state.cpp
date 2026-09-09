#include "pyro_infantry2_chassis.h"

namespace pyro {

void infantry2_chassis_t::fsm_active_t::state_follow_yaw_t::enter(owner *owner) {
    owner->_ctx.deps.pid.yaw_follow_pid->clear();
}

void infantry2_chassis_t::fsm_active_t::state_follow_yaw_t::execute(owner *owner) {
    if(fabs(owner->_ctx.data.current_yaw_rad) > infantry2_chassis::YAW_DEADZONE) {
        float wz = owner->_ctx.deps.pid.yaw_follow_pid->calculate(0, owner->_ctx.data.current_yaw_rad);
        owner->_ctx.data.target_states =
            _kinematics.solve(owner->_ctx.cmd->vx, owner->_ctx.cmd->vy, wz, owner->_ctx.data.current_states);
    } else {
        owner->_ctx.data.target_states =
            _kinematics.solve(owner->_ctx.cmd->vx, owner->_ctx.cmd->vy, 0, owner->_ctx.data.current_states);
    }

    _limit_steer_rate(&owner->_ctx);
    _chassis_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void infantry2_chassis_t::fsm_active_t::state_follow_yaw_t::exit(owner *owner) {}

} // namespace pyro