#include "pyro_infantry2_gimbal.h"

namespace pyro {

void infantry2_gimbal_t::state_passive_t::enter(owner *owner) {
    owner->_ctx.deps.motor.pitch->disable();
    owner->_ctx.deps.motor.yaw->disable();

    owner->_ctx.deps.pid.pitch_pos_pid->clear();
    owner->_ctx.deps.pid.pitch_spd_pid->clear();
    owner->_ctx.deps.pid.yaw_pos_pid->clear();
    owner->_ctx.deps.pid.yaw_spd_pid->clear();
}

void infantry2_gimbal_t::state_passive_t::execute(owner *owner) {
    owner->_ctx.data.out_pitch_torque = 0;
    owner->_ctx.data.out_yaw_torque = 0;

    //防跳变
    // pitch在任何场景都无需imu控制
    owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_pitch_motor_rad;
    if(owner->_ctx.cmd->is_imu_control) {
        owner->_ctx.data.target_yaw_rad = owner->_ctx.data.current_yaw_imu_rad;
    } else {
        owner->_ctx.data.target_yaw_rad = owner->_ctx.data.current_yaw_motor_rad;
    }

    _send_motor_command(&owner->_ctx);
}

void infantry2_gimbal_t::state_passive_t::exit(owner *owner) {}

} // namespace pyro