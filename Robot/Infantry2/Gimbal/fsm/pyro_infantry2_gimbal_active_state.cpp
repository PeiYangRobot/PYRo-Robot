#include "pyro_infantry2_gimbal.h"

namespace pyro {

void infantry2_gimbal_t::fsm_active_t::on_enter(owner *owner) {
    //防跳变
    // pitch在任何场景都无需imu控制
    owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_pitch_motor_rad;
    if(owner->_ctx.cmd->is_imu_control) {
        owner->_ctx.data.target_yaw_rad = owner->_ctx.data.current_yaw_imu_rad;
    } else {
        owner->_ctx.data.target_yaw_rad = owner->_ctx.data.current_yaw_motor_rad;
    }

    owner->_ctx.deps.pid.pitch_pos_pid->clear();
    owner->_ctx.deps.pid.pitch_spd_pid->clear();
    owner->_ctx.deps.pid.yaw_pos_pid->clear();
    owner->_ctx.deps.pid.yaw_spd_pid->clear();

    owner->_ctx.deps.motor.pitch->enable();
    owner->_ctx.deps.motor.yaw->enable();

    if(owner->_ctx.cmd->state == infantry2_gimbal_cmd_t::state_t::MANUAL)
        change_state(&_manual_state);
    else if(owner->_ctx.cmd->state == infantry2_gimbal_cmd_t::state_t::AUTO)
        change_state(&_auto_state);
}

void infantry2_gimbal_t::fsm_active_t::on_execute(owner *owner) {
    if(owner->_ctx.cmd->state == infantry2_gimbal_cmd_t::state_t::MANUAL)
        change_state(&_manual_state);
    else if(owner->_ctx.cmd->state == infantry2_gimbal_cmd_t::state_t::AUTO)
        change_state(&_auto_state);
}

void infantry2_gimbal_t::fsm_active_t::on_exit(owner *owner) {}

} // namespace pyro