#include "pyro_infantry2_chassis.h"

namespace pyro {

void infantry2_chassis_t::fsm_active_t::on_enter(owner *owner) {
    // 防跳变
    owner->_ctx.data.target_states = owner->_ctx.data.current_states;

    for(int i = 0; i < 4; i++) {
        owner->_ctx.deps.pid.rud_pos_pid[i]->clear();
        owner->_ctx.deps.pid.rud_spd_pid[i]->clear();
        owner->_ctx.deps.pid.wheel_pid[i]->clear();

        owner->_ctx.deps.motor.rudder[i]->enable();
        owner->_ctx.deps.motor.wheel[i]->enable();
    }
    owner->_ctx.deps.pid.yaw_follow_pid->clear();

    if(owner->_ctx.cmd->state == infantry2_chassis_cmd_t::state_t::FOLLOW_YAW)
        change_state(&_follow_yaw_state);
    else if(owner->_ctx.cmd->state == infantry2_chassis_cmd_t::state_t::SPIN)
        change_state(&_spin_state);
    else if (owner->_ctx.cmd->state == infantry2_chassis_cmd_t::state_t::NORMAL)
        change_state(&_normal_state);
}

void infantry2_chassis_t::fsm_active_t::on_execute(owner *owner) {
    if(owner->_ctx.cmd->state == infantry2_chassis_cmd_t::state_t::FOLLOW_YAW)
        change_state(&_follow_yaw_state);
    else if(owner->_ctx.cmd->state == infantry2_chassis_cmd_t::state_t::SPIN)
        change_state(&_spin_state);
    else if (owner->_ctx.cmd->state == infantry2_chassis_cmd_t::state_t::NORMAL)
        change_state(&_normal_state);
}

void infantry2_chassis_t::fsm_active_t::on_exit(owner *owner) {}

} // namespace pyro