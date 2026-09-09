#include "pyro_infantry2_chassis.h"

namespace pyro {

void infantry2_chassis_t::state_passive_t::enter(owner *owner) {
    for(int i = 0; i < 4; i++) {
        owner->_ctx.deps.motor.rudder[i]->disable();
        owner->_ctx.deps.motor.wheel[i]->disable();

        owner->_ctx.deps.pid.rud_pos_pid[i]->clear();
        owner->_ctx.deps.pid.rud_spd_pid[i]->clear();
        owner->_ctx.deps.pid.wheel_pid[i]->clear();
    }

    owner->_ctx.deps.pid.yaw_follow_pid->clear();
}

void infantry2_chassis_t::state_passive_t::execute(owner *owner) {
    for(int i = 0; i < 4; i++) {
        owner->_ctx.data.out_rudder_torque[i] = 0;
        owner->_ctx.data.out_wheel_torque[i] = 0;
    }

    // 防跳变
    owner->_ctx.data.target_states = owner->_ctx.data.current_states;
}

void infantry2_chassis_t::state_passive_t::exit(owner *owner) {}

} // namespace pyro