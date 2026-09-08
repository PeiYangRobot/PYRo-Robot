#include "pyro_infantry2_booster.h"

namespace pyro {

void infantry2_booster_t::state_passive_t::enter(owner *owner) {
    owner->_ctx.deps.motor.fric[0]->disable();
    owner->_ctx.deps.motor.fric[1]->disable();
    owner->_ctx.deps.motor.trigger->disable();
    
    owner->_ctx.deps.pid.bullet_spd_pid->clear();
    owner->_ctx.deps.pid.fric_pid[0]->clear();
    owner->_ctx.deps.pid.fric_pid[1]->clear();
    owner->_ctx.deps.pid.trigger_pos_pid->clear();
    owner->_ctx.deps.pid.trigger_spd_pid->clear();

    owner->_ctx.data.is_calibrated = false;
}

void infantry2_booster_t::state_passive_t::execute(owner *owner) {
    owner->_ctx.data.out_fric_torque[0] = 0;
    owner->_ctx.data.out_fric_torque[1] = 0;
    owner->_ctx.data.out_trigger_torque = 0;

    //防跳变
    owner->_ctx.data.target_trigger_rad = owner->_ctx.data.current_trigger_rad;

    _send_fric_command(&owner->_ctx);
    _send_trigger_command(&owner->_ctx);
}

void infantry2_booster_t::state_passive_t::exit(owner *owner) {}

} // namespace pyro