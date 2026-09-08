#include "pyro_infantry2_booster.h"

namespace pyro {

void infantry2_booster_t::fsm_active_t::on_enter(owner *owner) {
    owner->_ctx.deps.pid.bullet_spd_pid->clear();
    owner->_ctx.deps.pid.fric_pid[0]->clear();
    owner->_ctx.deps.pid.fric_pid[1]->clear();
    owner->_ctx.deps.pid.trigger_pos_pid->clear();
    owner->_ctx.deps.pid.trigger_spd_pid->clear();

    owner->_ctx.deps.motor.fric[0]->enable();
    owner->_ctx.deps.motor.fric[1]->enable();
    owner->_ctx.deps.motor.trigger->enable();

    change_state(&_waiting_state);
}

void infantry2_booster_t::fsm_active_t::on_execute(owner *owner) {
    if(owner->_ctx.cmd->is_fric_on) {
        owner->_ctx.data.target_fric_radps[0] =
            infantry2_booster::TARGET_BULLET_SPEED / infantry2_booster::FRIC_RADIUS;
        owner->_ctx.data.target_fric_radps[1] =
            - infantry2_booster::TARGET_BULLET_SPEED / infantry2_booster::FRIC_RADIUS;

        _fric_control(&owner->_ctx);
    } else {
        owner->_ctx.data.target_fric_radps[0] = 0.0f;
        owner->_ctx.data.target_fric_radps[1] = 0.0f;

        _fric_control(&owner->_ctx);

        if(std::fabs(owner->_ctx.data.current_fric_radps[0]) < infantry2_booster::FRIC_RADPS_DEADZONE)
            owner->_ctx.data.out_fric_torque[0] = 0.0f;
        if(std::fabs(owner->_ctx.data.current_fric_radps[1]) < infantry2_booster::FRIC_RADPS_DEADZONE)
            owner->_ctx.data.out_fric_torque[1] = 0.0f;
    }

    _send_fric_command(&owner->_ctx);
}

void infantry2_booster_t::fsm_active_t::on_exit(owner *owner) {}

} // namespace pyro