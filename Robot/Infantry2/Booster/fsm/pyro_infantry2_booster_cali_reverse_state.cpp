#include "pyro_infantry2_booster.h"

namespace pyro {

void infantry2_booster_t::fsm_active_t::state_cali_reverse_t::enter(owner *owner) {
    owner->_ctx.data.trigger_mode = infantry2_booster_data_t::trigger_pid_mode_t::SPD;
    owner->_ctx.data.target_trigger_radps = - infantry2_booster::CALI_REVERSE_RADPS;
    owner->_ctx.data.block_start_tick = 0;
}

void infantry2_booster_t::fsm_active_t::state_cali_reverse_t::execute(owner *owner) {
    if(!owner->_ctx.cmd->is_fric_on) {
        request_switch(&owner->_active_state._waiting_state);

        owner->_ctx.data.target_trigger_rad = owner->_ctx.data.current_trigger_rad;
        _trigger_control(&owner->_ctx);
        _send_trigger_command(&owner->_ctx);

        return;
    }

    _trigger_control(&owner->_ctx);
    _send_trigger_command(&owner->_ctx);

    if(std::fabs(owner->_ctx.data.current_trigger_radps - owner->_ctx.data.target_trigger_radps)
            >= std::fabs(owner->_ctx.data.target_trigger_radps * infantry2_booster::BLOCK_SPD_ERROR_RATE_THRESHOLD)) {
        if(owner->_ctx.data.block_start_tick == 0)
            owner->_ctx.data.block_start_tick = xTaskGetTickCount();
        else if(xTaskGetTickCount() - owner->_ctx.data.block_start_tick
                >= pdMS_TO_TICKS(infantry2_booster::BLOCK_TIME_THRESHOLD)) {
            request_switch(&owner->_active_state._cali_forward_state);
            // owner->_ctx.data.is_calibrated = true;
            // request_switch(&owner->_active_state._single_state);
        }
    } else {
        owner->_ctx.data.block_start_tick = 0;
    }
}

void infantry2_booster_t::fsm_active_t::state_cali_reverse_t::exit(owner *owner) {
    owner->_ctx.deps.pid.trigger_spd_pid->clear();
    owner->_ctx.data.trigger_mode = infantry2_booster_data_t::trigger_pid_mode_t::POS;
    owner->_ctx.data.target_trigger_rad = owner->_ctx.data.current_trigger_rad;
}

} // namespace pyro