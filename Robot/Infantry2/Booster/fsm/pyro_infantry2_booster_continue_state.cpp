#include "pyro_infantry2_booster.h"

namespace pyro {

void infantry2_booster_t::fsm_active_t::state_continue_t::enter(owner *owner) {
    owner->_ctx.data.is_calibrated = false;
    owner->_ctx.data.trigger_mode = infantry2_booster_data_t::trigger_pid_mode_t::SPD;
    owner->_ctx.data.target_trigger_radps = infantry2_booster::TRIGGER_CONTINUOUS_RADPS;
}

void infantry2_booster_t::fsm_active_t::state_continue_t::execute(owner *owner) {
    if(!owner->_ctx.cmd->is_fric_on) {
        request_switch(&owner->_active_state._waiting_state);

        owner->_ctx.data.target_trigger_rad = owner->_ctx.data.current_trigger_rad;
        _trigger_control(&owner->_ctx);
        _send_trigger_command(&owner->_ctx);

        return;
    }

    _trigger_control(&owner->_ctx);
    _send_trigger_command(&owner->_ctx);

    //速度环时的堵转检测
    // if(std::fabs(owner->_ctx.data.current_trigger_radps - owner->_ctx.data.target_trigger_radps)
    //         >= owner->_ctx.data.target_trigger_radps * infantry2_booster::BLOCK_SPD_ERROR_RATE_THRESHOLD) {
    //     if(owner->_ctx.data.block_start_tick == 0)
    //         owner->_ctx.data.block_start_tick = xTaskGetTickCount();
    //     else if(xTaskGetTickCount() - owner->_ctx.data.block_start_tick
    //             >= pdMS_TO_TICKS(infantry2_booster::BLOCK_TIME_THRESHOLD)) {
    //         owner->_ctx.data.is_calibrated = false;
    //         request_switch(&owner->_active_state._waiting_state);
    //     }
    // } else {
    //     owner->_ctx.data.block_start_tick = 0;
    // }

    if(!owner->_ctx.cmd->fire_licence || !owner->_ctx.cmd->continue_shoot) {
        if(_is_fric_ready(&owner->_ctx))
            request_switch(&owner->_active_state._ready_state);
        else
            request_switch(&owner->_active_state._waiting_state);
    }
}

void infantry2_booster_t::fsm_active_t::state_continue_t::exit(owner *owner) {
    owner->_ctx.data.trigger_mode = infantry2_booster_data_t::trigger_pid_mode_t::POS;
    owner->_ctx.data.target_trigger_rad = owner->_ctx.data.current_trigger_rad;
}

} // namespace pyro