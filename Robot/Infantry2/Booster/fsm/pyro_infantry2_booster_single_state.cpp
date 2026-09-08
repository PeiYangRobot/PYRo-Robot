#include "pyro_infantry2_booster.h"

namespace pyro {

void infantry2_booster_t::fsm_active_t::state_single_t::enter(owner *owner) {
    owner->_ctx.data.trigger_mode = infantry2_booster_data_t::trigger_pid_mode_t::POS;
    owner->_ctx.data.target_trigger_rad += infantry2_booster::SINGLE_BULLET_RAD+0.3f;
}

void infantry2_booster_t::fsm_active_t::state_single_t::execute(owner *owner) {
    if(!owner->_ctx.cmd->is_fric_on) {
        request_switch(&owner->_active_state._waiting_state);

        owner->_ctx.data.target_trigger_rad = owner->_ctx.data.current_trigger_rad;
        _trigger_control(&owner->_ctx);
        _send_trigger_command(&owner->_ctx);

        return;
    }
    
    _trigger_control(&owner->_ctx);
    _send_trigger_command(&owner->_ctx);

    //位置环时的堵转检测
    if(std::fabs(owner->_ctx.data.current_trigger_rad - owner->_ctx.data.target_trigger_rad) 
            > infantry2_booster::BLOCK_RAD_THRESHOLD &&
        std::fabs(owner->_ctx.data.current_trigger_radps) 
            < infantry2_booster::BLOCK_SPD_THRESHOLD) {
        if(owner->_ctx.data.block_start_tick == 0)
            owner->_ctx.data.block_start_tick = xTaskGetTickCount();
        else if(xTaskGetTickCount() - owner->_ctx.data.block_start_tick
                >= pdMS_TO_TICKS(infantry2_booster::BLOCK_TIME_THRESHOLD)) {
            owner->_ctx.data.is_calibrated = false;
            request_switch(&owner->_active_state._cali_reverse_state);
        }
    } else {
        owner->_ctx.data.block_start_tick = 0;
    }

    if(std::fabs(owner->_ctx.data.current_trigger_rad - owner->_ctx.data.target_trigger_rad)
            < infantry2_booster::TRIGGER_RAD_TOLERANCE
        || std::max(std::fabs(owner->_ctx.data.out_fric_torque[0]), std::fabs(owner->_ctx.data.out_fric_torque[1]))
             >= infantry2_booster::FRIC_SHOOT_TORQUE_THRESHOLD) {
        if(_is_fric_ready(&owner->_ctx))
            request_switch(&owner->_active_state._ready_state);
        else
            request_switch(&owner->_active_state._waiting_state);
    }
}

void infantry2_booster_t::fsm_active_t::state_single_t::exit(owner *owner) {}

} // namespace pyro