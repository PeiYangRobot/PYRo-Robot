#include "pyro_wl_booster.h"
#include "pyro_algo_common.h"
#include "booster_config.h"

void pyro::wl_booster_t::fsm_active_t::state_cali_reverse_t::enter(owner* owner)
{
    // --- 初始化状态 ---
    owner->_ctx.data.target_state.useTriggerSpeedLoopOnly = true;
}

void pyro::wl_booster_t::fsm_active_t::state_cali_reverse_t::execute(owner* owner)
{
    owner->_ctx.data.target_state.targetTriggerSpeed = -TRIGGER_SPEED;

    //堵转检测
    static uint16_t blocked_count = 0;
    if (fabs(owner->_ctx.data.target_state.targetTriggerSpeed) >= 50.0f &&
        fabs(owner->_ctx.data.motor_state.trigger.vel) <= 20.0f)
    {
        if(blocked_count >= 800)
        {

            owner->_ctx.data.target_state.targetTriggerRad = owner->_ctx.data.motor_state.trigger_rad;
            request_switch(&owner->_state_active._state_cali_forward);
            blocked_count = 0;
        }
        blocked_count++;
    }
    else
    {
        blocked_count = 0;
    }


    owner->calculateFricCurrents();
    owner->calculateTriggerCurrents(owner->_ctx.data.target_state.useTriggerSpeedLoopOnly);
}

void pyro::wl_booster_t::fsm_active_t::state_cali_reverse_t::exit(owner* owner)
{

}