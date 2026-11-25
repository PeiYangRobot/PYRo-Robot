#include "pyro_wl_booster.h"

void pyro::wl_booster_t::state_passive_t::enter(owner* owner)
{
    // --- 初始化状态 ---
    owner->_ctx.data.isCalibrated            = false;
    owner->_ctx.data.target_state.targetFricSpeed         = 0.0f;
    owner->_ctx.data.target_state.useTriggerSpeedLoopOnly = true;
    owner->_ctx.data.target_state.targetTriggerSpeed      =0.0f;
    owner->_module_deps.speed_control_deps->Reset();
}

void pyro::wl_booster_t::state_passive_t::execute(owner* owner)
{
    owner->_ctx.data.target_state.targetTriggerRad =  owner->_ctx.data.motor_state.trigger_rad;

    owner->_ctx.data.output.triggerCurrent = 0.0f;

    if (fabs(owner->_ctx.data.motor_state.fric[0].vel) >= 1.0f ||
        fabs(owner->_ctx.data.motor_state.fric[1].vel) >= 1.0f)
    {
        owner->calculateFricCurrents();
    }
    else
    {
        owner->_ctx.data.output.fric1Current = 0.0f;
        owner->_ctx.data.output.fric2Current = 0.0f;
    }

}

void pyro::wl_booster_t::state_passive_t::exit(owner* owner)
{

}