#include "pyro_wl_booster.h"
#include "booster_config.h"
void pyro::wl_booster_t::fsm_active_t::state_spin_up_t::enter(owner* owner)
{

    owner->_ctx.data.target_state.targetFricSpeed         = FRIC_TARGET_SPEED;
    owner->_ctx.data.target_state.useTriggerSpeedLoopOnly = false;

}

void pyro::wl_booster_t::fsm_active_t::state_spin_up_t::execute(owner* owner)
{

    // --- 启动完成检测: 双摩擦轮速度均接近目标 ---
    if (fabs(std::fabs(owner->_ctx.data.motor_state.fric[0].vel) - owner->_ctx.data.target_state.targetFricSpeed) < 10 &&
        fabs(std::fabs(owner->_ctx.data.motor_state.fric[1].vel) - owner->_ctx.data.target_state.targetFricSpeed) < 10)
    {
        request_switch(&owner->_state_active._state_ready);
    }

    owner->calculateFricCurrents();
    owner->calculateTriggerCurrents(owner->_ctx.data.target_state.useTriggerSpeedLoopOnly);


}

void pyro::wl_booster_t::fsm_active_t::state_spin_up_t::exit(owner* owner)
{


}