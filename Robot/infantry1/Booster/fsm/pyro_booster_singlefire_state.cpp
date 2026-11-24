#include "pyro_wl_booster.h"
#include "pyro_algo_common.h"

void pyro::wl_booster_t::fsm_active_t::state_singlefire_t::enter(owner* owner)
{
    // --- 初始化状态 ---
    owner->_ctx.data.target_state.useTriggerSpeedLoopOnly = false;

    owner->_ctx.data.target_state.targetTriggerRad += ONE_BULLET_RAD;

    if(owner->_ctx.data.target_state.targetTriggerRad >= ONE_CIRCLE_RAD)
    {
        owner->_ctx.data.target_state.targetTriggerRad -= ONE_CIRCLE_RAD;
    }

}

void pyro::wl_booster_t::fsm_active_t::state_singlefire_t::execute(owner* owner)
{
    float err = wrap2pi_f32_normalized
        ((owner->_ctx.data.target_state.targetTriggerRad - owner->_ctx.data.motor_state.trigger_rad) /
                (float)TRIGGER_MOTOR_REDUCTION_RATIO);

    //堵转检测
    static uint16_t blocked_count = 0;
    if((err >= PI / 16.0f) && (fabs(owner->_ctx.data.motor_state.trigger.vel) <= 10.0f))
    {
        if(blocked_count >= 800)
        {
            request_switch(&owner->_state_active._state_cali_reverse);
            blocked_count = 0;
        }
        blocked_count++;
    }
    else
    {
        blocked_count = 0;
    }

    //到达目标 , 回到ready
    static int steady_count = 0;
    if(err <= PI / 16.0f)
    {
        if(steady_count >= 10)
        {
            #if HEAT_CONTROL
                owner->_module_deps.heat_control_deps->LocalShootAdd();
            #endif
            request_switch(&owner->_state_active._state_ready);
            steady_count = 0;
        }
        steady_count++;
    }
    else
    {
        steady_count = 0;
    }


    owner->calculateFricCurrents();
    owner->calculateTriggerCurrents(owner->_ctx.data.target_state.useTriggerSpeedLoopOnly);
}

void pyro::wl_booster_t::fsm_active_t::state_singlefire_t::exit(owner* owner)
{

}
