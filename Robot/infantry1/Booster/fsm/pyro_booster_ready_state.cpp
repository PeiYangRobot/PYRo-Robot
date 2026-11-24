#include "pyro_wl_booster.h"
#include "booster_config.h"
void pyro::wl_booster_t::fsm_active_t::state_ready_t::enter(owner* owner)
{

    // --- 切回位置环, 提供物理刚性防止溜弹 ---
    owner->_ctx.data.target_state.useTriggerSpeedLoopOnly = false;
    owner->_ctx.data.target_state.targetTriggerSpeed      = 0.0f;

}

void pyro::wl_booster_t::fsm_active_t::state_ready_t::execute(owner* owner)
{

    if(owner->_ctx.data.cmd_event == ShootEvent::SINGLE_FIRE)
    {
        if(owner->_ctx.data.isCalibrated)
        {
            #if HEAT_CONTROL
                // 此时有火控，判断热量是否充足
                if(owner->_module_deps.heat_control_deps->CanShootSingle())
                {
                    request_switch(&owner->_state_active._state_singlefire);
                }
            #else
                request_switch(&owner->_state_active._state_singlefire);
            #endif
        }
        else
        {
            request_switch(&owner->_state_active._state_cali_reverse);
        }
    }
    else if(owner->_ctx.data.cmd_event == ShootEvent::BURST_START)
    {
        #if HEAT_CONTROL
                // 此时有火控，判断热量是否充足
                if(owner->_module_deps.heat_control_deps->CanShootSingle())
                {
                    request_switch(&owner->_state_active._state_burstfire);
                }
        #else
            request_switch(&owner->_state_active._state_burstfire);
        #endif

    }

    owner->calculateFricCurrents();
    owner->calculateTriggerCurrents(owner->_ctx.data.target_state.useTriggerSpeedLoopOnly);

}

void pyro::wl_booster_t::fsm_active_t::state_ready_t::exit(owner* owner)
{


}