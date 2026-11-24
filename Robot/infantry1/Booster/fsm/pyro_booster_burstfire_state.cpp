#include "pyro_wl_booster.h"
#include "pyro_algo_common.h"
#include "booster_config.h"

static float bullet_rad = 0; //拨弹盘到达这个角度时就会向热量控制器注册一发子弹
void pyro::wl_booster_t::fsm_active_t::state_burstfire_t::enter(owner* owner)
{
    // --- 初始化状态 ---
    owner->_ctx.data.target_state.useTriggerSpeedLoopOnly = true;
    owner->_ctx.data.isCalibrated = false;
    bullet_rad = owner->_ctx.data.motor_state.trigger_rad + ONE_BULLET_RAD;
    if(bullet_rad >= ONE_CIRCLE_RAD)
    {
        bullet_rad -= ONE_CIRCLE_RAD;
    }
}

void pyro::wl_booster_t::fsm_active_t::state_burstfire_t::execute(owner* owner)
{
    owner->_ctx.data.target_state.targetTriggerSpeed = TRIGGER_SPEED;
    owner->_ctx.data.target_state.targetTriggerRad =  owner->_ctx.data.motor_state.trigger_rad;

    //堵转检测
    static uint16_t blocked_count = 0;
    if (fabs(owner->_ctx.data.target_state.targetTriggerSpeed) >= 50.0f &&
        fabs(owner->_ctx.data.motor_state.trigger.vel) <= 10.0f)
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

    //此处作热量冷却检测
    #if HEAT_CONTROL
        //在连发过程中注册发弹事件
        if (owner->_ctx.data.motor_state.trigger_rad > bullet_rad &&
            abs(bullet_rad - owner->_ctx.data.motor_state.trigger_rad) <= ONE_CIRCLE_RAD * 0.5f)
        {
            owner->_module_deps.heat_control_deps->LocalShootAdd();
            bullet_rad = owner->_ctx.data.motor_state.trigger_rad + ONE_BULLET_RAD;
            if(bullet_rad >= ONE_CIRCLE_RAD)
            {
                bullet_rad -= ONE_CIRCLE_RAD;
            }
        }


        if (!owner->_module_deps.heat_control_deps->CanShootSingle())
        {
            request_switch(&owner->_state_active._state_ready);
        }
    #endif

    //侦测到指令结束，切回ready
    if(owner->_ctx.data.cmd_event == ShootEvent::BURST_END)
    {
        request_switch(&owner->_state_active._state_ready);
    }


    owner->calculateFricCurrents();
    owner->calculateTriggerCurrents(owner->_ctx.data.target_state.useTriggerSpeedLoopOnly);
}

void pyro::wl_booster_t::fsm_active_t::state_burstfire_t::exit(owner* owner)
{

}