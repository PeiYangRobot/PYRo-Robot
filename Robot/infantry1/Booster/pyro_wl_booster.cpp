#include "pyro_wl_booster.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"
#include "dsp/fast_math_functions.h"


namespace pyro
{

wl_booster_t::wl_booster_t() : module_base_t("wl_booster")
{
}

status_t wl_booster_t::_init()
{
    _ctx                                      = {};
    _module_deps.motor_deps.fric1->enable();
    _module_deps.motor_deps.fric2->enable();
    _module_deps.motor_deps.trigger->enable();

    return PYRO_OK;
}

void wl_booster_t::_update_feedback()
{

    //更新电机反馈
    _module_deps.motor_deps.fric1->update_feedback();
    _module_deps.motor_deps.fric2->update_feedback();
    _module_deps.motor_deps.trigger->update_feedback();

    _ctx.data.motor_state.fric[0].online = _module_deps.motor_deps.fric1->is_online();
    _ctx.data.motor_state.fric[0].pos    = _module_deps.motor_deps.fric1->get_current_position();
    _ctx.data.motor_state.fric[0].vel    = _module_deps.motor_deps.fric1->get_current_rotate();
    _ctx.data.motor_state.fric[0].temp   = _module_deps.motor_deps.fric1->get_temperature();
    _ctx.data.motor_state.fric[0].torque = _module_deps.motor_deps.fric1->get_current_torque();


    _ctx.data.motor_state.fric[1].online   = _module_deps.motor_deps.fric2->is_online();
    _ctx.data.motor_state.fric[1].pos      = _module_deps.motor_deps.fric2->get_current_position();
    _ctx.data.motor_state.fric[1].vel      = _module_deps.motor_deps.fric2->get_current_rotate();
    _ctx.data.motor_state.fric[1].temp     = _module_deps.motor_deps.fric2->get_temperature();
    _ctx.data.motor_state.fric[1].torque   = _module_deps.motor_deps.fric2->get_current_torque();

    _ctx.data.motor_state.trigger.online   = _module_deps.motor_deps.trigger->is_online();
    _ctx.data.motor_state.trigger.pos      = _module_deps.motor_deps.trigger->get_current_position();
    _ctx.data.motor_state.trigger.vel      = _module_deps.motor_deps.trigger->get_current_rotate();
    _ctx.data.motor_state.trigger.temp     = _module_deps.motor_deps.trigger->get_temperature();
    _ctx.data.motor_state.trigger.torque   = _module_deps.motor_deps.trigger->get_current_torque();


    _ctx.data.mode                         =    _current_cmd.mode;

    static ShootEvent last_event = ShootEvent::NONE;
    if(_current_cmd.event != last_event)
    {
        _ctx.data.cmd_event                =    _current_cmd.event;
    }
    else
    {
        _ctx.data.cmd_event                =    ShootEvent::NONE;
    }
    last_event = _current_cmd.event;

    _ctx.data.target_state.burst_state     = _current_cmd.burst_state.burstShot;

    if(_ctx.data.cmd_event == ShootEvent::FRIC_TOGGLE)
    {
        _ctx.data.motor_state.enable =! _ctx.data.motor_state.enable;
    }


    //拨弹盘角度计算
    static float lastRad = _ctx.data.motor_state.trigger.pos;
    float deltaRad = _ctx.data.motor_state.trigger.pos - lastRad;
    if (deltaRad < -PI * 0.5f) {
        // 原始值突变变小，说明正向转过了零点
        _ctx.data.motor_state.triggerRound++;
        if (_ctx.data.motor_state.triggerRound >= TRIGGER_MOTOR_REDUCTION_RATIO)
        {
            _ctx.data.motor_state.triggerRound -= TRIGGER_MOTOR_REDUCTION_RATIO; // 满36圈，输出轴刚好转满一圈，圈数归零
        }
    } else if (deltaRad > PI * 0.5f) {
        // 原始值突变变大，说明反向转过了零点 (例如 10 -> 8190)
        _ctx.data.motor_state.triggerRound--;
        if (_ctx.data.motor_state.triggerRound < 0)
        {
            _ctx.data.motor_state.triggerRound += TRIGGER_MOTOR_REDUCTION_RATIO; // 退回上一圈
        }
    }
    lastRad = _ctx.data.motor_state.trigger.pos;

    _ctx.data.motor_state.trigger_rad =
                _ctx.data.motor_state.trigger.pos + _ctx.data.motor_state.triggerRound * 2 * PI;

    //热量控制与弹速补偿更新
    if(pyro::board_drv_t::get_instance().check_online())
    {
        const auto& c2g_data = pyro::board_drv_t::get_instance().get_c2g_rx_data();
        #if HEAT_CONTROL

            static int last_heat = 0;
            _module_deps.heat_control_deps->UpdateParameter(c2g_data.heat_limit, c2g_data.cooling_rate);
            _module_deps.heat_control_deps->LocalHeatCompute();
            if(c2g_data.current_heat != last_heat)
            {
                _module_deps.heat_control_deps->UpdateRefereeHeat(c2g_data.current_heat);
            }
            last_heat = c2g_data.current_heat;
        #endif

        #if BULLET_SPEED_CONTROL
            static int last_bullet_speed100 = 0;
            if(c2g_data.bullet_speed100 != last_bullet_speed100)
            {
                _module_deps.speed_control_deps->UpdateNewShootSpeed(c2g_data.bullet_speed100 * 0.01f);
                _module_deps.speed_control_deps->CalculateSpeedCompensation();
            }
            last_bullet_speed100 = c2g_data.bullet_speed100;
        #endif
    }


    static uint32_t dwtCnt;
    _ctx.data.dt = pyro::dwt_drv_t::get_delta_t(&dwtCnt);

}




void wl_booster_t::_fsm_execute()
{

    if (_ctx.data.mode == cmd_base_t::mode_t::ACTIVE && _ctx.data.motor_state.enable)
    {
        _main_fsm.change_state(&_state_active);
    }
    else
    {
        _ctx.data.motor_state.enable = 0;
        _main_fsm.change_state(&_state_passive);
    }
    _main_fsm.execute(this);

    sendCurrents();
}


void wl_booster_t::calculateFricCurrents()
{

    //加上弹速闭环补偿
    float finalFricTargetSpeed = _ctx.data.target_state.targetFricSpeed +
                    _module_deps.speed_control_deps->GetSpeedCompensation();

    float bullet_L_speed = _ctx.data.motor_state.fric[0].vel;
    float bullet_R_speed = _ctx.data.motor_state.fric[1].vel;


    // 左摩擦轮
    _ctx.data.output.fric1Current = _module_deps.pid_deps.fric1_spd->calculate(
        finalFricTargetSpeed,
        bullet_L_speed);

    // 右摩擦轮 (反向安装, 目标取反)
    _ctx.data.output.fric2Current = _module_deps.pid_deps.fric2_spd->calculate(
        -finalFricTargetSpeed,
        bullet_R_speed);


}

void wl_booster_t::calculateTriggerCurrents(bool useTriggerSpeedLoopOnly)
{
    float spdTarget = 0.0f;
    if(useTriggerSpeedLoopOnly)
    {
        //仅速度环逻辑
        spdTarget = _ctx.data.target_state.targetTriggerSpeed;
    }
    else
    {
        float err = (_ctx.data.motor_state.trigger_rad - _ctx.data.target_state.targetTriggerRad) /
                            (float)TRIGGER_MOTOR_REDUCTION_RATIO;
        while (err >  (float)M_PI) err -= 2.0f * (float)M_PI;
        while (err < -(float)M_PI) err += 2.0f * (float)M_PI;

        spdTarget = _module_deps.pid_deps.trigger_pos->calculate(0.0f, err);
    }

    _ctx.data.output.triggerCurrent = _module_deps.pid_deps.trigger_spd->
                    calculate(spdTarget, _ctx.data.motor_state.trigger.vel);
}

void wl_booster_t::sendCurrents()
{
    _module_deps.motor_deps.fric1->send_torque(_ctx.data.output.fric1Current);
    _module_deps.motor_deps.fric2->send_torque(_ctx.data.output.fric2Current);
    _module_deps.motor_deps.trigger->send_torque(_ctx.data.output.triggerCurrent);
}


}
