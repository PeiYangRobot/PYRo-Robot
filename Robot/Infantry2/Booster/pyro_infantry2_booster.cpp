#include "pyro_infantry2_booster.h"
#include "pyro_algo_common.h"

namespace pyro {

infantry2_booster_t::infantry2_booster_t()
        : module_base_t("infantry2_booster", 512, 512, task_base_t::priority_t::HIGH) {
    _ctx.data={};
}

status_t infantry2_booster_t::_init() {
    _ctx.deps = _module_deps;
    return PYRO_OK;
}

void infantry2_booster_t::notify_single_shoot() {
    taskENTER_CRITICAL();
    _ctx.data.notify_ev |= EVENT_BIT_SINGLE_SHOOT;
    taskEXIT_CRITICAL();
}

void infantry2_booster_t::_update_feedback() {
    // 电机反馈更新
    _ctx.deps.motor.fric[0]->update_feedback();
    _ctx.deps.motor.fric[1]->update_feedback();
    _ctx.deps.motor.trigger->update_feedback();

    _ctx.data.current_fric_radps[0] = _ctx.deps.motor.fric[0]->get_current_rotate();
    _ctx.data.current_fric_radps[1] = _ctx.deps.motor.fric[1]->get_current_rotate();

    _ctx.data.current_trigger_radps =
        _ctx.deps.motor.trigger->get_current_rotate() / infantry2_booster::TRIGGER_REDUCTION_RATIO;

    _ctx.data.current_trigger_torque = _ctx.deps.motor.trigger->get_current_torque();

    const float now_trigger_rad = _ctx.deps.motor.trigger->get_current_position();

    float delta_rad = now_trigger_rad - _ctx.data.last_trigger_rad;
    if (delta_rad > PI)
        delta_rad -= 2.0f * PI;
    else if (delta_rad < -PI)
        delta_rad += 2.0f * PI;

    _ctx.data.current_trigger_rad += delta_rad / infantry2_booster::TRIGGER_REDUCTION_RATIO;

    _ctx.data.last_trigger_rad = now_trigger_rad;
}

void infantry2_booster_t::_fsm_execute() {
    _ctx.cmd = &_current_cmd;
    
    if(_ctx.cmd->mode == infantry2_booster_cmd_t::mode_t::PASSIVE)
        _main_fsm.change_state(&_passive_state);
    else if(_ctx.cmd->mode == infantry2_booster_cmd_t::mode_t::ACTIVE)
        _main_fsm.change_state(&_active_state);

    _main_fsm.execute(this);
}

bool infantry2_booster_t::_is_fric_ready(infantry2_booster_ctx_t *ctx) {
    return std::fabs(std::fabs(ctx->data.current_fric_radps[0])
            - infantry2_booster::TARGET_BULLET_SPEED / infantry2_booster::FRIC_RADIUS)
            < infantry2_booster::FRIC_RADPS_TOLERANCE
        && std::fabs(std::fabs(ctx->data.current_fric_radps[1])
            - infantry2_booster::TARGET_BULLET_SPEED / infantry2_booster::FRIC_RADIUS) 
            < infantry2_booster::FRIC_RADPS_TOLERANCE;
}

void infantry2_booster_t::_fric_control(infantry2_booster_ctx_t *ctx) {
    ctx->data.out_fric_torque[0] =
        ctx->deps.pid.fric_pid[0]->calculate(ctx->data.target_fric_radps[0], ctx->data.current_fric_radps[0]);
    ctx->data.out_fric_torque[1] =
        ctx->deps.pid.fric_pid[1]->calculate(ctx->data.target_fric_radps[1], ctx->data.current_fric_radps[1]);
}

void infantry2_booster_t::_trigger_control(infantry2_booster_ctx_t *ctx) {
    if(ctx->data.trigger_mode == infantry2_booster_data_t::trigger_pid_mode_t::POS) {
        //处理过零点问题
        const float error = ctx->data.target_trigger_rad - ctx->data.current_trigger_rad;
        if (error > PI)
            ctx->data.target_trigger_rad -= 2.0f * PI;
        else if (error < -PI)
            ctx->data.target_trigger_rad += 2.0f * PI;

        // // 死区，防止由于安装间隙导致的振动
        // if(std::fabs(ctx->data.target_trigger_rad - ctx->data.current_trigger_rad)
        //         < infantry2_booster::TRIGGER_RAD_DEADZONE * infantry2_booster::TRIGGER_REDUCTION_RATIO ) {
        //     ctx->data.target_trigger_radps = 0.0f;
        //     ctx->data.out_trigger_torque =0.0f;
        //     return;
        // }

        ctx->data.target_trigger_radps =
            ctx->deps.pid.trigger_pos_pid->calculate(ctx->data.target_trigger_rad, ctx->data.current_trigger_rad);
        ctx->data.out_trigger_torque =
            ctx->deps.pid.trigger_spd_pid->calculate(ctx->data.target_trigger_radps, ctx->data.current_trigger_radps);
    }
    if(ctx->data.trigger_mode == infantry2_booster_data_t::trigger_pid_mode_t::SPD) {
        ctx->data.out_trigger_torque =
            ctx->deps.pid.trigger_spd_pid->calculate(ctx->data.target_trigger_radps, ctx->data.current_trigger_radps);
    }
}

void infantry2_booster_t::_send_fric_command(infantry2_booster_ctx_t *ctx) {
    ctx->deps.motor.fric[0]->send_torque(ctx->data.out_fric_torque[0]);
    ctx->deps.motor.fric[1]->send_torque(ctx->data.out_fric_torque[1]);
}

void infantry2_booster_t::_send_trigger_command(infantry2_booster_ctx_t *ctx) {
    ctx->deps.motor.trigger->send_torque(ctx->data.out_trigger_torque);
}

} // namespace pyro