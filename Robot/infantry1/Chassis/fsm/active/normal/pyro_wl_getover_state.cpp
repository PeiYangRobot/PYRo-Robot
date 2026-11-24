#include "pyro_wl_chassis.h"

#include <algorithm>

namespace pyro
{
void wl_chassis_t::fsm_active_t::state_normal_t::state_getover_t::enter(owner *owner)
{
    owner->_getover_elapsed = 0.0f;
    for (uint8_t i = 0; i < 2; ++i)
    {
        const float current_length =
            owner->_ctx.data.leg[i].current_leg_length;
        const float target_length =
            current_length > GETOVER_MIN_TARGET_LENGTH
                ? std::max(current_length - GETOVER_SHORTEN_LENGTH,
                           GETOVER_MIN_TARGET_LENGTH)
                : current_length;
        owner->_getover_target_length[i] =
            std::clamp(target_length, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    }
    owner->_ctx.data.current_function = chassis_function_state_t::GETOVER;
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_getover_t::execute(owner *owner)
{
    if (owner->_current_cmd.cmd_function_state != chassis_function_state_t::NONE)
    {
        request_switch(&owner->_state_active._state_normal._state_balance);
        return;
    }

    owner->_getover_elapsed += owner->_ctx.data._dt;
    const bool legs_reached_target =
        owner->_ctx.data.leg[leg_def::L].current_leg_length <=
            owner->_getover_target_length[leg_def::L]  &&
        owner->_ctx.data.leg[leg_def::R].current_leg_length <=
            owner->_getover_target_length[leg_def::R] ;
    if (legs_reached_target || owner->_getover_elapsed >= GETOVER_TIMEOUT)
    {
        owner->_ctx.data.target_aux.L = NORMAL_LENGTH_TARGET;
        owner->_ctx.data.target_aux.dot_L = 0.0f;
        owner->_ctx.data.target_state.theta = 0.0f;
        owner->_ctx.data.target_state.dot_theta = 0.0f;
        request_switch(&owner->_state_active._state_normal._state_balance);
        return;
    }

    owner->_ctx.data.target_state.theta = GETOVER_FORWARD_RAD;
    owner->_ctx.data.target_state.dot_theta = 0.0f;
    owner->_ctx.data.target_aux.L =
        0.5f * (owner->_getover_target_length[leg_def::L] +
                owner->_getover_target_length[leg_def::R]);
    owner->_ctx.data.target_aux.dot_L = 0.0f;

    owner->_gain_calculate();
    owner->_balance_control();

    /* Replace leg-length force and wheel torque before the single send. */
    for (uint8_t i = 0; i < 2; ++i)
    {
        auto &leg = owner->_ctx.data.leg[i];
        leg.target_leg_length = owner->_getover_target_length[i];
        const float total_force = GETOVER_FORCE_SCALE * owner->_ctx.pid.leg_length[i]->calculate(
            leg.target_leg_length, leg.current_leg_length, leg.current_leg_speed);
        leg.out_F_L = std::clamp(
            owner->_motor_leg_force_from_total(leg, total_force), -MAX_F_L, MAX_F_L);

        auto &wheel = owner->_ctx.data.wheel[i];
        wheel.out_T_w = std::clamp(
            wheel.out_T_w + 2.0f,
            -MAX_T_W, MAX_T_W);
        wheel.out_current = std::clamp(
            wheel.out_T_w / K_t, -MAX_CURRENT, MAX_CURRENT);
    }
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_getover_t::exit(owner *owner)
{
    owner->_getover_elapsed = 0.0f;
    owner->_getover_target_length[leg_def::L] = NORMAL_LENGTH_TARGET;
    owner->_getover_target_length[leg_def::R] = NORMAL_LENGTH_TARGET;
    owner->_ctx.data.target_aux.L = NORMAL_LENGTH_TARGET;
    owner->_ctx.data.target_aux.dot_L = 0.0f;
    owner->_ctx.data.target_state.theta = 0.0f;
    owner->_ctx.data.target_state.dot_theta = 0.0f;
    owner->_ctx.data.current_function = chassis_function_state_t::NONE;
}
}
