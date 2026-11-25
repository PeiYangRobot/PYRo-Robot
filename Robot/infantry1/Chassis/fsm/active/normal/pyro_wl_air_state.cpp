#include "pyro_wl_chassis.h"

#include "dsp/fast_math_functions.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

void wl_chassis_t::_update_accel_heading_frame()
{
    const float theta = _ctx.data.ins.euler_rad[1];
    const float phi   = _ctx.data.ins.euler_rad[2];
    const float sin_theta = arm_sin_f32(theta);
    const float cos_theta = arm_cos_f32(theta);
    const float sin_phi = arm_sin_f32(phi);
    const float cos_phi = arm_cos_f32(phi);
    const float ax_b = _ctx.data.ins.accel[0];
    const float ay_b = _ctx.data.ins.accel[1];
    const float az_b = _ctx.data.ins.accel[2];

    _ctx.data.airborne.accel_z_y =
        -sin_theta * ax_b
        + cos_theta * sin_phi * ay_b
        + cos_theta * cos_phi * az_b;

    const float tau = AIR_ACCEL_LPF_TAU;
    const float alpha = _ctx.data._dt / (tau + _ctx.data._dt);
    _ctx.data.airborne.accel_z_y_lpf +=
        alpha * (_ctx.data.airborne.accel_z_y -
                 _ctx.data.airborne.accel_z_y_lpf);
}

void wl_chassis_t::_calc_support_force()
{
    const float support_alpha =
        _ctx.data._dt / (SUPPORT_FORCE_LPF_TAU + _ctx.data._dt);
    for (uint8_t i = 0; i < 2; ++i)
    {
        const leg_ctx_t &leg = _ctx.data.leg[i];
        const float beta =
            leg.current_leg_rad - PI / 2.0f - _ctx.data.ins.euler_rad[1];
        const float length = std::max(leg.current_leg_length, 0.05f);
        const float sin_beta = arm_sin_f32(beta);
        const float cos_beta = arm_cos_f32(beta);
        const float beta_dot =
            leg.current_leg_radps - _ctx.data.ins.gyro[1];
        const float leg_endpoint_accel =
            _ctx.data.airborne.accel_z_y
            - leg.current_leg_accel * cos_beta
            + 2.0f * leg.current_leg_speed *
                  beta_dot * sin_beta
            + length * leg.current_leg_rad_accel * sin_beta
            + length * beta_dot * beta_dot *
                  cos_beta;

        const float length_derivative = L_WP_POLY_COEF[1] +
            2.0f * L_WP_POLY_COEF[2] * length + 3.0f * L_WP_POLY_COEF[3] * length * length;
        const float length_second_derivative = 2.0f * L_WP_POLY_COEF[2] +
            6.0f * L_WP_POLY_COEF[3] * length;
        const float body_to_leg_com = length - leg.L_wp;
        const float leg_com_accel = _ctx.data.airborne.accel_z_y -
            ((1.0f - length_derivative) * leg.current_leg_accel -
             length_second_derivative * leg.current_leg_speed * leg.current_leg_speed) * cos_beta +
            2.0f * (1.0f - length_derivative) * leg.current_leg_speed * beta_dot * sin_beta +
            body_to_leg_com * (leg.current_leg_rad_accel * sin_beta + beta_dot * beta_dot * cos_beta);
        // Endpoint inertia belongs to the wheel and distributed leg, not a
        // half-body mass. Include gravity because these accelerations exclude g.
        // current_F_L already includes the gas spring once.
        const float support_force_raw =
            leg.current_F_L  * cos_beta -
            leg.current_T_p / length * sin_beta +
            WHEEL_MASS * (leg_endpoint_accel + GRAVITY_ACCELERATION) +
            LEG_MASS * (1.0f - length_derivative) * (leg_com_accel + GRAVITY_ACCELERATION);
        _ctx.data.airborne.support_force[i] +=
            support_alpha *
                (support_force_raw + SUPPORT_FORCE_BIAS[i] -
                 _ctx.data.airborne.support_force[i]);
    }
    _ctx.data.airborne.support_force_sum =
        _ctx.data.airborne.support_force[0] +
        _ctx.data.airborne.support_force[1];
}

bool wl_chassis_t::_detect_takeoff()
{
    //支持力判据
    // const bool low_support =
    //     _ctx.data.airborne.support_force[leg_def::L] <
    //         AIR_TAKEOFF_FORCE_ON &&
    //     _ctx.data.airborne.support_force[leg_def::R] <
    //         AIR_TAKEOFF_FORCE_ON;
    const bool distance_clear =
        _ctx.data.distance_down >= AIR_TAKEOFF_DISTANCE_ON_MM;
    if (distance_clear)
    {
        if (_ctx.data.airborne.takeoff_counter <
            AIR_TAKEOFF_DEBOUNCE_TICKS)
        {
            ++_ctx.data.airborne.takeoff_counter;
        }
    }
    else
    {
        _ctx.data.airborne.takeoff_counter = 0;
    }
    return _ctx.data.airborne.takeoff_counter >=
           AIR_TAKEOFF_DEBOUNCE_TICKS;
}

bool wl_chassis_t::_detect_landing()
{
    auto &data = _ctx.data;
    auto &air = data.airborne;
    air.flight_time += data._dt;

    for (uint8_t i = 0; i < 2; ++i)
    {
        const auto &leg = data.leg[i];
        air.extension_speed_peak[i] = std::max(
            air.extension_speed_peak[i], leg.current_leg_speed);
        const bool reverse = leg.current_leg_speed < JUMP_LANDING_REVERSE_SPEED;
        const bool driven_extension = leg.current_leg_length <
            LEG_LENGTH_WALL_MAX - AIR_CONTACT_WALL_GUARD_MARGIN;
        const bool slowed = driven_extension &&
            leg.current_leg_length < MAX_LEG_LENGTH - JUMP_LANDING_LENGTH_MARGIN &&
            air.extension_speed_peak[i] > JUMP_LANDING_EXTENSION_SPEED_MIN &&
            air.extension_speed_peak[i] - leg.current_leg_speed > JUMP_LANDING_EXTENSION_SPEED_DROP;
        // Eq.93 applies only in Air. Confirm support as well, so planned
        // braking/retraction of an unloaded leg cannot become a touchdown.
        const bool candidate = air.flight_time >= AIR_CONTACT_MIN_TIME &&
            air.support_force[i] > AIR_CONTACT_FORCE_OFF && (reverse || slowed);
        // const bool candidate = air.flight_time >= AIR_CONTACT_MIN_TIME && (reverse || slowed);
        if (!air.contact[i])
        {
            air.contact_confirm_time[i] = candidate
                ? air.contact_confirm_time[i] + data._dt : 0.0f;
            if (air.contact_confirm_time[i] >= AIR_CONTACT_CONFIRM_TIME)
            {
                air.contact[i] = true;
                air.L_air_ref[i] = leg.current_leg_length;
            }
        }
        else
        {
            air.contact_release_time[i] = air.support_force[i] < AIR_CONTACT_RELEASE_FORCE
                ? air.contact_release_time[i] + data._dt : 0.0f;
            if (air.contact_release_time[i] >= AIR_CONTACT_RELEASE_TIME)
            {
                air.contact[i] = false;
                air.contact_confirm_time[i] = 0.0f;
                air.contact_release_time[i] = 0.0f;
                air.extension_speed_peak[i] = 0.0f;
            }
        }
    }
    data.jump_landing_confirm_time = std::min(
        air.contact_confirm_time[leg_def::L], air.contact_confirm_time[leg_def::R]);
    const bool distance_down =
       _ctx.data.distance_down <= AIR_TAKEOFF_DISTANCE_DOWN_MM;
    return distance_down;
    // return air.contact[leg_def::L] && air.contact[leg_def::R];
}

void wl_chassis_t::_execute_landing_recovery()
{
    if (!_ctx.data.airborne.landing_recovery)
    {
        return;
    }

    const float error =
        NORMAL_LENGTH_TARGET - _ctx.data.target_aux.L;
    const float max_step = AIR_LENGTH_RECOVERY_RATE * _ctx.data._dt;
    const float step = std::clamp(error, -max_step, max_step);
    _ctx.data.target_aux.L += step;
    _ctx.data.target_aux.dot_L =
        (_ctx.data._dt > 1.0e-5f) ? step / _ctx.data._dt : 0.0f;

    if (std::fabs(error) <= AIR_LENGTH_RECOVERY_EPSILON &&
        _ctx.data.airborne.support_force_sum > AIR_CONTACT_FORCE_OFF)
    {
        _ctx.data.target_aux.L = NORMAL_LENGTH_TARGET;
        _ctx.data.target_aux.dot_L = 0.0f;
        _ctx.data.airborne.landing_recovery = false;
    }
}

float wl_chassis_t::_landing_velocity_reference(const float commanded_velocity)
{
    auto &air = _ctx.data.airborne;
    if (!air.velocity_recovery)
    {
        return commanded_velocity;
    }

    const float error = commanded_velocity - air.forward_velocity_ref;
    const float max_step = LANDING_VELOCITY_REFERENCE_RATE * _ctx.data._dt;
    const bool reached = std::fabs(error) <= max_step;
    air.forward_velocity_ref = reached ? commanded_velocity
        : air.forward_velocity_ref + std::clamp(error, -max_step, max_step);
    // Keep filtering command changes throughout the other landing recoveries,
    // even if the velocity reference happens to match the command earlier.
    if (reached && !air.landing_recovery && !air.attitude_recovery)
    {
        air.velocity_recovery = false;
    }
    return air.forward_velocity_ref;
}

void wl_chassis_t::_execute_air_control()
{
    auto &data = _ctx.data;
    _gain_calculate();
    for (uint8_t i = 0; i < 2; ++i)
    {
        auto &leg = data.leg[i];
        const uint8_t beta_index = i == leg_def::L
                                       ? lqr_state_def::BETA_1
                                       : lqr_state_def::BETA_2;
        const uint8_t dot_beta_index = i == leg_def::L
                                           ? lqr_state_def::DOT_BETA_1
                                           : lqr_state_def::DOT_BETA_2;
        const uint8_t input_index = i == leg_def::L
                                        ? lqr_input_def::T_P1
                                        : lqr_input_def::T_P2;
        // 普通 AIR 的腿摆力矩只反馈本腿摆角 beta 和摆角速度 dot_beta，
        // 两者目标均为零，不引入 Pitch、另一条腿或其他状态。
        leg.out_T_p = std::clamp(
            -data.K[input_index][beta_index] *
                data.measured_state.data[beta_index] -
                data.K[input_index][dot_beta_index] *
                    data.measured_state.data[dot_beta_index],
            -MAX_T_P, MAX_T_P);
        // Fixed extension effort on both airborne and confirmed-contact legs.
        // No gas-spring cancellation or length PD; boundary feedback is in VMC.
        leg.target_leg_length = LEG_LENGTH_WALL_MAX;
        leg.out_F_L = _motor_torque_to_leg_force(leg, AIR_EXTENSION_MOTOR_TORQUE);
        // Hold zero wheel speed in Air, regardless of contact state.
        // data.wheel[i].out_T_w = std::clamp(
        //     -AIR_WHEEL_LOCK_K * data.wheel[i].current_radps, -MAX_T_W, MAX_T_W);
        // data.wheel[i].out_T_w = 0.0f;
        data.wheel[i].out_current = std::clamp(
            data.wheel[i].out_T_w / K_t, -MAX_CURRENT, MAX_CURRENT);
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_air_t::enter(wl_chassis_t *owner)
{
    owner->_ctx.data.airborne.velocity_recovery = false;
    owner->_ctx.data.airborne.landing_counter = 0;
    owner->_reset_air_contacts();
    owner->_ctx.data.airborne.forward_velocity_ref = owner->_ctx.data.measured_state.dot_x;
    owner->_ctx.data.airborne.L_ref =
        0.5f * (owner->_ctx.data.leg[leg_def::L].current_leg_length +
                owner->_ctx.data.leg[leg_def::R].current_leg_length);
    for (uint8_t i = 0; i < 2; ++i)
    {
        owner->_ctx.data.airborne.L_air_ref[i] =
            owner->_ctx.data.leg[i].current_leg_length;
        owner->_ctx.data.leg[i].target_leg_length =
            owner->_ctx.data.airborne.L_air_ref[i];
        owner->_ctx.data.leg[i].out_F_L = 0.0f;
        owner->_ctx.data.leg[i].out_T_p = 0.0f;
        owner->_ctx.data.wheel[i].out_T_w = 0.0f;
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_air_t::execute(wl_chassis_t *owner)
{
    const bool landed = owner->_detect_landing();
    owner->_execute_air_control();
    owner->_vmc_trans_v2j(false);
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
    if (landed)
    {
        owner->_ctx.data.airborne.landing_recovery = true;
        owner->_ctx.data.airborne.L_ref =
            0.5f * (owner->_ctx.data.leg[leg_def::L].current_leg_length +
                    owner->_ctx.data.leg[leg_def::R].current_leg_length);
        owner->_ctx.data.target_aux.L =
            owner->_ctx.data.airborne.L_ref;
        owner->_ctx.data.target_aux.dot_L = 0.0f;
        owner->_ctx.data.airborne.takeoff_counter = 0;
        owner->_ctx.data.airborne.landing_counter = 0;
        request_switch(&owner->_state_active._state_normal._state_balance);
        return;
    }

}

void wl_chassis_t::fsm_active_t::state_normal_t::state_air_t::exit(wl_chassis_t *owner)
{
    for (auto &wheel : owner->_ctx.data.wheel)
    {
        wheel.out_T_w = 0.0f;
        wheel.out_current = 0.0f;
    }
}

} // namespace pyro
