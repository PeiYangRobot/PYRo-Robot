#include "pyro_wl_chassis.h"
#include "stm32h7xx_hal_rcc_ex.h"
#include <algorithm>
#include <cmath>
#include <limits>

float debug = 0;
float debug2 = 0;
namespace pyro
{

static int reset_count;

namespace
{
// Advance an exact acceleration/cruise/braking trajectory by one control tick.
// Replanning from the previous reference also follows a moving beta trim without
// resetting its velocity. References finish at the goal with zero velocity.
bool advance_landing_angle_reference(const float goal, const float max_velocity,
                                     const float max_acceleration, const float dt,
                                     float &angle, float &velocity, float &acceleration)
{
    acceleration = 0.0f;
    if (!std::isfinite(dt) || dt <= 0.0f)
    {
        return false;
    }

    const double q0 = angle;
    const double v0 = velocity;
    const double distance = static_cast<double>(goal) - q0;
    const double acceleration_limit = max_acceleration;
    const double velocity_limit = max_velocity;
    double duration = std::numeric_limits<double>::infinity();
    double a = 0.0, peak = 0.0, accelerating_time = 0.0, cruise_time = 0.0;
    for (const double sign : {-1.0, 1.0})
    {
        const double signed_a = sign * acceleration_limit;
        const double square = signed_a * distance + 0.5 * v0 * v0;
        if (square < 0.0)
        {
            continue;
        }
        double candidate_peak = sign * std::sqrt(square);
        double first = (candidate_peak - v0) / signed_a;
        double coast = 0.0;
        double last = candidate_peak / signed_a;
        if (std::fabs(candidate_peak) > velocity_limit)
        {
            candidate_peak = sign * velocity_limit;
            first = (candidate_peak - v0) / signed_a;
            last = candidate_peak / signed_a;
            const double consumed = (2.0 * candidate_peak * candidate_peak - v0 * v0) /
                                    (2.0 * signed_a);
            coast = (distance - consumed) / candidate_peak;
        }
        if (first < -1.0e-10 || coast < -1.0e-10 || last < -1.0e-10)
        {
            continue;
        }
        first = std::max(0.0, first);
        coast = std::max(0.0, coast);
        last = std::max(0.0, last);
        const double candidate_duration = first + coast + last;
        if (candidate_duration < duration)
        {
            duration = candidate_duration;
            a = signed_a;
            peak = candidate_peak;
            accelerating_time = first;
            cruise_time = coast;
        }
    }
    if (!std::isfinite(duration))
    {
        return false;
    }

    const double t = dt;
    const bool finished = t >= duration;
    if (finished)
    {
        angle = goal;
        velocity = 0.0f;
    }
    else if (t < accelerating_time)
    {
        angle = static_cast<float>(q0 + v0 * t + 0.5 * a * t * t);
        velocity = static_cast<float>(v0 + a * t);
    }
    else
    {
        const double after_acceleration = t - accelerating_time;
        const double q1 = q0 + v0 * accelerating_time +
                          0.5 * a * accelerating_time * accelerating_time;
        if (after_acceleration < cruise_time)
        {
            angle = static_cast<float>(q1 + peak * after_acceleration);
            velocity = static_cast<float>(peak);
        }
        else
        {
            const double braking_time = after_acceleration - cruise_time;
            angle = static_cast<float>(q1 + peak * cruise_time + peak * braking_time -
                                       0.5 * a * braking_time * braking_time);
            velocity = static_cast<float>(peak - a * braking_time);
        }
    }
    acceleration = static_cast<float>((static_cast<double>(velocity) - v0) / t);
    return finished;
}
} // namespace


void wl_chassis_t::fsm_active_t::state_normal_t::state_balance_t::enter(wl_chassis_t *owner)
{
    owner->_ctx.data.roll_pid = {};
    owner->_ctx.data.mean_leg_length_pid = {};
    owner->_ctx.data.airborne.attitude_recovery = owner->_ctx.data.airborne.landing_recovery;
    owner->_ctx.data.airborne.velocity_recovery = owner->_ctx.data.airborne.landing_recovery;
    owner->_ctx.data.airborne.pitch_reference_acceleration = 0.0f;
    owner->_ctx.data.airborne.beta_reference_acceleration[0] = 0.0f;
    owner->_ctx.data.airborne.beta_reference_acceleration[1] = 0.0f;
    owner->_ctx.data.odom.real_x            = 0;
    owner->_reset_forward_kf();

    owner->_ctx.data.target_state.x         = 0;
    owner->_ctx.data.target_state.dot_x = owner->_ctx.data.airborne.velocity_recovery
        ? owner->_ctx.data.airborne.forward_velocity_ref : 0.0f;
    #if Using_Gimbal_Cmd
    owner->_ctx.data.target_state.psi       = 0.0f;
    #else
    owner->_ctx.data.target_state.psi       = owner->_ctx.data.ins.euler_rad[0];
    #endif
    owner->_ctx.data.target_state.dot_psi   = 0.0f;
    owner->_ctx.data.target_aux.L =
        owner->_ctx.data.airborne.landing_recovery
            ? owner->_ctx.data.airborne.L_ref
            : NORMAL_LENGTH_TARGET;
    owner->_ctx.data.target_aux.dot_L     = 0.0f;
    owner->_ctx.data.target_state.theta     = 0.0f;
    owner->_ctx.data.target_state.dot_theta = 0.0f;
    owner->_ctx.data.target_aux.phi       = 0.0f;
    owner->_ctx.data.target_aux.dot_phi   = 0.0f;
    owner->_ctx.data.target_state.beta1     = 0.0f;
    owner->_ctx.data.target_state.beta2     = 0.0f;
    owner->_ctx.data.target_state.dot_beta1 = 0.0f;
    owner->_ctx.data.target_state.dot_beta2 = 0.0f;
    if (owner->_ctx.data.airborne.attitude_recovery)
    {
        auto &data = owner->_ctx.data;
        data.target_state.theta = data.measured_state.theta;
        data.target_state.beta1 = data.measured_state.beta1;
        data.target_state.beta2 = data.measured_state.beta2;
    }


    for (float & i : owner->_ctx.data.U0)
    {
        i = 0.0f;
    }

    float avg_length = (owner->_ctx.data.leg[leg_def::L].current_leg_length +
                        owner->_ctx.data.leg[leg_def::R].current_leg_length) *
                       0.5f;
    for (auto &leg : owner->_ctx.data.leg)
    {
        leg.target_leg_rad                    = leg.current_leg_rad;
        leg.target_leg_speed                  = leg.current_leg_speed;
        leg.target_leg_radps                  = leg.current_leg_radps;
        leg.target_leg_length                 = avg_length;
        leg.out_F_L                           = 0;
        leg.out_T_p                           = 0;
        leg.out_joint_torque[joint_def::HIP]  = 0;
        leg.out_joint_torque[joint_def::KNEE] = 0;
    }
    // owner->_ctx.pid.leg_length[leg_def::L]->clear();
    // owner->_ctx.pid.leg_length[leg_def::R]->clear();


    owner->_ctx.motor.wheel[leg_def::L]->enable();
    owner->_ctx.motor.wheel[leg_def::R]->enable();
    owner->_ctx.data.airborne.takeoff_counter = 0;
    owner->_ctx.data.airborne.landing_counter = 0;
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_balance_t::execute(wl_chassis_t *owner)
{
    //紧急下力判断
    if(abs(owner->_ctx.data.ins.euler_rad[1]) >= PI / 4.0f ||
       abs(owner->_ctx.data.ins.euler_rad[2]) >= PI / 9.0f)
    {
        if(reset_count >= 50)
        {
            owner->_ctx.data.flag.leg_is_should_restart = true;
            owner->_ctx.data.flag.chassis_is_align_ready =false;
        }
        reset_count++;
    }
    else
    {
        reset_count = 0;
    }
    // 过坎
    owner->_getover_rearm_elapsed = std::min(
        owner->_getover_rearm_elapsed + owner->_ctx.data._dt,
        GETOVER_REARM_DELAY);
    const float beta[2] = {owner->_ctx.data.measured_state.beta1,
                           owner->_ctx.data.measured_state.beta2};
    const auto leg_hits_obstacle = [&](const uint8_t i) {
        const auto &leg = owner->_ctx.data.leg[i];
        return std::fabs(leg.current_leg_rad_accel) > LEG_ACCEL_THRESHOLD &&
               beta[i] < -GETOVER_BETA_THRESHOLD;
    };
    if (owner->_current_cmd.getover_detection_enabled &&
        owner->_getover_rearm_elapsed >= GETOVER_REARM_DELAY &&
        (leg_hits_obstacle(leg_def::L) || leg_hits_obstacle(leg_def::R)))
    {
        owner->_getover_rearm_elapsed = 0.0f;
        request_switch(&owner->_state_active._state_normal._state_getover);
        return;
    }

    //自动上台阶判断

    static uint16_t auto_step_count = 0;
    static int press_forward_time = 0;
    //只有在按下前行1s后才可以进;
    if(owner->_ctx.data.target_state.dot_x > 0)
    {
        press_forward_time++;
    }
    else
    {
        press_forward_time = 0;
    }

    debug = owner->_ctx.data.measured_state.dot_beta1;
    debug2 = owner->_ctx.data.leg[leg_def::R].current_leg_rad;

    if (owner->_ctx.data.measured_aux.L >= 0.30f &&
        (owner->_ctx.data.leg[leg_def::R].current_leg_rad - owner->_ctx.data.measured_state.theta)< 1.2f &&
        (owner->_ctx.data.leg[leg_def::L].current_leg_rad - owner->_ctx.data.measured_state.theta)< 1.2f &&
        press_forward_time >= 200)
    {
        if(auto_step_count >= 100)
        {
            request_switch(&owner->_state_active._state_normal._state_step);
        }
        auto_step_count++;
    }
    else
    {
        auto_step_count = 0;
    }

    // 离地检测：支撑力降低且 STP-23L 测得离地距离达到阈值。
    if (!owner->_ctx.data.airborne.landing_recovery &&
        owner->_detect_takeoff())
    {
        owner->_ctx.data.airborne.takeoff_counter = 0;
        owner->_ctx.data.airborne.landing_counter = 0;
        request_switch(&owner->_state_active._state_normal._state_air);
        return;
    }

    //落地回复
    if (owner->_ctx.data.airborne.landing_recovery)
    {
        owner->_execute_landing_recovery();
    }
    else
    {
        //腿长加上遥控器的小量
        const float target_dot_L = owner->_current_cmd.dot_L;
        owner->_ctx.data.target_aux.dot_L = target_dot_L;
        owner->_ctx.data.target_aux.L +=
        target_dot_L * owner->_ctx.data._dt;
        //腿长限幅
        owner->_ctx.data.target_aux.L =
        std::clamp(owner->_ctx.data.target_aux.L,
                       MIN_LEG_LENGTH, MAX_LEG_LENGTH);
        // Once the length reference reaches a stop, do not keep requesting
        // velocity farther into that stop. Motion back into range stays enabled.
        if ((owner->_ctx.data.target_aux.L <= MIN_LEG_LENGTH && target_dot_L < 0.0f) ||
            (owner->_ctx.data.target_aux.L >= MAX_LEG_LENGTH && target_dot_L > 0.0f))
        {
            owner->_ctx.data.target_aux.dot_L = 0.0f;
        }
    }


    auto &data = owner->_ctx.data;
    // 限制前进速度参考的上升速率，松杆减速和倒车直接跟随遥控指令。
    const float max_forward_vx = std::max(0.0f, data.target_state.dot_x) +
                                 BALANCE_FORWARD_ACCEL_LIMIT * data._dt;
    const float commanded_vx = owner->_current_cmd.v;
    const float target_vx = data.airborne.velocity_recovery
        ? owner->_landing_velocity_reference(commanded_vx)
        : std::min(commanded_vx, max_forward_vx);
    data.target_state.x += target_vx * data._dt;
    data.target_state.dot_x = target_vx;

    #if Using_Gimbal_Cmd
    //使用云台VT03

    //角速度设置

    float target_wz;
    target_wz = owner->_current_cmd.wz;
    if(fabs(target_wz) <= 0.1f)
    {
        //底盘跟随状态
        owner->_ctx.data.target_state.psi = 0.0f;
        owner->_ctx.data.target_state.dot_psi = 0.0f;
    }
    else
    {
        //小陀螺状态
        owner->_ctx.data.target_state.psi = owner->_ctx.data.measured_state.psi;
        owner->_ctx.data.target_state.psi = loop_fp32_constrain(owner->_ctx.data.target_state.psi,-PI,PI);
        owner->_ctx.data.target_state.dot_psi = target_wz;
    }


    #else

    //使用底盘dr16
    //角速度设置
    float target_wz;
    target_wz =owner->_current_cmd.wz;

    owner->_ctx.data.target_state.psi    += target_wz * owner->_ctx.data._dt;
    owner->_ctx.data.target_state.psi     = loop_fp32_constrain(owner->_ctx.data.target_state.psi,-PI,PI);
    owner->_ctx.data.target_state.dot_psi = target_wz;

    #endif


    const float previous_beta[2] = {data.target_state.beta1, data.target_state.beta2};
    const float previous_beta_speed[2] = {data.target_state.dot_beta1, data.target_state.dot_beta2};
    data.airborne.pitch_reference_acceleration = 0.0f;
    data.airborne.beta_reference_acceleration[0] = 0.0f;
    data.airborne.beta_reference_acceleration[1] = 0.0f;
    owner->_gain_calculate();
    if (data.airborne.attitude_recovery)
    {
        bool restored = advance_landing_angle_reference(
            0.0f, LANDING_PITCH_REFERENCE_RATE, LANDING_PITCH_REFERENCE_ACCELERATION,
            data._dt, data.target_state.theta, data.target_state.dot_theta,
            data.airborne.pitch_reference_acceleration);
        float *beta_ref[2] = {&data.target_state.beta1, &data.target_state.beta2};
        float *beta_speed_ref[2] = {&data.target_state.dot_beta1, &data.target_state.dot_beta2};
        for (uint8_t i = 0; i < 2; ++i)
        {
            const float trim = *beta_ref[i]; // _gain_calculate() schedules the current trim.
            *beta_ref[i] = previous_beta[i];
            *beta_speed_ref[i] = previous_beta_speed[i];
            const bool beta_restored = advance_landing_angle_reference(
                trim, LANDING_BETA_REFERENCE_RATE, LANDING_BETA_REFERENCE_ACCELERATION,
                data._dt, *beta_ref[i], *beta_speed_ref[i],
                data.airborne.beta_reference_acceleration[i]);
            restored &= beta_restored;
        }
        if (restored && !data.airborne.landing_recovery)
        {
            data.airborne.attitude_recovery = false;
            data.target_state.dot_theta = 0.0f;
            data.target_state.dot_beta1 = data.target_state.dot_beta2 = 0.0f;
        }
    }
    owner->_update_forward_kf();
    owner->_balance_control(true, true);
    owner->_vmc_trans_v2j(false);
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_balance_t::exit(wl_chassis_t *owner)
{
    owner->_ctx.data.roll_pid = {};
    owner->_ctx.data.mean_leg_length_pid = {};
    owner->_ctx.data.airborne.velocity_recovery = false;
    owner->_ctx.data.airborne.pitch_reference_acceleration = 0.0f;
    owner->_ctx.data.airborne.beta_reference_acceleration[0] = 0.0f;
    owner->_ctx.data.airborne.beta_reference_acceleration[1] = 0.0f;
}

} // namespace pyro
