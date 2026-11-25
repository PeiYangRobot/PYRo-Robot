#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"
#include "pyro_stp23l_drv.h"
#include "dsp/fast_math_functions.h"
#include "wl_config.h"
#include "pyro_board_drv.h"

#include <algorithm>
#include <cmath>

namespace pyro
{



wl_chassis_t::wl_chassis_t() : module_base_t("wl_chassis")
{
}

status_t wl_chassis_t::_init()
{
    _ctx                                      = {};
    _ctx.motor                                = _module_deps.motor;
    _ctx.pid                                  = _module_deps.pid;

    _current_cmd.delta_leg_length[leg_def::L] = 0.0f;
    _current_cmd.delta_leg_length[leg_def::R] = 0.0f;
    _current_cmd.delta_leg_rad[leg_def::L]    = 0.0f;
    _current_cmd.delta_leg_rad[leg_def::R]    = 0.0f;

    _ctx.data.leg[leg_def::L].direction       = LEFT_LEG_DIRECTION;
    _ctx.data.leg[leg_def::R].direction       = RIGHT_LEG_DIRECTION;

    _ctx.data.wheel[leg_def::L].direction     = LEFT_WHEEL_DIRECTION;
    _ctx.data.wheel[leg_def::R].direction     = RIGHT_WHEEL_DIRECTION;

    _forward_kf.init();
    return PYRO_OK;
}

void wl_chassis_t::_update_feedback()
{
    // 1. Update timing.
    static uint32_t dwt_cnt = 0;
    const float measured_dt = dwt_drv_t::get_delta_t(&dwt_cnt);
    _ctx.data._dt =
        (measured_dt > 1.0e-5f && measured_dt < 0.1f) ? measured_dt : 0.001f;

    // Snapshot the latest STP-23L distance into the chassis state so the FSM
    // and application code can use one consistent feedback value.

    _ctx.data.distance_down = stp23l_drv::down_get_instance().get_distance();
    _ctx.data.distance_front  = stp23l_drv::front_get_instance().get_distance();

    // 2. Update all motor feedback before reading cached values.
    for (uint8_t leg = 0; leg < 2; ++leg)
    {
        for (uint8_t joint = 0; joint < 2; ++joint)
        {
            _ctx.motor.joint[leg][joint]->update_feedback();
        }
        _ctx.motor.wheel[leg]->update_feedback();
    }
    _ctx.motor.yaw->update_feedback();

    // 3. Read and organize all leg-related feedback.
    constexpr float JOINT_POSITION_OFFSET[2][2] = {
        {LEFT_HIP_OFFSET, LEFT_KNEE_OFFSET},
        {RIGHT_HIP_OFFSET, RIGHT_KNEE_OFFSET},
    };

    for (uint8_t leg = 0; leg < 2; ++leg)
    {
        leg_ctx_t &leg_ctx     = _ctx.data.leg[leg];
        wheel_ctx_t &wheel_ctx = _ctx.data.wheel[leg];

        for (uint8_t joint = 0; joint < 2; ++joint)
        {
            motor_base_t *motor = _ctx.motor.joint[leg][joint];
            const float raw_rad =
                motor->get_current_position() +
                leg_ctx.direction * JOINT_POSITION_OFFSET[leg][joint];

            leg_ctx.current_joint_rad[joint] =
                leg_ctx.direction * loop_fp32_constrain(raw_rad, -PI, PI);
            leg_ctx.current_joint_torque[joint] =
                leg_ctx.direction * motor->get_current_torque();
            leg_ctx.current_joint_radps[joint] =
                leg_ctx.direction * motor->get_current_rotate();
        }

        motor_base_t *wheel_motor = _ctx.motor.wheel[leg];
        wheel_ctx.current_radps   = wheel_motor->get_current_rotate() *
                                  wheel_ctx.direction * rec_reduction_ratio;
        wheel_ctx.current_T_w = wheel_motor->get_current_torque() *
                                wheel_ctx.direction * rec_reduction_ratio;
    }

    _ctx.data.yaw.pos = loop_fp32_constrain(_ctx.motor.yaw->get_current_position() - YAW_OFFSET,-PI, PI) ;
    _ctx.data.yaw.rot = _ctx.motor.yaw->get_current_rotate();

    // 4. Convert joint-space feedback to virtual-mechanism feedback.
    _vmc_trans_j2v();

    // 5. Update wheel odometry.
    _ctx.data.odom.real_dot_x[1] = _ctx.data.odom.real_dot_x[0];
    _ctx.data.odom.real_dot_x[0] = 0.5f * WHEEL_RADIUS *
                                   (_ctx.data.wheel[leg_def::L].current_radps +
                                    _ctx.data.wheel[leg_def::R].current_radps);
    _ctx.data.odom.real_x +=
        0.5f * (_ctx.data.odom.real_dot_x[0] + _ctx.data.odom.real_dot_x[1]) *
        _ctx.data._dt;

    // 6. Read IMU feedback.
    auto ins = ins_drv_t::get_instance();
    ins->get_rads_n(&_ctx.data.ins.euler_rad[0], &_ctx.data.ins.euler_rad[1],
                    &_ctx.data.ins.euler_rad[2]);
    ins->get_gyro_b(&_ctx.data.ins.gyro[0], &_ctx.data.ins.gyro[1],
                    &_ctx.data.ins.gyro[2]);
    ins->get_accel_without_g_b(&_ctx.data.ins.accel[0],
                               &_ctx.data.ins.accel[1],
                               &_ctx.data.ins.accel[2]);

    // 7. State vector feedback.
    state_vec_t &state = _ctx.data.measured_state;

    state.x            = _ctx.data.odom.real_x;
    state.dot_x        = (_ctx.data.odom.real_dot_x[0] + _ctx.data.odom.real_dot_x[1]) / 2;

    #if Using_Gimbal_Cmd
    //更新yaw的反馈
    const auto& board_ptr = &pyro::board_drv_t::get_instance(pyro::board_drv_t::role_t::CHASSIS,pyro::bsp_can::can3);
    const auto& g2c_data = board_ptr->get_g2c_rx_data();

    state.psi          = -_ctx.data.yaw.pos;
    state.dot_psi = _ctx.data.ins.gyro[0] - g2c_data.imu_yaw_radps_100 * 0.01f;
    #else
    state.psi          = _ctx.data.ins.euler_rad[0];
    state.dot_psi      = _ctx.data.ins.gyro[0];
    #endif


    state.theta        = _ctx.data.ins.euler_rad[1];
    state.dot_theta    = _ctx.data.ins.gyro[1];
    _ctx.data.measured_aux.phi = _ctx.data.ins.euler_rad[2];
    _ctx.data.measured_aux.dot_phi = _ctx.data.ins.gyro[2];
    state.beta1        = _ctx.data.leg[leg_def::L].current_leg_rad - PI / 2 -
                  _ctx.data.ins.euler_rad[1];
    state.dot_beta1 =
        _ctx.data.leg[leg_def::L].current_leg_radps - _ctx.data.ins.gyro[1];
    state.beta2 = _ctx.data.leg[leg_def::R].current_leg_rad - PI / 2 -
                  _ctx.data.ins.euler_rad[1];
    state.dot_beta2 =
        _ctx.data.leg[leg_def::R].current_leg_radps - _ctx.data.ins.gyro[1];

    _ctx.data.measured_aux.L =
        0.5f * (_ctx.data.leg[leg_def::L].current_leg_length +
                _ctx.data.leg[leg_def::R].current_leg_length);
    _ctx.data.measured_aux.dot_L =
        0.5f * (_ctx.data.leg[leg_def::L].current_leg_speed  +
                _ctx.data.leg[leg_def::R].current_leg_speed );

    const float accel_alpha =
        _ctx.data._dt / (0.01f + _ctx.data._dt);
    for (auto &leg : _ctx.data.leg)
    {
        const float raw_leg_accel =
            std::clamp((leg.current_leg_speed - leg.previous_leg_speed) /
                           _ctx.data._dt,
                       -100.0f, 100.0f);
        leg.current_leg_accel +=
            accel_alpha * (raw_leg_accel - leg.current_leg_accel);
        leg.previous_leg_speed = leg.current_leg_speed;
        const float beta_dot = leg.current_leg_radps - _ctx.data.ins.gyro[1];
        const float raw_beta_accel =
            std::clamp((beta_dot - leg.previous_leg_radps) /
                           _ctx.data._dt,
                       -100.0f, 100.0f);
        leg.current_leg_rad_accel +=
            accel_alpha * (raw_beta_accel - leg.current_leg_rad_accel);
        leg.previous_leg_radps = beta_dot;
    }

    _calc_ddot_x_imu();
    _update_accel_heading_frame();
    // Refresh with the current body pitch before support-force estimation.
    // The takeoff detector must account for force carried by the gas spring,
    // otherwise compensation lowers motor torque and looks like loss of contact.
    _calc_support_force();

}


void wl_chassis_t::_calc_ddot_x_imu()
{
    auto &odom = _ctx.data.odom;
    const auto &ins = _ctx.data.ins;
    const auto &state = _ctx.data.measured_state;
    const auto &left = _ctx.data.leg[leg_def::L];
    const auto &right = _ctx.data.leg[leg_def::R];
    const float wx = ins.gyro[2];
    const float wy = ins.gyro[1];
    const float wz = ins.gyro[0];
    const float st = arm_sin_f32(state.theta);
    const float ct = arm_cos_f32(state.theta);
    const float sp = arm_sin_f32(_ctx.data.measured_aux.phi);
    const float cp = arm_cos_f32(_ctx.data.measured_aux.phi);
    const float s1 = arm_sin_f32(state.beta1);
    const float c1 = arm_cos_f32(state.beta1);
    const float s2 = arm_sin_f32(state.beta2);
    const float c2 = arm_cos_f32(state.beta2);

    // beta_i = current_leg_rad_i - pi/2 - theta；倾斜时 dot_theta != wy。
    const float dot_theta = cp * wy - sp * wz;
    const float s_bar = 0.5f * (left.current_leg_length * s1
                               + right.current_leg_length * s2);
    const float dot_s_bar = 0.5f * (
        left.current_leg_speed * s1
        + left.current_leg_length * c1 * (left.current_leg_radps - dot_theta)
        + right.current_leg_speed * s2
        + right.current_leg_length * c2 * (right.current_leg_radps - dot_theta));

    static float previous_gyro[3] = {ins.gyro[0], ins.gyro[1], ins.gyro[2]};
    static float previous_dot_s_bar = dot_s_bar;
    static float angular_accel[3] = {};
    static float ddot_s_bar = 0.0f;

    for (uint8_t i = 0; i < 3; ++i)
    {
        angular_accel[i] = IMU_ACCEL_DIFF_A * angular_accel[i]
            + IMU_ACCEL_DIFF_B * (ins.gyro[i] - previous_gyro[i]);
        previous_gyro[i] = ins.gyro[i];
    }
    ddot_s_bar = IMU_ACCEL_DIFF_A * ddot_s_bar
        + IMU_ACCEL_DIFF_B * (dot_s_bar - previous_dot_s_bar);
    previous_dot_s_bar = dot_s_bar;

    // I -> H：减去 alpha x r 和 omega x (omega x r)，重力已由 INS 去除。
    const float a_hx = ins.accel[0] - angular_accel[1] * IMU_DZ
        - (wx * wz * IMU_DZ - (wy * wy + wz * wz) * IMU_DX);
    const float a_hy = ins.accel[1]
        - (angular_accel[0] * IMU_DX - angular_accel[2] * IMU_DZ)
        - (wx * wy * IMU_DX + wy * wz * IMU_DZ);
    const float a_hz = ins.accel[2] + angular_accel[1] * IMU_DX
        - (wx * wz * IMU_DX - (wx * wx + wy * wy) * IMU_DZ);

    // H -> 航向系 -> C：这里使用实际航向角速度，不能使用云台相对转速。
    const float dot_psi = (sp * wy + cp * wz) / ct;
    odom.ddot_x_imu = ct * a_hx + st * (sp * a_hy + cp * a_hz)
        + ddot_s_bar - dot_psi * dot_psi * s_bar;
}

void wl_chassis_t::_reset_forward_kf()
{
    const auto &odom = _ctx.data.odom;
    const bool wheel_valid = _ctx.motor.wheel[leg_def::L]->is_online() &&
                             _ctx.motor.wheel[leg_def::R]->is_online();
    _apply_forward_estimate(_forward_kf.reset(
        0.0f, odom.real_dot_x[0], odom.ddot_x_imu, wheel_valid));
}

void wl_chassis_t::_update_forward_kf()
{
    const auto &odom = _ctx.data.odom;
    const bool wheel_valid = _ctx.motor.wheel[leg_def::L]->is_online() &&
                             _ctx.motor.wheel[leg_def::R]->is_online();
    _apply_forward_estimate(_forward_kf.update(
        odom.ddot_x_imu, odom.real_dot_x[0], wheel_valid));
}

void wl_chassis_t::_apply_forward_estimate(const ForwardStateEstimate &estimate)
{
    _ctx.data.odom.forward = estimate;
    _ctx.data.measured_state.x = estimate.x;
    _ctx.data.measured_state.dot_x = estimate.v;
}

void wl_chassis_t::_fsm_execute()
{
    //判断是否主动清除紧急下力
    static pyro::chassis_function_state_t last_cmd_state = pyro::chassis_function_state_t::NONE;
    if (_current_cmd.cmd_function_state == pyro::chassis_function_state_t::RESTART &&
        last_cmd_state != _current_cmd.cmd_function_state)
    {
        _ctx.data.flag.leg_is_should_restart = false;
    }
    last_cmd_state = _current_cmd.cmd_function_state;

    //如果触发了自救键位，则紧急下力自动清除，进入平衡模式
    if(_current_cmd.cmd_function_state == pyro::chassis_function_state_t::RESCUE)
    {
        _ctx.data.flag.leg_is_should_restart = false;
    }

    if (_current_cmd.mode == cmd_base_t::mode_t::ACTIVE && (!_ctx.data.flag.leg_is_should_restart))
    {
        _main_fsm.change_state(&_state_active);
    }
    else
    {
        _main_fsm.change_state(&_state_passive);
    }

    _main_fsm.execute(this);
}

void wl_chassis_t::_vmc_trans_j2v()
{
    for (auto &leg : _ctx.data.leg)
    {
        const float raw_2_theta = leg.current_joint_rad[joint_def::KNEE] -
                                  leg.current_joint_rad[joint_def::HIP];
        const float theta     = loop_fp32_constrain(raw_2_theta, 0, 2*PI ) / 2;
        const float dot_theta = (leg.current_joint_radps[joint_def::KNEE] -
                                 leg.current_joint_radps[joint_def::HIP]) /
                                2;
        const float sin_theta  = arm_sin_f32(theta);
        const float cos_theta  = arm_cos_f32(theta);
        float OH               = OJ5 * cos_theta;
        float HJ5              = OJ5 * sin_theta;
        float HJ4              = sqrt(J4J5 * J4J5 - HJ5 * HJ5);
        float OJ4              = OH + HJ4;
        leg.J_L                = -OJ8 * sin_theta * (OJ4 / HJ4);
        leg.current_leg_length = OJ4 * OJ8 / OJ5;
        leg.L_wp               = evaluate_polynomial_ascending(
            leg.current_leg_length, L_WP_POLY_COEF, L_WP_POLY_DEGREE);
        leg.gas_spring_force   = _calc_gas_spring_force(
            leg.current_leg_length);
        leg.current_leg_speed = dot_theta * leg.J_L;

        const float raw_beta  = leg.current_joint_rad[joint_def::HIP] + theta;
        const float beta      = loop_fp32_constrain(raw_beta, -PI, PI);
        leg.current_leg_rad   = beta;
        leg.current_leg_radps = (leg.current_joint_radps[joint_def::KNEE] +
                                 leg.current_joint_radps[joint_def::HIP]) /
                                2;
        leg.current_F_L = (leg.current_joint_torque[joint_def::KNEE] -
                           leg.current_joint_torque[joint_def::HIP]) /
                          leg.J_L +  leg.gas_spring_force;
        leg.current_T_p = leg.current_joint_torque[joint_def::KNEE] +
                          leg.current_joint_torque[joint_def::HIP];
    }
}

void wl_chassis_t::_manual_control()
{
    for (uint8_t i = 0; i < 2; ++i)
    {
        auto &leg = _ctx.data.leg[i];
        const float total_force = _ctx.pid.leg_length[i]->calculate(
            leg.target_leg_length, leg.current_leg_length, leg.current_leg_speed);
        leg.out_F_L = _motor_leg_force_from_total(leg, total_force);
    }


    float ll_target_radps;
    ll_target_radps = _ctx.pid.leg_control_rad[leg_def::L]->calculate(
        0.0f, _ctx.data.leg[leg_def::L].error_leg_rad);
    _ctx.data.leg[leg_def::L].out_T_p = _ctx.pid.leg_control_radps[leg_def::L]->calculate
        (ll_target_radps, _ctx.data.leg[leg_def::L].current_leg_radps);

    float rl_target_radps;
    rl_target_radps = _ctx.pid.leg_control_rad[leg_def::R]->calculate(
    0.0f, _ctx.data.leg[leg_def::R].error_leg_rad);
    _ctx.data.leg[leg_def::R].out_T_p = _ctx.pid.leg_control_radps[leg_def::R]->calculate
    (rl_target_radps, _ctx.data.leg[leg_def::R].current_leg_radps);

    // _ctx.data.leg[leg_def::L].out_T_p = _ctx.pid.leg_rad[leg_def::L]->calculate(
    //     0.0f, _ctx.data.leg[leg_def::L].error_leg_rad,
    //     _ctx.data.leg[leg_def::L].current_leg_radps);

    // _ctx.data.leg[leg_def::R].out_T_p = _ctx.pid.leg_rad[leg_def::R]->calculate(
    //     0.0f, _ctx.data.leg[leg_def::R].error_leg_rad,
    //     _ctx.data.leg[leg_def::R].current_leg_radps);
}

float wl_chassis_t::_calc_leg_length_wall_force(const leg_ctx_t &leg) const
{
    // Damping opposes motion toward the stop over the wider region; the
    // narrower stiffness region supplies a static restoring force.
    if (leg.current_leg_length > LEG_LENGTH_WALL_DAMPING_MAX)
    {
        const float penetration = std::max(
            leg.current_leg_length - LEG_LENGTH_WALL_MAX, 0.0f);
        const float damping_force =
            LEG_LENGTH_WALL_D * std::max(leg.current_leg_speed, 0.0f);
        return -LEG_LENGTH_WALL_K * penetration - damping_force;
    }

    if (leg.current_leg_length < LEG_LENGTH_WALL_DAMPING_MIN)
    {
        const float penetration = std::max(
            LEG_LENGTH_WALL_MIN - leg.current_leg_length, 0.0f);
        const float damping_force =
            LEG_LENGTH_WALL_D * std::max(-leg.current_leg_speed, 0.0f);
        return LEG_LENGTH_WALL_K * penetration + damping_force;
    }

    return 0.0f;
}

float wl_chassis_t::_motor_torque_to_leg_force(const leg_ctx_t &leg, const float signed_torque) const
{
    const float jacobian = std::fabs(leg.J_L);
    if (!std::isfinite(jacobian) || jacobian < 1.0e-4f)
    {
        return 0.0f;
    }
    // Positive signed_torque always means extension (+F_L), independent of
    // the kinematic branch. Each joint receives +/- its magnitude in VMC.
    return 2.0f * signed_torque / jacobian;
}

float wl_chassis_t::_calc_gas_spring_force(const float leg_length) const
{
    return wheel_leg_ground10::evaluateGasSpringForce(leg_length);
}

float wl_chassis_t::_motor_leg_force_from_total(const leg_ctx_t &leg, const float total_force)
{
    return total_force - GAS_SPRING_COMPENSATION_SCALE * leg.gas_spring_force;
}

void wl_chassis_t::_reset_air_contacts()
{
    auto &air = _ctx.data.airborne;
    air.flight_time = 0.0f;
    for (uint8_t i = 0; i < 2; ++i)
    {
        air.contact[i] = false;
        air.contact_confirm_time[i] = 0.0f;
        air.contact_release_time[i] = 0.0f;
        air.extension_speed_peak[i] = 0.0f;
    }
}

void wl_chassis_t::_gain_calculate()
{
    const float l1 = _ctx.data.leg[leg_def::L].current_leg_length;
    const float l2 = _ctx.data.leg[leg_def::R].current_leg_length;
    wheel_leg_ground10::evaluateGain(l1, l2, _ctx.data.K);
    float trim_state[STATE_DIM];
    wheel_leg_ground10::evaluateFeedforward(
        l1, l2, trim_state, _ctx.data.U0, GAS_SPRING_COMPENSATION_SCALE);
    // 保留前向位置、偏航和俯仰参考值，按腿长调度平衡摆角。
    _ctx.data.target_state.beta1 = trim_state[lqr_state_def::BETA_1];
    _ctx.data.target_state.beta2 = trim_state[lqr_state_def::BETA_2];
}

float wl_chassis_t::_calc_roll_force_trim(const float left_base_force,
                                         const float right_base_force,
                                         const float mean_leg_speed_ref)
{
    auto &pid = _ctx.data.roll_pid;
    const auto &target = _ctx.data.target_aux;
    const auto &measured = _ctx.data.measured_aux;
    const float dt = _ctx.data._dt;
    if (!std::isfinite(target.phi) || !std::isfinite(target.dot_phi) ||
        !std::isfinite(measured.phi) || !std::isfinite(measured.dot_phi) ||
        !std::isfinite(dt) || dt <= 0.0f || dt > 0.1f)
    {
        pid = {};
        return 0.0f;
    }

    pid.error = loop_fp32_constrain(target.phi - measured.phi, -PI, PI);
    pid.p_force = NORMAL_ROLL_KP * pid.error;
    // 微分项直接使用陀螺仪测得的横滚角速度。
    pid.d_force = NORMAL_ROLL_KD * (target.dot_phi - measured.dot_phi);
    // 前馈为负时帮助右腿伸长，为正时帮助右腿缩短。
    const float feedforward_target = std::clamp(
        -NORMAL_ROLL_HEIGHT_RATE_FF_GAIN * mean_leg_speed_ref,
        -NORMAL_ROLL_HEIGHT_RATE_FF_LIMIT, NORMAL_ROLL_HEIGHT_RATE_FF_LIMIT);
    const float ff_alpha = dt / (NORMAL_ROLL_HEIGHT_RATE_FF_TAU + dt);
    pid.height_rate_feedforward += ff_alpha * (feedforward_target - pid.height_rate_feedforward);
    const float nonintegral_force = pid.p_force + pid.d_force + pid.height_rate_feedforward;

    // 根据两腿的虚拟力余量限幅，保持修正力等大反向。
    const float lower = std::max(-NORMAL_ROLL_MAX_OUT,
        std::max(-MAX_F_L - left_base_force, right_base_force - MAX_F_L));
    const float upper = std::min(NORMAL_ROLL_MAX_OUT,
        std::min(MAX_F_L - left_base_force, right_base_force + MAX_F_L));
    const float integral_step = std::fabs(pid.error) > NORMAL_ROLL_INTEGRAL_DEADBAND
        ? NORMAL_ROLL_INTEGRAL_KI * pid.error * dt : 0.0f;
    const float candidate_i = std::clamp(pid.integral_force + integral_step,
        -NORMAL_ROLL_INTEGRAL_LIMIT, NORMAL_ROLL_INTEGRAL_LIMIT);
    const float candidate_out = nonintegral_force + candidate_i;
    // 积分会加重输出饱和时，暂停累积。
    if (!((candidate_out > upper && integral_step > 0.0f) ||
          (candidate_out < lower && integral_step < 0.0f)))
    {
        pid.integral_force = candidate_i;
    }
    pid.force_trim = std::clamp(nonintegral_force + pid.integral_force, lower, upper);
    return pid.force_trim;
}

float wl_chassis_t::_calc_mean_leg_length_force_trim(const float left_base_force,
                                                    const float right_base_force,
                                                    const float mean_leg_speed_ref)
{
    auto &pid = _ctx.data.mean_leg_length_pid;
    const auto &target = _ctx.data.target_aux;
    const auto &measured = _ctx.data.measured_aux;
    const float dt = _ctx.data._dt;
    if (!std::isfinite(target.L) || !std::isfinite(target.dot_L) ||
        !std::isfinite(measured.L) || !std::isfinite(measured.dot_L) ||
        !std::isfinite(dt) || dt <= 0.0f || dt > 0.1f)
    {
        pid = {};
        return 0.0f;
    }

    const float target_length = std::clamp(target.L, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    pid.error = target_length - measured.L;
    pid.p_force = MEAN_LEG_LENGTH_PID_KP * pid.error;
    pid.d_force = MEAN_LEG_LENGTH_PID_KD * (mean_leg_speed_ref - measured.dot_L);

    // 按各腿叠加 U0 后的剩余力限幅，保证两腿能施加相同修正力。
    const float lower = std::max(-MEAN_LEG_LENGTH_PID_MAX_OUT,
        -MAX_F_L - std::min(left_base_force, right_base_force));
    const float upper = std::min(MEAN_LEG_LENGTH_PID_MAX_OUT,
        MAX_F_L - std::max(left_base_force, right_base_force));
    const float integral_step = MEAN_LEG_LENGTH_PID_KI * pid.error * dt;
    const float candidate_i = std::clamp(pid.integral_force + integral_step,
        -MEAN_LEG_LENGTH_PID_INTEGRAL_LIMIT, MEAN_LEG_LENGTH_PID_INTEGRAL_LIMIT);
    const float candidate_out = pid.p_force + candidate_i + pid.d_force;
    if (!((candidate_out > upper && integral_step > 0.0f) ||
          (candidate_out < lower && integral_step < 0.0f)))
    {
        pid.integral_force = candidate_i;
    }
    pid.force_trim = std::clamp(pid.p_force + pid.integral_force + pid.d_force,
                                lower, upper);
    return pid.force_trim;
}

void wl_chassis_t::_balance_control(const bool apply_roll_control,
                                   const bool apply_mean_leg_length_control)
{
    float error[STATE_DIM];
    for (uint8_t state = 0; state < STATE_DIM; ++state)
    {
        error[state] = _ctx.data.target_state.data[state] -
                       _ctx.data.measured_state.data[state];
    }
    error[lqr_state_def::PSI] =
        loop_fp32_constrain(error[lqr_state_def::PSI], -PI, PI);

    // 计算十状态 LQR 的四路力矩输出。

    for (uint8_t input = 0; input < INPUT_DIM; ++input)
    {
        _ctx.data.control.data[input] = _ctx.data.U0[input];
        for (uint8_t state = 0; state < STATE_DIM; ++state)
        {
            _ctx.data.control.data[input] +=
                _ctx.data.K[input][state] * error[state];
        }
    }

    _ctx.data.control.T_w1 =
        std::clamp(_ctx.data.control.T_w1, -MAX_T_W, MAX_T_W);
    _ctx.data.control.T_w2 =
        std::clamp(_ctx.data.control.T_w2, -MAX_T_W, MAX_T_W);
    _ctx.data.control.T_p1 =
        std::clamp(_ctx.data.control.T_p1, -MAX_T_P, MAX_T_P);
    _ctx.data.control.T_p2 =
        std::clamp(_ctx.data.control.T_p2, -MAX_T_P, MAX_T_P);

    _ctx.data.leg[leg_def::L].out_T_p   = _ctx.data.control.T_p1;
    _ctx.data.leg[leg_def::R].out_T_p   = _ctx.data.control.T_p2;
    float base_force[2];
    for (uint8_t i = 0; i < 2; ++i)
    {
        // 导出的 U0 已从支撑力中扣除气弹簧力。
        base_force[i] = std::clamp(
            _ctx.data.U0[lqr_feedforward_def::F_L1 + i],
            -MAX_F_L, MAX_F_L);
    }
    float mean_leg_speed_ref = 0.0f;
    if (apply_mean_leg_length_control)
    {
        const auto &target = _ctx.data.target_aux;
        const auto &measured = _ctx.data.measured_aux;
        if (std::isfinite(target.L) && std::isfinite(target.dot_L) &&
            std::isfinite(measured.L) && std::isfinite(measured.dot_L) &&
            !(target.L <= MIN_LEG_LENGTH && target.dot_L < 0.0f) &&
            !(target.L >= MAX_LEG_LENGTH && target.dot_L > 0.0f))
        {
            mean_leg_speed_ref = target.dot_L;
        }
        const float height_trim = _calc_mean_leg_length_force_trim(
            base_force[leg_def::L], base_force[leg_def::R], mean_leg_speed_ref);
        base_force[leg_def::L] += height_trim;
        base_force[leg_def::R] += height_trim;
    }
    else
    {
        _ctx.data.mean_leg_length_pid = {};
    }
    float roll_trim = 0.0f;
    if (apply_roll_control)
    {
        roll_trim = _calc_roll_force_trim(
            base_force[leg_def::L], base_force[leg_def::R], mean_leg_speed_ref);
    }
    else
    {
        _ctx.data.roll_pid = {};
    }
    _ctx.data.leg[leg_def::L].out_F_L = base_force[leg_def::L] + roll_trim;
    _ctx.data.leg[leg_def::R].out_F_L = base_force[leg_def::R] - roll_trim;

    _ctx.data.wheel[leg_def::L].out_T_w = _ctx.data.control.T_w1;
    _ctx.data.wheel[leg_def::R].out_T_w = _ctx.data.control.T_w2;
    for (auto &wheel : _ctx.data.wheel)
    {
        wheel.out_current =
            std::clamp(wheel.out_T_w * (1 / K_t), -MAX_CURRENT, MAX_CURRENT);
    }
}

void wl_chassis_t::_vmc_trans_v2j(const bool apply_leg_length_wall)
{
    constexpr float MAX_MOTOR_TORQUE = 40.0f;

    // Inputs are already motor-side forces; each phase applies its own
    // feedforward exactly once. Airborne extension can explicitly bypass the
    // leg-length wall so the fixed extension effort is not opposed at the stop.
    for (auto &leg : _ctx.data.leg)
    {
        leg.virtual_wall_force = apply_leg_length_wall
            ? _calc_leg_length_wall_force(leg) : 0.0f;
        const float t_p_motor = leg.out_T_p;
        leg.motor_f_l = std::clamp(leg.out_F_L + leg.virtual_wall_force,
                                   -MAX_F_L, MAX_F_L);
        leg.motor_t_p = t_p_motor;

        float tau_sum                        = leg.motor_t_p;
        float tau_diff                       = leg.motor_f_l * leg.J_L;
        leg.out_joint_torque[joint_def::HIP] = std::clamp(
            (tau_sum - tau_diff) / 2, -MAX_MOTOR_TORQUE, MAX_MOTOR_TORQUE);
        leg.out_joint_torque[joint_def::KNEE] = std::clamp(
            (tau_sum + tau_diff) / 2, -MAX_MOTOR_TORQUE, MAX_MOTOR_TORQUE);
    }
}

void wl_chassis_t::_send_joint_torque() const
{
    _ctx.motor.joint[leg_def::L][joint_def::HIP]->send_torque(
        _ctx.data.leg[leg_def::L].direction *
        _ctx.data.leg[leg_def::L].out_joint_torque[joint_def::HIP]);
    _ctx.motor.joint[leg_def::L][joint_def::KNEE]->send_torque(
        _ctx.data.leg[leg_def::L].direction *
        _ctx.data.leg[leg_def::L].out_joint_torque[joint_def::KNEE]);
    _ctx.motor.joint[leg_def::R][joint_def::HIP]->send_torque(
        _ctx.data.leg[leg_def::R].direction *
        _ctx.data.leg[leg_def::R].out_joint_torque[joint_def::HIP]);
    _ctx.motor.joint[leg_def::R][joint_def::KNEE]->send_torque(
        _ctx.data.leg[leg_def::R].direction *
        _ctx.data.leg[leg_def::R].out_joint_torque[joint_def::KNEE]);

    // _ctx.motor.joint[leg_def::L][joint_def::HIP]->send_torque(0);
    // _ctx.motor.joint[leg_def::L][joint_def::KNEE]->send_torque(0);
    // _ctx.motor.joint[leg_def::R][joint_def::HIP]->send_torque(0);
    // _ctx.motor.joint[leg_def::R][joint_def::KNEE]->send_torque(0);
}

void wl_chassis_t::_send_wheel_torque() const
{
    _ctx.motor.wheel[leg_def::L]->send_torque(
        _ctx.data.wheel[leg_def::L].direction *
        _ctx.data.wheel[leg_def::L].out_current);
    _ctx.motor.wheel[leg_def::R]->send_torque(
        _ctx.data.wheel[leg_def::R].direction *
        _ctx.data.wheel[leg_def::R].out_current);

    // _ctx.motor.wheel[leg_def::L]->send_torque(0);
    // _ctx.motor.wheel[leg_def::R]->send_torque(0);
}

} // namespace pyro
