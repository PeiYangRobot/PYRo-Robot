// Firmware feedback, KF, LQR and FSM bodies with simulated hardware.
#include "pyro_wl_forward_kf.h"
#include "wl_config.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

static void check(bool condition, const char *message)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

float debug = 0.0f;
float debug2 = 0.0f;

namespace pyro
{
float arm_sin_f32(float x) { return std::sin(x); }
float arm_cos_f32(float x) { return std::cos(x); }
struct motor_base_t
{
    float position = 0.0f, rotate = 0.0f, torque = 0.0f;
    bool online = true;
    void update_feedback() {}
    float get_current_position() const { return position; }
    float get_current_rotate() const { return rotate; }
    float get_current_torque() const { return torque; }
    bool is_online() const { return online; }
    void enable() {}
};
struct pd_ctrl_t {};
struct pid_t {};
struct cmd_base_t {};
#include "chassis_types.inc"

struct dwt_drv_t
{
    static float get_delta_t(uint32_t *) { return FORWARD_KF_TS; }
};
struct ins_drv_t
{
    float forward_accel = 0.0f;
    static ins_drv_t *get_instance() { static ins_drv_t instance; return &instance; }
    void get_rads_n(float *yaw, float *pitch, float *roll) { *yaw = *pitch = *roll = 0.0f; }
    void get_gyro_b(float *z, float *y, float *x) { *z = *y = *x = 0.0f; }
    void get_accel_without_g_b(float *x, float *y, float *z) { *x = forward_accel; *y = *z = 0.0f; }
};
struct stp23l_drv
{
    static stp23l_drv down_get_instance() { return {}; }
    static stp23l_drv front_get_instance() { return {}; }
    uint32_t get_distance() const { return 0; }
};
namespace bsp_can { enum { can3 }; }
struct board_drv_t
{
    enum class role_t { CHASSIS };
    struct data_t { int imu_yaw_radps_100 = 0; };
    static board_drv_t &get_instance(role_t, int) { static board_drv_t instance; return instance; }
    data_t get_g2c_rx_data() const { return {}; }
};

struct wl_chassis_t
{
    struct state_stub_t { template<class T> void request_switch(T *) {} };
    struct fsm_active_t
    {
        struct state_normal_t
        {
            struct state_balance_t : state_stub_t
            {
                void enter(wl_chassis_t *);
                void execute(wl_chassis_t *);
                void exit(wl_chassis_t *);
            } _state_balance;
            struct state_spin_t : state_stub_t
            {
                void enter(wl_chassis_t *);
                void execute(wl_chassis_t *);
                void exit(wl_chassis_t *);
            } _state_spin;
            state_stub_t _state_step, _state_air, _state_getover;
        } _state_normal;
    } _state_active;
    wl_chassis_ctx_t _ctx{};
    wl_chassis_cmd_t _current_cmd{};
    wl_forward_kf_t _forward_kf;
    motor_base_t motors[7];
    float _getover_rearm_elapsed = GETOVER_REARM_DELAY;
    int kf_updates = 0;
    int lqr_updates = 0;

    wl_chassis_t()
    {
        _forward_kf.init();
        for (int leg = 0; leg < 2; ++leg)
        {
            _ctx.motor.wheel[leg] = &motors[leg];
            _ctx.motor.joint[leg][0] = &motors[2 + 2 * leg];
            _ctx.motor.joint[leg][1] = &motors[3 + 2 * leg];
            _ctx.data.wheel[leg].direction = leg == 0 ? LEFT_WHEEL_DIRECTION : RIGHT_WHEEL_DIRECTION;
            auto &data = _ctx.data.leg[leg];
            data.current_leg_length = 0.28f;
            data.direction = leg == 0 ? LEFT_LEG_DIRECTION : RIGHT_LEG_DIRECTION;
            data.current_leg_rad = PI / 2.0f;
        }
        _ctx.motor.yaw = &motors[6];
        motors[6].position = YAW_OFFSET;
        _ctx.data._dt = FORWARD_KF_TS;
    }

    void _update_feedback();
    void _gain_calculate();
    void _calc_ddot_x_imu();
    void _reset_forward_kf();
    void _apply_forward_estimate(const ForwardStateEstimate &estimate);
    void _update_forward_kf_impl();
    void _balance_control_impl(bool apply_roll_control, bool apply_mean_leg_length_control);
    float _calc_roll_force_trim(float left_force, float right_force, float speed);
    float _calc_mean_leg_length_force_trim(float left_force, float right_force, float speed);
    void _vmc_trans_j2v() {}
    void _vmc_trans_v2j(bool = true) {}
    void _send_joint_torque() {}
    void _send_wheel_torque() {}
    void _update_accel_heading_frame() {}
    void _calc_support_force() {}
    bool _detect_takeoff() { return false; }
    void _execute_landing_recovery() {}
    float _landing_velocity_reference(float commanded_velocity) { return commanded_velocity; }

    void _update_forward_kf()
    {
        check(kf_updates == lqr_updates, "one KF update per control tick");
        const auto before = _ctx.data.measured_state;
        _update_forward_kf_impl();
        for (int state = 2; state < STATE_DIM; ++state)
            check(_ctx.data.measured_state.data[state] == before.data[state],
                  "KF preserves all non-forward feedback entries");
        ++kf_updates;
    }

    void _balance_control(bool apply_roll_control, bool apply_mean_leg_length_control)
    {
        check(kf_updates == lqr_updates + 1, "LQR runs after current KF");
        const auto &kf = _forward_kf.get_filter();
        const auto &data = _ctx.data;
        check(data.measured_state.x == kf.get_state()[0] &&
              data.measured_state.dot_x == kf.get_state()[1], "LQR reads current KF x/v");
        check(data.measured_state.x == data.odom.forward.x &&
              data.measured_state.dot_x == data.odom.forward.v, "fused odometry and LQR feedback agree");
        check(apply_roll_control && apply_mean_leg_length_control, "both leg PID loops remain enabled");
        _balance_control_impl(apply_roll_control, apply_mean_leg_length_control);
        for (int input = 0; input < INPUT_DIM; ++input)
        {
            float expected = data.U0[input];
            for (int state = 0; state < STATE_DIM; ++state)
            {
                float error = data.target_state.data[state] - data.measured_state.data[state];
                if (state == lqr_state_def::PSI)
                    error = loop_fp32_constrain(error, -PI, PI);
                expected += data.K[input][state] * error;
            }
            const float limit = input < 2 ? MAX_T_W : MAX_T_P;
            check(std::abs(data.control.data[input] - std::clamp(expected, -limit, limit)) < 1.0e-4f,
                  "actual LQR command uses fused feedback");
        }
        ++lqr_updates;
    }

    void feedback_frame(float wheel_v, float imu_accel = 0.0f)
    {
        for (int leg = 0; leg < 2; ++leg)
            motors[leg].rotate = wheel_v /
                (WHEEL_RADIUS * _ctx.data.wheel[leg].direction * rec_reduction_ratio);
        ins_drv_t::get_instance()->forward_accel = imu_accel;
        _update_feedback();
        const auto &data = _ctx.data;
        check(data.measured_state.x == data.odom.real_x &&
              data.measured_state.dot_x == 0.5f * (data.odom.real_dot_x[0] + data.odom.real_dot_x[1]),
              "raw wheel odometry remains available to other modes");
        check(std::abs(data.odom.ddot_x_imu - imu_accel) < 1.0e-5f,
              "straight-ahead IMU acceleration reaches the KF adapter");
    }
};
} // namespace pyro

#include "chassis_functions.inc"

template <typename GroundState>
void check_ground_state(pyro::wl_chassis_t &chassis, GroundState &state)
{
    auto &data = chassis._ctx.data;
    chassis.feedback_frame(0.0f);
    state.enter(&chassis);
    check(data.measured_state.x == 0.0f && data.odom.forward.x == 0.0f &&
          data.odom.forward.slip_score == 0.0f, "entering ground control resets KF and its origin");

    chassis.feedback_frame(0.0f);
    state.execute(&chassis);

    chassis.feedback_frame(3.0f);
    const float raw_velocity = data.measured_state.dot_x;
    state.execute(&chassis);
    check(data.odom.forward.slip_score >= pyro::FORWARD_KF_BIAS_FREEZE_SCORE,
          "a wheel-speed spike raises the slip score in the same tick");
    check(std::abs(data.measured_state.dot_x) < raw_velocity,
          "slip attenuates the wheel-speed spike before LQR");

    chassis.motors[1].online = false;
    chassis.feedback_frame(3.0f, 0.5f);
    state.execute(&chassis);
    const auto &kf = chassis._forward_kf.get_filter();
    check(kf.get_gain()(1, 1) == 0.0f && kf.get_gain()(3, 0) == 0.0f,
          "one offline wheel masks wheel speed and freezes bias learning");
    chassis.motors[1].online = true;

    state.exit(&chassis);
}

int main()
{
    pyro::wl_chassis_t chassis;
    auto &normal = chassis._state_active._state_normal;
    check_ground_state(chassis, normal._state_balance);
    check_ground_state(chassis, normal._state_spin);
    check(chassis._ctx.data.target_state.x == chassis._ctx.data.measured_state.x,
          "spin target follows the current fused position");
    check(chassis.kf_updates == 6 && chassis.lqr_updates == 6,
          "balance and spin run one KF update before each control tick");
    std::puts("PASS: balance and spin use fused forward feedback and keep both leg PID loops.");
}
