#ifndef __PYRO_WL_CHASSIS_H__
#define __PYRO_WL_CHASSIS_H__

#include "pyro_algo_pd.h"
#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "pyro_wl_forward_kf.h"
#include "wl_config.h"
#include "dsp/window_functions.h"
#include <cstdint>

namespace pyro
{
struct wl_chassis_deps_t
{
    struct motor_deps_t
    {
        motor_base_t *joint[2][2];
        motor_base_t *wheel[2];
        motor_base_t *yaw;
    };

    struct pid_deps_t
    {
        pd_ctrl_t *leg_length[2];
        pyro::pid_t *leg_control_rad[2];   //角度环
        pyro::pid_t *leg_control_radps[2]; //速度环
    };
    motor_deps_t motor;
    pid_deps_t pid;
};

enum class chassis_active_state_t : uint8_t
{
    NORMAL,
    MANUAL,

};

enum class chassis_function_state_t : uint8_t
{
    NONE,
    RESTART,
    STEP,
    JUMP,
    SPIN_TOGGLE,
    GETOVER,
    RESCUE,


};

enum class jump_phase_t : uint8_t
{
    PRE_COMPRESS,
    JUMP,
    RECOVERY,
    AIR,
    RETURN,
};

enum class jump_trigger_source_t : uint8_t
{
    NONE,
    SINGLE_COMMAND,
    FRONT_DISTANCE,
};

struct wl_chassis_cmd_t final : public cmd_base_t
{
    float delta_leg_length[2];
    float delta_leg_rad[2];
    float v;
    float wz;
    //bool balance_flag = false;
    float dot_L;

    int reset_chassis_times;
    int step_times;

    chassis_active_state_t cmd_continus_state;
    chassis_function_state_t cmd_function_state;
    bool getover_detection_enabled{false};
    bool jump_duo_enabled{false};

};

struct leg_ctx_t
{
    float target_leg_length;
    float target_leg_speed;
    float target_leg_rad;
    float target_leg_radps;
    float current_leg_length; // current_leg_length = f((θ1 - θ2) / 2)
    float current_leg_speed;
    float previous_leg_speed;
    float current_leg_accel;
    float current_leg_rad; // current_leg_rad = (θ1 + θ2) / 2
    float current_leg_radps;
    float previous_leg_radps;
    float current_leg_rad_accel;
    float error_leg_rad;

    float J_L;
    float L_wp;
    float out_F_L; // F_L * J_L = tau_2 - tau_1
    float virtual_wall_force;
    float gas_spring_force; // 沿腿长增大方向的被动广义力
    float out_T_p; // T_p = tau_2 + tau_1
    float motor_f_l;
    float motor_t_p;
    float current_F_L;
    float current_T_p;

    float direction; // Motor-positive sign in the defined leg coordinate system
    float current_joint_rad[2]; // θ1，θ2
    float current_joint_radps[2];
    float current_joint_torque[2];
    float out_joint_torque[2];
};

struct wheel_ctx_t
{
    float direction;
    float current_radps;
    float current_T_w;
    float out_T_w;
    float out_current; // Current: 电流 (A)
};


struct yaw_ctx_t
{
    float pos;
    float rot;
};


struct state_vec_t
{
    union
    {
        struct
        {
            float x, dot_x;
            float psi, dot_psi;
            float theta, dot_theta;
            float beta1, dot_beta1;
            float beta2, dot_beta2;
        };
        float data[STATE_DIM];
    };
};
struct control_vec_t
{
    union
    {
        struct
        {
            float T_w1,T_w2;
            float T_p1,T_p2;
        };
        float data[INPUT_DIM];
    };
};
static_assert(sizeof(state_vec_t) == 10 * sizeof(float), "Ground LQR requires ten states");
static_assert(sizeof(control_vec_t) == 4 * sizeof(float), "Ground LQR requires four torques");

// 状态机、车高和横滚控制所用的辅助参考量与反馈量。
struct chassis_aux_state_t
{
    float phi, dot_phi;
    float L, dot_L;
};
struct roll_pid_data_t
{
    float error;
    float p_force;
    float integral_force;
    float d_force;
    float height_rate_feedforward; // 滤波后的升降差动力前馈，单位 N/腿。
    float force_trim; // 左腿加此修正力，右腿减此修正力。
};
struct mean_leg_length_pid_data_t
{
    float error; // 目标平均腿长减去实测平均腿长，单位 m。
    float p_force;
    float integral_force;
    float d_force;
    float force_trim; // 左右腿均加此修正力，单位 N/腿。
};
struct odom_t
{
    float real_x;
    float real_dot_x[2]; // 0 : v_x(t) , 1 :v_x(t-1)
    ForwardStateEstimate forward;
    float ddot_x_imu;
};

struct ins_data_t
{
    float euler_rad[3];
    float gyro[3];
    float accel[3];
};



struct airborne_data_t
{
    bool landing_recovery = false;
    bool attitude_recovery = false;
    bool velocity_recovery = false;
    float pitch_reference_acceleration = 0.0f; // 本拍 pitch 参考角速度变化率 (rad/s^2)
    float beta_reference_acceleration[2] = {0.0f, 0.0f}; // 本拍左右腿角参考角速度变化率 (rad/s^2)
    bool contact[2] = {false, false};
    float contact_confirm_time[2] = {0.0f, 0.0f};
    float contact_release_time[2] = {0.0f, 0.0f};
    float extension_speed_peak[2] = {0.0f, 0.0f};
    float flight_time = 0.0f;
    float forward_velocity_ref = 0.0f; // AIR 触地轮速与 BALANCE 落地速度恢复共用参考 (m/s)
    uint16_t takeoff_counter = 0;
    uint16_t landing_counter = 0;
    float L_ref = NORMAL_LENGTH_TARGET;
    float L_air_ref[2] = {NORMAL_LENGTH_TARGET, NORMAL_LENGTH_TARGET};
    float accel_z_y = 0.0f;
    float accel_z_y_lpf = 0.0f;
    float support_force[2] = {0.0f, 0.0f};
    float support_force_sum = 0.0f;
};

struct flag_data_t
{
    bool leg_is_should_restart;  //紧急下力的标志位
    bool chassis_is_align_ready = true; // 机体姿态对齐的标志位,只供给云盘读取
    bool chassis_is_rescuing = false; // 机体姿态自救的标志位,只供给云盘读取
};


struct wl_chassis_data_ctx_t
{
    yaw_ctx_t yaw;
    leg_ctx_t leg[2];
    wheel_ctx_t wheel[2];
    state_vec_t target_state;
    state_vec_t measured_state;
    chassis_aux_state_t target_aux;
    chassis_aux_state_t measured_aux;
    roll_pid_data_t roll_pid;
    mean_leg_length_pid_data_t mean_leg_length_pid;
    control_vec_t control;
    flag_data_t flag;
    float K[INPUT_DIM][STATE_DIM];
    float U0[FEEDFORWARD_DIM]; // 四路 LQR 力矩与两路电机侧轴向腿力。
    odom_t odom;
    ins_data_t ins;
    uint32_t distance_down = 0;// UART10 测距模块距离，单位 mm。
    uint32_t distance_front = 0; // UART7 测距模块距离，单位 mm。
    airborne_data_t airborne;
    jump_phase_t jump_phase = jump_phase_t::PRE_COMPRESS;
    float jump_phase_time = 0.0f;
    float jump_mode_time = 0.0f; // 从进入单跳或连跳状态起累计的时间 (s)
    float jump_recovery_hold_time = 0.0f;
    float jump_heading_ref = 0.0f;
    float jump_forward_velocity_ref = 0.0f; // 进入跳跃时锁存的前进速度 (m/s)
    float jump_precompress_exit_velocity = 0.0f; // 预压缩正常结束时的实测轮式里程计速度 (m/s)
    float jump_vertical_velocity_integral = 0.0f; // 跳跃期间竖直去重力加速度积分得到的速度观测值 (m/s)
    float jump_leg_extension_speed_peak[2] = {0.0f, 0.0f}; // 进入 AIR 后各腿伸展速度的逐拍最大值 (m/s)
    float jump_landing_confirm_time = 0.0f; // 落地判据连续成立的时间 (s)
    uint16_t jump_push_accel_hold_ticks = 0;
    float _dt;
    chassis_function_state_t current_function;//主动量，改变它即可改变状态
    jump_trigger_source_t jump_trigger_source = jump_trigger_source_t::NONE;
    uint32_t jump_trigger_distance_front = 0; // 最近一次跳跃切换请求判定当拍的前向距离 (mm)
};

struct wl_chassis_ctx_t
{
    wl_chassis_deps_t::motor_deps_t motor;
    wl_chassis_deps_t::pid_deps_t pid;
    wl_chassis_data_ctx_t data;
};

struct wl_chassis_param_t
{

    using CmdType    = wl_chassis_cmd_t;
    using ModuleDeps = wl_chassis_deps_t;
    using ModuleCtx  = wl_chassis_ctx_t;
};

class wl_chassis_t final
    : public module_base_t<wl_chassis_t, wl_chassis_param_t>
{
    friend class module_base_t<wl_chassis_t, wl_chassis_param_t>;

  public:
    wl_chassis_t(const wl_chassis_t &)            = delete;
    wl_chassis_t &operator=(const wl_chassis_t &) = delete;
    using data_ctx_t                              = wl_chassis_data_ctx_t;
    using ctx_t                                   = wl_chassis_ctx_t;

  private:
    wl_chassis_t();
    ~wl_chassis_t() override = default;

    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // 私有成员变量

    // 辅助函数
    void _vmc_trans_j2v();
    void _manual_control();
    float _calc_leg_length_wall_force(const leg_ctx_t &leg) const;
    float _motor_torque_to_leg_force(const leg_ctx_t &leg, float signed_torque) const;
    float _calc_gas_spring_force(float leg_length) const;
    static float _motor_leg_force_from_total(const leg_ctx_t &leg, float total_force);
    void _gain_calculate();
    void _calc_ddot_x_imu();
    void _reset_forward_kf();
    void _update_forward_kf();
    void _apply_forward_estimate(const ForwardStateEstimate &estimate);
    float _calc_roll_force_trim(float left_base_force, float right_base_force,
                                float mean_leg_speed_ref);
    float _calc_mean_leg_length_force_trim(float left_base_force, float right_base_force,
                                           float mean_leg_speed_ref);
    void _balance_control(bool apply_roll_control = false,
                          bool apply_mean_leg_length_control = false);
    void _vmc_trans_v2j(bool apply_leg_length_wall = true);
    void _send_joint_torque() const;
    void _send_wheel_torque() const;
    void _update_accel_heading_frame();
    void _calc_support_force();
    bool _detect_takeoff();
    bool _detect_landing();
    void _execute_air_control();
    void _reset_air_contacts();
    void _execute_landing_recovery();
    float _landing_velocity_reference(float commanded_velocity);

    using owner = wl_chassis_t;


    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };
    struct fsm_active_t final : public fsm_t<owner>
    {
        struct state_manual_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_normal_t final : public fsm_t<owner>
        {
            struct state_balance_t final : public state_t<owner>
            {
                void enter(owner *owner) override;
                void execute(owner *owner) override;
                void exit(owner *owner) override;
            };
            struct state_air_t final : public state_t<owner>
            {
                void enter(owner *owner) override;
                void execute(owner *owner) override;
                void exit(owner *owner) override;
            };
            struct state_align_t final : public state_t<owner>
            {
                void enter(owner *owner) override;
                void execute(owner *owner) override;
                void exit(owner *owner) override;
            };
            struct state_step_t final : public state_t<owner>
            {
                void enter(owner *owner) override;
                void execute(owner *owner) override;
                void exit(owner *owner) override;
            };
            struct state_jump_t : public state_t<owner>
            {
                void enter(owner *owner) override;
                void execute(owner *owner) override;
                void exit(owner *owner) override;

              protected:
                void start_jump(owner *owner);
                virtual bool handle_landing(owner *owner);
            };
            struct state_jump_duo_t final : public state_jump_t
            {
                void enter(owner *owner) override;

              private:
                bool handle_landing(owner *owner) override;
                bool _second_jump{false};
            };
            struct state_spin_t final : public state_t<owner>
            {
                void enter(owner *owner) override;
                void execute(owner *owner) override;
                void exit(owner *owner) override;
            };
            struct state_getover_t final : public state_t<owner>
            {
                void enter(owner *owner) override;
                void execute(owner *owner) override;
                void exit(owner *owner) override;
            };
            struct state_rescue_t final : public state_t<owner>
            {
                void enter(owner *owner) override;
                void execute(owner *owner) override;
                void exit(owner *owner) override;
            };

            void on_enter(owner *owner) override;
            void on_execute(owner *owner) override;
            void on_exit(owner *owner) override;

          private:
                state_balance_t _state_balance;
                state_air_t     _state_air;
                state_align_t   _state_align;
                state_step_t    _state_step;
                state_jump_t    _state_jump;
                state_jump_duo_t _state_jump_duo;
                state_spin_t    _state_spin;
                state_getover_t _state_getover;
                state_rescue_t  _state_rescue;
                bool _jump_duo_triggered{false};
        };

        void on_enter(wl_chassis_t *ctx) override;
        void on_execute(wl_chassis_t *ctx) override;
        void on_exit(wl_chassis_t *ctx) override;

      private:
        state_manual_t _state_manual;
        state_normal_t _state_normal;


    };

    fsm_t<owner> _main_fsm;
    state_passive_t _state_passive;
    fsm_active_t _state_active;

    wl_forward_kf_t _forward_kf;
    float _getover_elapsed{0.0f};
    float _getover_rearm_elapsed{GETOVER_REARM_DELAY};
    float _getover_target_length[2]{NORMAL_LENGTH_TARGET, NORMAL_LENGTH_TARGET};
};


} // namespace pyro
#endif // __PYRO_WL_CHASSIS_H__
