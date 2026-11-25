#ifndef PYRO_GIMBAL_H
#define PYRO_GIMBAL_H

#include "pyro_algo_pid.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_module_base.h"


namespace pyro
{

enum MotionState
{
    Relax,
    Align,
    Manual,
    Auto,
};
//命令结构体部分
struct wl_gimbal_cmd_t {
    // 【修改】将绝对角度改为期望角速度 (rad/s)
    float yawVel;
    float pitchVel;

    // --- 新增：视觉模式使用的高阶期望 ---
    float targetYaw;
    float targetPitch;
    float targetYawSpeed;        // 目标角速度 (用于前馈)
    // ------------------------------------
    uint32_t timestamp;

    pyro::cmd_base_t::mode_t mode;
    MotionState state_cmd;

    bool chassis_is_ready;
    bool chassis_is_rescuing;


};


//依赖对象部分
struct wl_gimbal_deps_t
{
    // 电机句柄
    struct motor_deps_t
    {
        pyro::dm_motor_drv_t *pitch{nullptr};
        pyro::dji_motor_drv_t *yaw{nullptr};
    };

    // 算法对象 (串级 PID)
    struct pid_deps_t
    {
        pid_t *pitch_spd{nullptr};
        pid_t *pitch_pos{nullptr};
        pid_t *yaw_pos{nullptr};
        pid_t *yaw_spd{nullptr};

        // --- 新增：自瞄专用的串级 PID ---
        pid_t *auto_pitch_pos{nullptr};
        pid_t *auto_yaw_pos{nullptr};
        pid_t *auto_yaw_spd{nullptr};
    };

    motor_deps_t motor_deps{};
    pid_deps_t pid_deps{};
};


//上下文部分

struct ImuState {
    float accel[3], gyro[3];
    float roll, pitch, yaw;
    float temperature;
    uint32_t timestamp;
};

struct GimbalMotorState {
    float pos;    // 角度 (rad)
    float vel;    // 角速度 (rad/s)
    float torque; // 真实反馈力矩 (N.m) 或 电流 (A)
    int8_t temp;  // 温度 (°C)
    bool online;
};

struct GimbalState {
    GimbalMotorState yaw;
    GimbalMotorState pitch;
};

struct GimbalOutput {
    float yawCurrent;
    float pitchTorque;

    bool pitchEn;
    bool yawEn;
};

//自瞄指令
struct GimbalTelemetry {
    float targetYawRad;
    float targetPitchRad;

    float target_yaw_vel;
    float target_pitch_vel;
};

struct wl_gimbal_data_ctx_t final : public cmd_base_t
{
    ImuState imu;
    GimbalState state;
    GimbalOutput output;
    GimbalTelemetry telem;
    float dt;
    uint8_t motionState;

    bool chassis_is_ready;
    bool chassis_is_rescuing;

};

struct wl_gimbal_ctx_t
{
    wl_gimbal_data_ctx_t data;
};

struct wl_gimbal_param_t
{
    using CmdType    = wl_gimbal_cmd_t;
    using ModuleDeps = wl_gimbal_deps_t;
    using ModuleCtx  = wl_gimbal_ctx_t;
};




class wl_gimbal_t final
    : public module_base_t<wl_gimbal_t, wl_gimbal_param_t>
{
    friend class module_base_t<wl_gimbal_t, wl_gimbal_param_t>;

    public:
    wl_gimbal_t(const wl_gimbal_t &)            = delete;
    wl_gimbal_t &operator=(const wl_gimbal_t &) = delete;
    using data_ctx_t                              = wl_gimbal_data_ctx_t;
    using ctx_t                                   = wl_gimbal_ctx_t;

    private:
    wl_gimbal_t();
    ~wl_gimbal_t() override = default;

    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;



    //辅助函数
    void set_pitchstate(bool enable);
    void set_yawstate(bool enable);

    void updatePitch();
    void updateYaw();
    void align_updatePitch();
    void align_updateYaw(float target_motor_yaw);

    void _send_motor_command();
    float wrapAngle(float angle);//角度归一化到正负PI



    using owner = wl_gimbal_t;

    //状态机定义

    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };
    struct fsm_active_t final : public fsm_t<owner>
    {
        struct state_align_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_manual_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        void on_enter(wl_gimbal_t *owner) override;
        void on_execute(wl_gimbal_t *owner) override;
        void on_exit(wl_gimbal_t *owner) override;

      private:
        state_manual_t _state_manual;
        state_align_t  _state_align;
    };


    //状态机实例
    fsm_t<owner> _main_fsm;
    state_passive_t _state_passive;
    fsm_active_t _state_active;

};
}



#endif