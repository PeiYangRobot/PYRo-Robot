#ifndef __PYRO_INFANTRY2_GIMBAL_H__
#define __PYRO_INFANTRY2_GIMBAL_H__

#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"

#include "infantry2_config.h"

namespace pyro {

struct infantry2_gimbal_cmd_t final : public cmd_base_t {
    bool is_imu_control;

    //由控制器产生的期望值
    float target_pitch_angle, target_yaw_angle;
    float target_pitch_delta_angle, target_yaw_delta_angle; //位置环模拟移动速度

    enum class state_t {
        MANUAL,
        AUTO,
        ALIGN
    } state;

    infantry2_gimbal_cmd_t() 
        : is_imu_control(true), target_pitch_angle(0.0f), target_yaw_angle(0.0f),
            target_pitch_delta_angle(0.0f), target_yaw_delta_angle(0.0f), state(state_t::MANUAL) {}
};

struct infantry2_gimbal_deps_t {
    struct motor_deps_t {
        motor_base_t *pitch{nullptr};
        motor_base_t *yaw{nullptr};
    };

    struct pid_deps_t {
        pid_t *pitch_pos_pid{nullptr};
        pid_t *pitch_spd_pid{nullptr};
        pid_t *yaw_pos_pid{nullptr};
        pid_t *yaw_spd_pid{nullptr};
    };

    motor_deps_t motor{};
    pid_deps_t pid{};
};

struct infantry2_gimbal_data_t {
    //当前状态（电机反馈）
    float current_pitch_motor_rad{0.0f};
    float current_pitch_motor_radps{0.0f};
    float current_yaw_motor_rad{0.0f};
    float current_yaw_motor_radps{0.0f};

    //当前状态（imu反馈）
    float current_pitch_imu_rad{0.0f};
    float current_pitch_imu_radps{0.0f};
    float current_yaw_imu_rad{0.0f};
    float current_yaw_imu_radps{0.0f};
    float current_roll_imu_rad{0.0f};
    float current_roll_imu_radps{0.0f};

    //计算后的目标值
    float target_pitch_rad{0.0f};
    float target_pitch_radps{0.0f};
    float target_yaw_rad{0.0f};
    float target_yaw_radps{0.0f};

    float gravity_compensate{0.0f};

    //输出
    float out_pitch_torque{0.0f};
    float out_yaw_torque{0.0f};
};

struct infantry2_gimbal_ctx_t {
    infantry2_gimbal_deps_t deps;
    infantry2_gimbal_data_t data;
    infantry2_gimbal_cmd_t *cmd{};
};

struct infantry2_gimbal_module_params_t {
    using CmdType = infantry2_gimbal_cmd_t;
    using ModuleDeps = infantry2_gimbal_deps_t;
    using ModuleCtx = infantry2_gimbal_ctx_t;
};

class infantry2_gimbal_t final
        : public module_base_t<infantry2_gimbal_t, infantry2_gimbal_module_params_t> {
    friend class module_base_t<infantry2_gimbal_t, infantry2_gimbal_module_params_t>;

  public:
    infantry2_gimbal_t(const infantry2_gimbal_t&) = delete;
    infantry2_gimbal_t &operator=(const infantry2_gimbal_t&) = delete;

  private:
    infantry2_gimbal_t();
    ~infantry2_gimbal_t() override = default;

    // 基类接口
    status_t _init() override;           // 资源初始化
    void _update_feedback() override;    // 传感器/电机反馈更新
    void _fsm_execute() override;        // 状态机调度入口

    // 业务逻辑方法
    static void _mec_control(infantry2_gimbal_ctx_t *ctx);
    static void _imu_control(infantry2_gimbal_ctx_t *ctx);
    static void _send_motor_command(infantry2_gimbal_ctx_t *ctx);

    // 状态机定义（内嵌类）
    using owner = infantry2_gimbal_t;

    struct state_passive_t : public state_t<owner> {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct fsm_active_t : public fsm_t<owner> {
        struct state_manual_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_auto_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

      private:
        state_manual_t _manual_state;
        state_auto_t _auto_state;
    };

    state_passive_t _passive_state;
    fsm_active_t _active_state;
    fsm_t<owner> _main_fsm;
};

} // namespace pyro

#endif // __PYRO_INFANTRY2_GIMBAL_H__