#ifndef __PYRO_INFANTRY2_CHASSIS_H__
#define __PYRO_INFANTRY2_CHASSIS_H__

#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"
#include "pyro_kin_rudder.h"

#include "infantry2_config.h"

namespace pyro {

struct infantry2_chassis_cmd_t final : public cmd_base_t {
    float vx, vy, wz;
    
    enum class state_t {
        NORMAL,
        FOLLOW_YAW,
        SPIN
    } state;

    infantry2_chassis_cmd_t() : vx(0.0f), vy(0.0f), wz(0.0f) , state(state_t::NORMAL) {}
};

struct infantry2_chassis_deps_t {   
    struct motor_deps_t {
        motor_base_t *rudder[4]{nullptr};
        motor_base_t *wheel[4]{nullptr};
        motor_base_t *yaw{nullptr};
    };

    struct pid_deps_t {
        pid_t *rud_pos_pid[4]{nullptr};
        pid_t *rud_spd_pid[4]{nullptr};
        pid_t *wheel_pid[4]{nullptr};
        pid_t *yaw_follow_pid{nullptr};
    };

    motor_deps_t motor{};
    pid_deps_t pid{};
};

struct infantry2_chassis_data_t {
    rudder_kin_t::rudder_states_t current_states{};
    rudder_kin_t::rudder_states_t target_states{};

    float target_rudder_radps[4]{0.0f};
    float current_rudder_radps[4]{0.0f};

    float target_wheel_radps[4]{0.0f};
    float current_wheel_radps[4]{0.0f};

    float current_yaw_rad{0.0f};
    float current_yaw_radps{0.0f};

    float out_wheel_torque[4]{0.0f};
    float out_rudder_torque[4]{0.0f};

    float spin_rata{1.0f}, move_rata{1.0f};
};

struct infantry2_chassis_ctx_t {
    infantry2_chassis_deps_t deps;
    infantry2_chassis_data_t data;
    infantry2_chassis_cmd_t *cmd{};
};

struct infantry2_chassis_module_params_t {
    using CmdType = infantry2_chassis_cmd_t;
    using ModuleDeps = infantry2_chassis_deps_t;
    using ModuleCtx = infantry2_chassis_ctx_t;
};

class infantry2_chassis_t final
        : public module_base_t<infantry2_chassis_t, infantry2_chassis_module_params_t> {
    friend class module_base_t<infantry2_chassis_t, infantry2_chassis_module_params_t>;

  public:
    infantry2_chassis_t(const infantry2_chassis_t&) = delete;
    infantry2_chassis_t &operator=(const infantry2_chassis_t&) = delete;

  private:
    infantry2_chassis_t();
    ~infantry2_chassis_t() override = default;

    inline static rudder_kin_t _kinematics{infantry2_chassis::WHEELBASE, infantry2_chassis::TRACK_WIDTH};

    // 基类接口
    status_t _init() override;           // 资源初始化
    void _update_feedback() override;    // 传感器/电机反馈更新
    void _fsm_execute() override;        // 状态机调度入口

    // 业务逻辑方法
    static void _chassis_control(infantry2_chassis_ctx_t *ctx);
    static void _send_motor_command(infantry2_chassis_ctx_t *ctx);
    static void _limit_steer_rate(infantry2_chassis_ctx_t *ctx);

    // 状态机定义（内嵌类）
    using owner = infantry2_chassis_t;

    struct state_passive_t : public state_t<owner> {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct fsm_active_t : public fsm_t<owner> {
        struct state_normal_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_follow_yaw_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_spin_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

      private:
        state_normal_t _normal_state;
        state_follow_yaw_t _follow_yaw_state;
        state_spin_t _spin_state;
    };

    state_passive_t _passive_state;
    fsm_active_t _active_state;
    fsm_t<owner> _main_fsm;
};

} // namespace pyro

#endif // __PYRO_INFANTRY2_CHASSIS_H__