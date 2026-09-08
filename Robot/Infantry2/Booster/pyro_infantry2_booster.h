#ifndef __PYRO_INFANTRY2_BOOSTER_H__
#define __PYRO_INFANTRY2_BOOSTER_H__

#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"
#include <cstdint>

#include "infantry2_config.h"

namespace pyro {

struct infantry2_booster_cmd_t final : public cmd_base_t {
    bool is_fric_on;            // 摩擦轮是否开启
    bool continue_shoot;        // 触发连发
    bool heat_control_on{false}; // 热量控制开关，false时跳过所有热量限制（调试用）
    bool fire_licence{};        // 发射许可，为false时拨弹盘绝对不允许转动

    infantry2_booster_cmd_t() : is_fric_on(false), continue_shoot(false), fire_licence(false) {}
};

struct infantry2_booster_deps_t {
    struct motor_deps_t {
        motor_base_t *fric[2]{nullptr};
        motor_base_t *trigger{nullptr};
    };

    struct pid_deps_t {
        pid_t *bullet_spd_pid{nullptr};
        pid_t *fric_pid[2]{nullptr};
        pid_t *trigger_pos_pid{nullptr};
        pid_t *trigger_spd_pid{nullptr};
    };

    motor_deps_t motor{};
    pid_deps_t pid{};
};

struct infantry2_booster_data_t {
    float target_fric_radps[2]{0.0f};
    float target_trigger_rad{0.0f}, target_trigger_radps{0.0f};

    float current_fric_radps[2]{0.0f};
    float current_trigger_rad, current_trigger_radps{0.0f};
    float current_trigger_torque;

    float last_trigger_rad;
    
    float out_fric_torque[2]{0.0f};
    float out_trigger_torque{0.0f};

    bool is_calibrated{false};

    uint32_t block_start_tick{0};
    volatile uint32_t notify_ev{0};  // 一次性事件位（跨任务置位，模块独享消费）

    enum class trigger_pid_mode_t {
        POS,
        SPD
    } trigger_mode{trigger_pid_mode_t::POS};
};

struct infantry2_booster_ctx_t {
    infantry2_booster_deps_t deps;
    infantry2_booster_data_t data;
    infantry2_booster_cmd_t *cmd{};
};

struct infantry2_booster_module_params_t {
    using CmdType = infantry2_booster_cmd_t;
    using ModuleDeps = infantry2_booster_deps_t;
    using ModuleCtx = infantry2_booster_ctx_t;
};

class infantry2_booster_t final
        : public module_base_t<infantry2_booster_t, infantry2_booster_module_params_t> {
    friend class module_base_t<infantry2_booster_t, infantry2_booster_module_params_t>;

  public:
    static constexpr uint32_t EVENT_BIT_SINGLE_SHOOT = (1 << 0);  // 单发事件位
    void notify_single_shoot();  // 投递单发事件（遥控/自瞄调用）

    infantry2_booster_t(const infantry2_booster_t&) = delete;
    infantry2_booster_t &operator=(const infantry2_booster_t&) = delete;

  private:
    infantry2_booster_t();
    ~infantry2_booster_t() override = default;

    // 基类接口
    status_t _init() override;           // 资源初始化
    void _update_feedback() override;    // 传感器/电机反馈更新
    void _fsm_execute() override;        // 状态机调度入口

    // 业务逻辑方法
    static bool _is_fric_ready(infantry2_booster_ctx_t *ctx);
    static void _fric_control(infantry2_booster_ctx_t *ctx);
    static void _trigger_control(infantry2_booster_ctx_t *ctx);
    static void _send_fric_command(infantry2_booster_ctx_t *ctx);
    static void _send_trigger_command(infantry2_booster_ctx_t *ctx);
    
    // 状态机定义（内嵌类）
    using owner = infantry2_booster_t;

    struct state_passive_t : public state_t<owner> {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct fsm_active_t : public fsm_t<owner> {
        struct state_waiting_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_ready_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_single_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_continue_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;           
        };

        struct state_cali_reverse_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_cali_forward_t : public state_t<owner> {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;
        
      private:
        state_waiting_t _waiting_state;
        state_ready_t _ready_state;
        state_single_t _single_state;
        state_continue_t _continue_state;
        state_cali_reverse_t _cali_reverse_state;
        state_cali_forward_t _cali_forward_state;
    };

    state_passive_t _passive_state;
    fsm_active_t _active_state;
    fsm_t<owner> _main_fsm;
};

} // namespace pyro

#endif // __PYRO_INFANTRY2_BOOSTER_H__