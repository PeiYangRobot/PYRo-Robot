#ifndef PYRO_BOOSTER_H
#define PYRO_BOOSTER_H

#include "pyro_algo_pid.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_module_base.h"
#include "bullet_speed_control.h"
#include "heat_control.h"
#include "booster_config.h"
#include "pyro_board_drv.h"


namespace pyro
{

enum class FireState
{
    Passive,       ///< 休眠: 摩擦轮停, 拨弹锁位
    SpinUp,        ///< 摩擦轮启动中
    Ready,         ///< 就绪: 摩擦轮已达速, 等待开火指令
    CaliReverse,   ///< 校准: 反转寻找机械死区
    CaliForward,   ///< 校准: 正转回到零点
    SingleFire,    ///< 单发: 拨弹盘推进一发
    BurstFire,     ///< 连发: 速度环全速连发
};

enum class ShootEvent
{
    NONE = 0,
    FRIC_TOGGLE, // 摩擦轮开启/关闭 切换事件
    SINGLE_FIRE, // 单发事件
    BURST_START, //连发开始
    BURST_END,   //连发结束
};

//命令结构体
struct wl_booster_cmd_t
{
    ShootEvent event = ShootEvent::NONE;
    pyro::cmd_base_t::mode_t mode;
    struct
    {
        uint32_t burstShot : 1;
    } burst_state;
};


//依赖对象部分
struct wl_booster_deps_t
{
    // 电机句柄
    struct motor_deps_t
    {
        pyro::dji_motor_drv_t *fric1{nullptr};
        pyro::dji_motor_drv_t *fric2{nullptr};
        pyro::dji_motor_drv_t *trigger{nullptr};
    };

    // 算法对象 (串级 PID)
    struct pid_deps_t
    {
        pid_t *fric1_spd{nullptr};
        pid_t *fric2_spd{nullptr};
        pid_t *trigger_spd{nullptr};
        pid_t *trigger_pos{nullptr};
    };

    motor_deps_t motor_deps{};
    pid_deps_t pid_deps{};
    BulletSpeedControl* speed_control_deps;
    HeatControl* heat_control_deps;
};


//上下文部分
struct BoosterMotorState
{
    float pos;    // 角度 (rad)
    float vel;    // 角速度 (rad/s)
    float torque; // 真实反馈力矩 (N.m) 或 电流 (A)
    int8_t temp;  // 温度 (°C)
    bool online;
};

struct BoosterState
{
    BoosterMotorState fric[2];
    BoosterMotorState trigger;
    bool enable;
    float trigger_rad;        //拨弹盘的位置(范围为2PI * 电机减速比)
    int8_t triggerRound;     //拨弹盘“电机”转过的圈数，用于换算至拨弹盘实际角度
};

struct TargetBoosterState
{
    float targetFricSpeed;              ///< 摩擦轮目标转速 (rad/s)
    float targetTriggerRad;             ///< 拨弹盘目标位置 (用于位置环) (最大为72*PI)
    //float triggerOffset;                ///< 拨弹盘零点偏移 (校准后确定)
    float targetTriggerSpeed;           ///< 拨弹盘目标转速 (用于速度环)
    bool useTriggerSpeedLoopOnly;       ///< true=绕过位置环, 仅速度环 (连发/校准)
    bool burst_state;
};

struct BoosterOutput
{
    float fric1Current;
    float fric2Current;
    float triggerCurrent;
};

struct booster_data_ctx_t final : public cmd_base_t
{
    BoosterState motor_state;
    TargetBoosterState target_state;
    BoosterOutput output;
    FireState fsm_state;
    ShootEvent cmd_event;
    bool isCalibrated              = false;  ///< 是否已完成拨弹盘校准
    float dt;
};

struct wl_booster_ctx_t
{
    booster_data_ctx_t data;
};

struct wl_booster_param_t
{
    using CmdType    = wl_booster_cmd_t;
    using ModuleDeps = wl_booster_deps_t;
    using ModuleCtx  = wl_booster_ctx_t;
};




class wl_booster_t final
    : public module_base_t<wl_booster_t, wl_booster_param_t>
{
    friend class module_base_t<wl_booster_t, wl_booster_param_t>;

    public:
    wl_booster_t(const wl_booster_t &)            = delete;
    wl_booster_t &operator=(const wl_booster_t &) = delete;
    using data_ctx_t                              = booster_data_ctx_t;
    using ctx_t                                   = wl_booster_ctx_t;

    private:
    wl_booster_t();
    ~wl_booster_t() override = default;

    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;


    //辅助函数
    void calculateFricCurrents();
    void calculateTriggerCurrents(bool useTriggerSpeedLoopOnly);
    void sendCurrents();

    using owner = wl_booster_t;

    //状态机定义
    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };
    struct fsm_active_t final : public fsm_t<owner>
    {
        struct state_spin_up_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_ready_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_cali_reverse_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_cali_forward_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_singlefire_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_burstfire_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        void on_enter(wl_booster_t *ctx) override;
        void on_execute(wl_booster_t *ctx) override;
        void on_exit(wl_booster_t *ctx) override;

      private:
      state_spin_up_t       _state_spin_up;
      state_ready_t         _state_ready;
      state_cali_reverse_t  _state_cali_reverse;
      state_cali_forward_t  _state_cali_forward;
      state_singlefire_t    _state_singlefire;
      state_burstfire_t     _state_burstfire;

    };

    //状态机实例
    fsm_t<owner> _main_fsm;
    state_passive_t _state_passive;
    fsm_active_t _state_active;

};
}


#endif
