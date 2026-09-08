#ifndef PYRO_SENTRY_BOOSTER_H
#define PYRO_SENTRY_BOOSTER_H

#include "pyro_module_base.h"
#include "pyro_rc_base_drv.h"


#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"

#include "pyro_ins.h"
#include "pyro_dm_motor_drv.h"


namespace pyro
{
struct sentry_booster_cmd_t: public cmd_base_t
{

    bool fric_on;       // 摩擦轮开启
    uint8_t fire_count; // 拨弹计数器，替代 fire_enable
    

    //float trig_target_spd; // 新增：拨弹盘目标速度

    float real_hit_speed;
    bool multi_shoot;
    
    // float imu_yaw;
    // float imu_pitch;
        sentry_booster_cmd_t()
        :fric_on(false), fire_count(0), /*trig_target_spd(0.0f),*/real_hit_speed(0.0f),multi_shoot(false)
    {
    }

    virtual ~sentry_booster_cmd_t() = default;
};


struct sentry_booster_deps_t
{
    struct motor_deps_t
    {
        motor_base_t* trigger;
        motor_base_t* fric_left;
        motor_base_t* fric_right;
    };

    struct pid_deps_t
    {   pid_t* trig_pos_pid;
        pid_t* trig_spd_pid;
        pid_t* l_fric_pid;
        pid_t* r_fric_pid;
    };

    motor_deps_t motor_deps{};
    pid_deps_t pid_deps{};

};
 //=======上下文Context====
    struct booster_data_t
    {   
        float left_spd{0};
        float right_spd{0};
        float left_torque{0};
        float right_torque{0};

        float trigger_spd{0};
        float trigger_torque{0};
        float trigger_pos{0};
        int32_t trigger_count{0};
        float trigger_equal_pos{0};
    };

    // struct imu_data_t
    // { 
    //             // IMU 姿态反馈
    //     float current_pitch_rad{0};
    //     float current_yaw_rad{0};
    //     float current_yaw_radps{0};
    //     float current_pitch_radps{0};

    //     float current_roll_rad{0};
    //     float current_roll_radps{0};

    //     float target_pitch_rad{0};
    //     float target_yaw_rad{0};
    //     float target_yaw_radps{0};
    //     float target_pitch_radps{0};
    // };

    struct out_data_t
    { 
        float l_fric_torque{0};
        float r_fric_torque{0};
        float trigger_torque{0};
        
    };

    struct booster_data_ctx_t
    {
        booster_data_t current_data;
        booster_data_t target_data;

        
        out_data_t out_data;
        
        uint16_t current_fire_count{0};
        bool singgle_shoot;
        uint8_t error_count{0};

    };
    struct sentry_booster_context_t
    {
        sentry_booster_deps_t::motor_deps_t motor;  //motor为句柄，里面的数据为private，得用别的结构存储
        sentry_booster_deps_t::pid_deps_t pid;

        booster_data_ctx_t data;

        sentry_booster_cmd_t *cmd{};
    };

struct booster_params_t
{
    using CmdType    = sentry_booster_cmd_t;
    using ModuleDeps = sentry_booster_deps_t;
    using ModuleCtx  = sentry_booster_context_t;
};



class sentry_booster_t final
    : public module_base_t<sentry_booster_t, booster_params_t> { 
    friend class module_base_t<sentry_booster_t, booster_params_t>;

  public:
    sentry_booster_t(const sentry_booster_t &)            = delete;
    sentry_booster_t &operator=(const sentry_booster_t &) = delete;

  private:
    sentry_booster_t();
    ~sentry_booster_t() override = default;

    // --- 基类接口 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 派生方法 ---
    void _solve();
   

    void _fric_control();
    void _trigger_control();
    void _send_trig_command() const;
    void _send_fric_command() const;
    //void _communicate_booster() const;
    


   


    //==========FSM==========
 using owner = sentry_booster_t;

    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };
    // struct state_active_t final : public state_t<owner>
    // {
    //     void enter(owner *owner) override;
    //     void execute(owner *owner) override;
    //     void exit(owner *owner) override;
    // };
    
    struct fsm_active_t final : public fsm_t<owner>
    {
        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

        struct state_stay_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_wait_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct fsm_ready_t final : public fsm_t<owner>
        {

            struct singgle_shoot_t final : public state_t<owner>
            {
                void enter(owner *owner) override;
                void execute(owner *owner) override;
                void exit(owner *owner) override;
            };
            struct multi_shoot_t final : public state_t<owner>
            {
                void enter(owner *owner) override;
                void execute(owner *owner) override;
                void exit(owner *owner) override;
            };

            void on_enter(owner *owner) override;
            void on_execute(owner *owner) override;
            void on_exit(owner *owner) override;
            
            private:
            singgle_shoot_t _singgle_shoot;
            multi_shoot_t _multi_shoot;

        };

        struct state_stall_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        state_stay_t _state_stay;
        state_wait_t _state_wait;
        state_stall_t _state_stall;
        fsm_ready_t _state_ready;
        

    };


    fsm_t<owner> _main_fsm;
    state_passive_t _state_passive;
    fsm_active_t _state_active;
    

};

}//pyro
#endif
