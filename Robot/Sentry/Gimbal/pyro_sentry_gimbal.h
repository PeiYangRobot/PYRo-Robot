#ifndef __PYRO_SENTRY_GIMBAL_H__
#define __PYRO_SENTRY_GIMBAL_H__ 




#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"

#include "pyro_ins.h"
#include "pyro_dm_motor_drv.h"
#include "gimbal_config.h"

namespace pyro
{
struct sentry_gimbal_cmd_t: public cmd_base_t
{
    float delta_yaw;
    float delta_pitch;
    bool a_mode;
    bool b_mode;

    /// @brief 从底盘接收的数据 (CAN ID 0x133)，4字节预留，格式待定
    uint8_t chassis_data[4]{0};
    

    // float imu_yaw;
    // float imu_pitch;
        sentry_gimbal_cmd_t()
        : delta_yaw(0), delta_pitch(0), a_mode(false), b_mode(false)
    {
    }

    virtual ~sentry_gimbal_cmd_t() = default;
};


struct sentry_gimbal_deps_t
{
    struct motor_deps_t
    {
        motor_base_t* motor_pitch;
        motor_base_t* motor_yaw;
    };

    struct pid_deps_t
    {
        pid_t* pitch_spd_pid;
        pid_t* pitch_pos_pid;
        pid_t* yaw_spd_pid;
        pid_t* yaw_pos_pid;
    };

    motor_deps_t motor_deps{};
    pid_deps_t pid_deps{};

    float pitch_offset_rad;
    float yaw_offset_rad;

    float pitch_max_rad;
    float pitch_min_rad;

    float yaw_min_rad;
    float yaw_max_rad;
};
 //=======上下文Context====
    struct gimbal_data_t
    {   
        float yaw_spd{0};
        float yaw_pos{0};
        float yaw_torque{0};

        float pitch_spd{0};
        float pitch_pos{0};
        float pitch_torque{0};
        
    };

    struct imu_data_t
    { 
                // IMU 姿态反馈
        float current_pitch_rad{0};
        float current_yaw_rad{0};
        float current_yaw_radps{0};
        float current_pitch_radps{0};

        float current_roll_rad{0};
        float current_roll_radps{0};

        float target_pitch_rad{0};
        float target_yaw_rad{0};
        float target_yaw_radps{0};
        float target_pitch_radps{0};
    };

    struct out_data_t
    { 
        float pitch_torque{0};
        float yaw_torque{0};
    };

    struct gimbal_data_ctx_t
    { 

        gimbal_data_t current_data;
        gimbal_data_t target_data;

        imu_data_t imu_data;
        out_data_t out_data;


        float gimbal_yaw_offset_rad{0};
        float gimbal_pitch_offset_rad{0};
        bool yaw_online{false};

        float pitch_max_rad;
        float pitch_min_rad;
    
        float yaw_min_rad;
        float yaw_max_rad;
    };
    struct sentry_gimbal_context_t
    {
        sentry_gimbal_deps_t::motor_deps_t motor;  //motor为句柄，里面的数据为private，得用别的结构存储
        sentry_gimbal_deps_t::pid_deps_t pid;
        gimbal_data_ctx_t data;
        sentry_gimbal_cmd_t *cmd{};

    };

struct gimbal_params_t
{
    using CmdType    = sentry_gimbal_cmd_t;
    using ModuleDeps = sentry_gimbal_deps_t;
    using ModuleCtx  = sentry_gimbal_context_t;
};





class sentry_gimbal_t final
    : public module_base_t<sentry_gimbal_t, gimbal_params_t> { 
    friend class module_base_t<sentry_gimbal_t, gimbal_params_t>;

  public:
    sentry_gimbal_t(const sentry_gimbal_t &)            = delete;
    sentry_gimbal_t &operator=(const sentry_gimbal_t &) = delete;

  private:
    sentry_gimbal_t();
    ~sentry_gimbal_t() override = default;

    // --- 基类接口 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 派生方法 ---
    void _solve();
   

    void _gimbal_control();
    void _send_motor_command() const;
    //void _communicate_gimbal() const;
    


   



    //==========FSM==========
 using owner = sentry_gimbal_t;

    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };
    struct state_active_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };
    


    fsm_t<owner> _main_fsm;
    state_passive_t _state_passive;
    state_active_t _state_active;
    

};

















}//pyro
















#endif
