#ifndef __PYRO_RUDDER_CHASSIS_H__
#define __PYRO_RUDDER_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_kin_rudder.h"
#include "pyro_motor_base.h"
#include "pyro_ins.h" // 新增 IMU 依赖
#include "rudder_config.h"
#include "pyro_power_control.h"



namespace pyro
{


// =========================================================
// 1. 命令定义
// =========================================================
struct rudder_cmd_t : cmd_base_t
{
    float vx;
    float vy;
    float wz;
    float delta_yaw;
    bool follow_yaw;
    float target_yaw_rad;
//————imu信息————————————
    float imu_yaw_rad;     
    float imu_yaw_radps;
    bool spinning;

    rudder_cmd_t()
        : vx(0), vy(0), wz(0),delta_yaw(0),follow_yaw(false),target_yaw_rad(0),
        imu_yaw_rad(0), imu_yaw_radps(0),spinning(false)
    {
    }

    virtual ~rudder_cmd_t() = default;
};


// =========================================================
// 2. 依赖定义
// =========================================================
struct rudder_deps_t
{
    // 电机句柄
    struct motor_deps_t
    {
        motor_base_t *rudder[4]{nullptr};
        motor_base_t *wheel[4]{nullptr};
        motor_base_t *yaw{nullptr};
    };

    // 算法对象
    struct pid_deps_t
    {
        pid_t *rudder_pos_pid[4]{nullptr};
        pid_t *rudder_spd_pid[4]{nullptr};
        pid_t *wheel_pid[4]{nullptr};
        pid_t *follow_yaw_pid{nullptr};
        pid_t *yaw_pos_pid{nullptr};
        pid_t *yaw_spd_pid{nullptr};

    };

    motor_deps_t motor_deps{};
    pid_deps_t pid_deps{};
    
};


// =========================================================
// 3. 数据上下文
// =========================================================

    struct rudder_data_t
    {   
        float rudder_pos[4]{0};
        float rudder_spd[4]{0};

        
        
        float wheel_pos[4]{0};
        float wheel_spd[4]{0};

        float rudder_torque[4]{0};
        float wheel_torque[4]{0};
        float yaw_spd{0};
        float yaw_error{0};    
        float yaw_torque{0}; 
    };

    struct imu_data_t
    { 
                // IMU 姿态反馈
        float current_pitch_rad{0};
        float current_roll_rad{0};
        float current_yaw_rad{0};
        float target_pitch_rad{0};
        float target_yaw_rad{0};

        float current_yaw_radps{0};
    };


    struct rudder_data_ctx_t
    {
    
        rudder_data_t current_data;
        rudder_data_t target_data;
        rudder_kin_t::rudder_states_t target_states;
        rudder_kin_t::rudder_states_t current_states;

        imu_data_t imu_data;


        
        float rudder_offset_rad[4]{0};
        bool yaw_online{false};

        /// @brief 底盘→云台 CAN 发送数据 (CAN ID 0x133)，4字节预留，格式待定
        uint8_t bus_tx_data[4]{0};

        power_node_t power_rmotor_data[4]{};
        power_node_t power_wmotor_data[4]{};
    };


// =========================================================
// 4. 模块上下文
// =========================================================

    struct rudder_context_t
    {
        rudder_deps_t::motor_deps_t motor;  //motor为句柄，里面的数据为private，得用别的结构存储
        rudder_deps_t::pid_deps_t pid;

        rudder_data_ctx_t data;
        
        
        rudder_cmd_t *cmd{};


    };


// =========================================================
// 5. 参数聚合
// =========================================================
struct rudder_params_t
{
    using CmdType    = rudder_cmd_t;
    using ModuleDeps = rudder_deps_t;
    using ModuleCtx  = rudder_context_t;
};



class rudder_chassis_t final
    : public module_base_t<rudder_chassis_t, rudder_params_t>
{
    friend class module_base_t<rudder_chassis_t, rudder_params_t>;
  public:
    rudder_chassis_t(const rudder_chassis_t &)            = delete;
    rudder_chassis_t &operator=(const rudder_chassis_t &) = delete;

     
    using data_ctx_t = rudder_data_ctx_t;


  private:
    rudder_chassis_t();
    ~rudder_chassis_t() override = default;

    // --- 基类接口 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 派生方法 ---
    static void _power_control_init();
    void _kinematics_solve();
    void _power_control();

    void _rudder_control();
    void _yaw_control();
    void _send_motor_command() const;
    void _communicate_gimbal() const;
    
    rudder_kin_t *_kinematics{nullptr};

    //--context--



   // rudder_context_t _ctx;


    using owner = rudder_chassis_t;

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

