#ifndef __PYRO_ENGINEER_CHASSIS_H__
#define __PYRO_ENGINEER_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_kin_mec.h"       // 麦轮运动学
#include "pyro_motor_base.h"
#include "pyro_power_control.h"
#include "pyro_powermeter.h"
//#include "pyro_supercap_drv.h"
// 超级电容驱动暂时不使用，后续可按需添加
#include "engineer_config.h"
#include "FreeRTOS.h"
#include "task.h"


namespace pyro
{

// =========================================================
// 前置声明
// =========================================================
struct engineer_cmd_t;
struct engineer_deps_t;
struct engineer_context_t;


// =========================================================
// 1. 命令定义
// =========================================================

// --- 底盘行走命令 ---
struct chassis_cmd_t
{
    float vx; // x轴方向速度 m/s
    float vy; // y轴方向速度 m/s
    float wz; // 旋转角速度 rad/s
};

// --- 摇臂相关枚举 ---
enum class lift_mode_t : uint8_t
{
    AUTO = 0,   // 自动模式
    MANUAL,     // 手动模式
};

enum class lift_action_t : uint8_t
{
    HOLD = 0,   // 保持当前位置
    DEPLOY,     // 放下摇臂
    RETRACT,    // 收起摇臂
};

enum class lift_manual_mod_t : uint8_t
{
    HOLD = 0,   // 保持
    UP,         // 向上
    DOWN,       // 向下
};
enum class lift_calib_state_t : uint8_t
{
    IDLE = 0,       //空闲/已完成
    CALIBRATING,    //正向找零点
    VERIFYING,      //区间核验
    RETRY_BACKOFF,  //失败,回退
    CALIB_DONE,     //校准成功
    CALIB_FAILED    //校准失败
};
struct lift_manual_t
{
    lift_manual_mod_t left_mod;   // 左摇臂运动状态
    lift_manual_mod_t right_mod;  // 右摇臂运动状态
};

struct lift_cmd_t
{
    lift_mode_t mode;             // 自动/手动
    lift_action_t auto_action;    // 自动模式动作
    lift_manual_t manual;         // 手动模式控制
};

// --- 矿仓命令 ---
enum class magazine_pos_t : uint8_t
{
    POS_1 = 0,
    POS_2,
    POS_3,
    POS_4,
};

struct magazine_cmd_t
{
    magazine_pos_t target_pos;    // 目标位置（4档）
};

// --- 总命令结构体 ---
struct engineer_cmd_t final : public cmd_base_t
{
    chassis_cmd_t  chassis;       // 底盘行走
    lift_cmd_t     lift;          // 摇臂
    magazine_cmd_t magazine;      // 矿仓

    engineer_cmd_t()
    {
        // 默认安全值：全部停止/保持
        chassis.vx = 0.0f;
        chassis.vy = 0.0f;
        chassis.wz = 0.0f;

        lift.mode = lift_mode_t::AUTO;
        lift.auto_action = lift_action_t::HOLD;
        lift.manual.left_mod  = lift_manual_mod_t::HOLD;
        lift.manual.right_mod = lift_manual_mod_t::HOLD;

        magazine.target_pos = magazine_pos_t::POS_1;
    }
};


// =========================================================
// 2. 依赖定义
// =========================================================
struct engineer_deps_t
{
    // 电机句柄
    struct motor_deps_t
    {
        motor_base_t *mecanum[4]{nullptr};    // 4个麦轮电机 FL, FR, BL, BR
        motor_base_t *lift[2]{nullptr};       // 2个摇臂电机 左, 右
        motor_base_t *magazine{nullptr};      // 矿仓电机
    };

    // PID控制对象
    struct pid_deps_t
    {
        pid_t *mecanum_pid[4]{nullptr};       // 麦轮速度环PID
        pid_t *lift_pos_pid[2]{nullptr};      // 摇臂位置环PID
        pid_t *lift_vel_pid[2]{nullptr};      // 摇臂速度环PID
        pid_t *magazine_pos_pid{nullptr};     // 矿仓位置环PID
        pid_t *magazine_vel_pid{nullptr};     // 矿仓速度环PID
    };

    motor_deps_t motor_deps{};
    pid_deps_t   pid_deps{};
};


// =========================================================
// 3. 上下文数据定义
// =========================================================

// 运行时数据
struct engineer_data_ctx_t
{
    // --- 麦轮反馈 ---
    bool  wheel_online[4]{};
    float current_wheel_rpm[4]{};
    float current_wheel_torque[4]{};
    float current_wheel_temp[4]{};
    float target_wheel_rpm[4]{};
    float out_wheel_torque[4]{};

    // 里程计（反算实际速度）
    float real_vx{0.0f};
    float real_vy{0.0f};
    float real_wz{0.0f};

    // --- 摇臂反馈 ---
    bool  lift_online[2]{};
    float current_lift_angle[2]{};      // 当前角度 rad
    float current_lift_speed[2]{};      // 当前角速度 rad/s
    float target_lift_angle[2]{};       // 目标角度 rad
    float out_lift_torque[2]{};         // 输出扭矩
    // --- 摇臂复位控制---
    //1. 多圈展开部分
    float lift_last_raw_pos[2];         // 上一周期原始角度
    int32_t lift_cycle_counter[2];      // 圈数累计
    float lift_unwrap_angle[2];         //真正意义上可以使用的角度

    //2.零点偏移
    float lift_zero_offset[2];          //零点偏移
    bool lift_zero_valid[2];            //零点是否有效

    //3.软件限位
    float lift_min_angle;               //  下限，放下的位置
    float lift_max_angle;               //  上限

    //4. 校准状态机
    lift_calib_state_t lift_calib_state[2];
    int     lift_calib_retry[2];        // 已经重试的次数
    uint32_t lift_stall_timer[2];       // 堵转的时间
    float   lift_backoff_target[2];     //回退的目标角度
    uint32_t lift_calib_start_time[2];  // 校准开始时间


    // --- 矿仓反馈 ---
    bool  magazine_full[4];//用来判断我的四个矿仓是否有东西
    bool  magazine_online{};
    bool  magazine_ready{};             // 是否就绪
    float current_magazine_angle{0.0f}; // 当前角度
    float target_magazine_angle{0.0f};  // 目标角度
    float out_magazine_torque{0.0f};    // 输出扭矩
    float current_magazine_speed{0.0f}; // 当前速度
    float target_magazine_speed{0.0f};  // 目标速度
    uint32_t magazine_mask{0};
    // --- 功率相关 ---
    float total_predicted_power{0.0f};
    float buffer_energy{0.0f};
};

// 完整上下文
struct engineer_context_t
{
    engineer_deps_t::motor_deps_t motor;
    engineer_deps_t::pid_deps_t   pid;
    engineer_data_ctx_t           data;

    engineer_cmd_t *cmd{nullptr};             // 当前命令指针
    powermeter_drv_t *powermeter{nullptr};    // 功率计
    powermeter_data powermeter_feedback{};    // 功率计反馈
    power_node_t *power_motor_data[4]{};      // 功控节点（4个麦轮）

};


// =========================================================
// 4. ModuleParams 聚合体（两参数版本必须）
// =========================================================
// 把 CmdType / ModuleDeps / ModuleCtx 打包到一个结构体里
// 作为 module_base_t 的第二个模板参数
struct engineer_params_t
{
    using CmdType    = engineer_cmd_t;
    using ModuleDeps = engineer_deps_t;
    using ModuleCtx  = engineer_context_t;
};


// =========================================================
// 5. 工程车底盘类（两参数版本）
// =========================================================
class engineer_chassis_t final
    : public module_base_t<engineer_chassis_t, engineer_params_t>
{
    // 友元：让基类能调用私有虚函数
    friend class module_base_t<engineer_chassis_t, engineer_params_t>;
    friend class jcom_drv_t;  // 调试上位机

public:
    // 单例：禁止拷贝
    engineer_chassis_t(const engineer_chassis_t &)            = delete;
    engineer_chassis_t &operator=(const engineer_chassis_t &) = delete;

    // 获取上下文（非const版本，基类只有const版本）
    [[nodiscard]] engineer_context_t &get_ctx();
    void lift_start_calibrate(int i);//外部触发校准
    bool lift_is_caliv_done(int i);//查询校准是否完成

private:
    // 构造函数私有（单例模式，通过 instance() 访问）
    engineer_chassis_t();
    ~engineer_chassis_t() override = default;

    // =============================================
    // 三个核心虚函数（基类要求必须实现）
    // =============================================
    status_t _init() override;             // 初始化回调（启动时调用一次）
    void _update_feedback() override;      // 反馈更新回调（每周期读传感器）
    void _fsm_execute() override;          // 状态机执行回调（每周期跑控制）

    // =============================================
    // 内部辅助函数
    // =============================================
    void _power_control_init();            // 功率控制初始化
    void _kinematics_solve();              // 麦轮逆运动学求解
    void _mecanum_control();               // 麦轮速度环PID控制
    void _lift_control();                  // 摇臂控制（自动/手动分发）
    void _magazine_control();              // 矿仓位置控制
    void _power_control();                 // 功率限制
    void _send_motor_command() const;      // 发送所有电机指令
    void _lift_calibrate_tick(int i);

    // 运动学求解器
    mecanum_kin_t *_kinematics{nullptr};

    // =============================================
    // FSM 状态机
    // =============================================
    using owner = engineer_chassis_t;

    // 被动状态（无力/安全模式）
    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    // 主动状态（工作模式）
    struct state_active_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    // 状态实例
    state_passive_t _state_passive;
    state_active_t  _state_active;
    fsm_t<owner>    _main_fsm;
};

} // namespace pyro

#endif // __PYRO_ENGINEER_CHASSIS_H__
