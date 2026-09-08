#include "pyro_engineer_chassis.h"

#include "pyro_algo_common.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_referee.h"
#include <algorithm>
#include <arm_math.h>
#include <cmath>

namespace pyro
{

// =========================================================
// 构造函数
// =========================================================
engineer_chassis_t::engineer_chassis_t()
    : module_base_t("engineer_chassis",512,1024,task_base_t::priority_t::HIGH)

{
    _ctx = {};
    // _ctx 是基类的 protected 成员，会被默认构造
}

// =========================================================
// 获取上下文（非 const 版本，基类只有 const 版本）
// =========================================================
engineer_context_t &engineer_chassis_t::get_ctx()
{
    return _ctx;  // 直接访问基类的 protected 成员 _ctx
}

// =========================================================
// _init()：初始化回调（只调用一次）
// =========================================================
//这里的功率控制我先不加上,怕对我的最基本的代码功能产生影响

status_t engineer_chassis_t::_init()
{
    // 1. 把外部依赖拷贝到上下文
    _ctx.motor = _module_deps.motor_deps;
    _ctx.pid   = _module_deps.pid_deps;

    // 2. new 麦轮运动学求解器
    _kinematics = new mecanum_kin_t(WHEELBASE, TRACK_WIDTH);

    // 3. new 功率计并初始化（如需启用功率计反馈，取消注释并确认 CAN ID）
    // _ctx.powermeter = new powermeter_drv_t(0x212, bsp_can::can2);
    // _ctx.powermeter->init();

    // 4. 功率控制初始化（注册4个麦轮到功率控制器）
    _power_control_init();
    // 5. 摇臂初始化
    for (int i = 0; i < 2; i++) {
        _ctx.data.lift_last_raw_pos[i]     = 0.0f;
        _ctx.data.lift_cycle_counter[i]    = 0;
        _ctx.data.lift_unwrap_angle[i]     = 0.0f;
        _ctx.data.lift_zero_offset[i]      = 0.0f;
        _ctx.data.lift_zero_valid[i]       = false;
        _ctx.data.lift_calib_state[i]      = lift_calib_state_t::IDLE;
        _ctx.data.lift_calib_retry[i]      = 0;
        _ctx.data.lift_stall_timer[i]      = 0;
        _ctx.data.target_lift_angle[i]     = 0.0f;
        _ctx.data.out_lift_torque[i]       = 0.0f;
    }
    _ctx.data.lift_min_angle = LIFT_ANGLE_MIN;
    _ctx.data.lift_max_angle = LIFT_ANGLE_MAX;
    // 6.矿仓初始化
    //默认我的矿仓是下力的
    if(_ctx.motor.magazine != nullptr){
        _ctx.motor.magazine->disable();
    }

    //如果初始化这里都没什么问题，直接返回PYRO_OK
    return PYRO_OK;
}

// =========================================================
// 功率控制初始化
// =========================================================
void engineer_chassis_t::_power_control_init()
{
    /*
     * 4个麦轮的功率拟合参数（保守默认值，需根据实际电机校准）
     *
     * power_fit_params_t 字段说明：
     *   k1: 机械功率系数 (cmd * rpm)
     *   k2: 铜损系数     (cmd^2)
     *   k3: 高频损耗系数 (rpm^2)
     *   k4: 低频损耗系数 (|rpm|)
     *   k5: 静态基础功耗 (W)
     *   alpha: 电阻温度系数 (铜默认0.00393)
     *
     * TODO: 替换为实际标定参数。老版本注释里的4参数模型
     *       (k1*扭矩² + k2*转速*扭矩 + k3*|扭矩| + k4) 与本结构
     *       语义不同，需重新拟合或手动映射。
     */
    power_fit_params_t params[4] = {
        {0.001f, 0.01f, 0.0f, 0.0f, 0.5f, 0.00393f},  // [0] FL
        {0.001f, 0.01f, 0.0f, 0.0f, 0.5f, 0.00393f},  // [1] FR
        {0.001f, 0.01f, 0.0f, 0.0f, 0.5f, 0.00393f},  // [2] BL
        {0.001f, 0.01f, 0.0f, 0.0f, 0.5f, 0.00393f},  // [3] BR
    };

    auto &pc = power_controller_t::get_instance();
    for (int i = 0; i < 4; ++i) {
        _ctx.power_motor_data[i] = pc.register_motor(params[i]);
    }

    // 配置缓冲能量环：安全能量60J，PID参数（保守值）
    // 无裁判系统/超级电容时，此环影响较小，主要靠 solve() 的平均功率限制
    pc.config_buffer_loop(60.0f, 0.1f, 0.01f, 0.0f);
}

// =========================================================
// _update_feedback()：反馈更新（每周期先执行）
// =========================================================
void engineer_chassis_t::_update_feedback()
{
    // ===== 1. 更新所有电机反馈 =====

    // 麦轮电机
    for (auto *motor : _ctx.motor.mecanum)
    {
        motor->update_feedback();
    }

    // 摇臂电机
    for (auto *motor : _ctx.motor.lift)
    {
        if (motor != nullptr)
            motor->update_feedback();
    }

    // 矿仓电机
    _ctx.motor.magazine->update_feedback();

    // ===== 2. 读取麦轮数据 =====
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.current_wheel_rpm[i] =
            radps_to_rpm(_ctx.motor.mecanum[i]->get_current_rotate() *
                         dji_m3508_motor_drv_t::reciprocal_reduction_ratio);
        _ctx.data.current_wheel_torque[i] =
            _ctx.motor.mecanum[i]->get_current_torque();
        _ctx.data.current_wheel_temp[i] =
            _ctx.motor.mecanum[i]->get_temperature();
        _ctx.data.wheel_online[i] =
            _ctx.motor.mecanum[i]->is_online();
    }
    _ctx.data.current_wheel_rpm[1] = - _ctx.data.current_wheel_rpm[1];
    _ctx.data.current_wheel_rpm[3] = - _ctx.data.current_wheel_rpm[3];
    // ===== 3. 里程计反算实际速度 =====
    mecanum_kin_t::wheel_speeds_t current_speed{};
    current_speed.fl = rpm_to_mps(_ctx.data.current_wheel_rpm[0], WHEEL_RADIUS);
    current_speed.fr = rpm_to_mps(_ctx.data.current_wheel_rpm[1], WHEEL_RADIUS);
    current_speed.bl = rpm_to_mps(_ctx.data.current_wheel_rpm[2], WHEEL_RADIUS);
    current_speed.br = rpm_to_mps(_ctx.data.current_wheel_rpm[3], WHEEL_RADIUS);
    _kinematics->compute_odometry(current_speed,
                                  _ctx.data.real_vx,
                                  _ctx.data.real_vy,
                                  _ctx.data.real_wz);
    // ===== 4.读取摇臂反馈 =====
    for (int i = 0; i < 2; i++) {
        if(!_ctx.motor.lift[i])  continue;
        _ctx.data.lift_online[i] = _ctx.motor.lift[i]->is_online();
        _ctx.data.current_lift_angle[i] = _ctx.motor.lift[i]->get_current_position();
        _ctx.data.current_lift_speed[i] = _ctx.motor.lift[i]->get_current_rotate();        
    }
    //对原始反馈进行处理
    for(int i = 0;i < 2;i++){
        if(!_ctx.motor.lift[i]) continue;
        float raw_pos = _ctx.data.current_lift_angle[i];

        //===多圈展开===
        if(raw_pos - _ctx.data.lift_last_raw_pos[i] < -PI){
            _ctx.data.lift_cycle_counter[i]++;
        }//如果这个东西正转过零
        else if(raw_pos - _ctx.data.lift_last_raw_pos[i] > PI){
            _ctx.data.lift_cycle_counter[i]--;
        }
        _ctx.data.lift_last_raw_pos[i] = raw_pos;
        _ctx.data.lift_unwrap_angle[i] = raw_pos + 2.0f * PI * _ctx.data.lift_cycle_counter[i];

    }
    // ===== 5. 读取矿仓反馈 =====
    _ctx.data.magazine_online    = _ctx.motor.magazine->is_online();
    _ctx.data.current_magazine_angle     = _ctx.motor.magazine->get_current_position();
    _ctx.data.current_magazine_speed     = _ctx.motor.magazine->get_current_rotate();
    _ctx.data.magazine_full[0] = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)  == GPIO_PIN_RESET);
    _ctx.data.magazine_full[1] = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_RESET);
    _ctx.data.magazine_full[2] = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)  == GPIO_PIN_RESET);
    _ctx.data.magazine_full[3] = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)  == GPIO_PIN_RESET);

    _ctx.data.magazine_mask = 0;
    for(int i = 0;i < 4; i++){
        if(_ctx.data.magazine_full[i]){
            _ctx.data.magazine_mask |= (1<<i);//第几位置一就是第几个矿仓是有东西的
        }
    }

    // ===== 6. TODO ：裁判系统功率数据 =====
    // auto *referee = referee_drv_t::get_instance();
    // const auto &ref_data = referee->get_data();
    // _ctx.data.buffer_energy = ref_data.power_heat.buffer_energy;
    // _ctx.data.total_predicted_power =
        // power_controller_t::get_instance().get_total_predicted_power();
}

// =========================================================
// _kinematics_solve()：麦轮逆运动学
// =========================================================
void engineer_chassis_t::_kinematics_solve()
{
    const auto wheel_speeds = _kinematics->solve(
        _ctx.cmd->chassis.vx,
        _ctx.cmd->chassis.vy,
        _ctx.cmd->chassis.wz,
        mecanum_kin_t::missing_mec_e::NONE
    );
    // 线速度 m/s ->转速 RPM
    _ctx.data.target_wheel_rpm[0] = mps_to_rpm(wheel_speeds.fl,WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[1] = mps_to_rpm(wheel_speeds.fr,WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[2] = mps_to_rpm(wheel_speeds.bl,WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[3] = mps_to_rpm(wheel_speeds.br,WHEEL_RADIUS);
}

// =========================================================
// _mecanum_control()：麦轮速度环PID
// =========================================================
void engineer_chassis_t::_mecanum_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.out_wheel_torque[i] =
            _ctx.pid.mecanum_pid[i]->calculate(
                _ctx.data.target_wheel_rpm[i],
                _ctx.data.current_wheel_rpm[i]);
    }
    
}

// =========================================================
// _lift_control()：摇臂控制
// =========================================================

//============================================
//摇臂校准：触发
//============================================
void engineer_chassis_t::lift_start_calibrate(int i)
{
    if (i < 0 || i >= 2) return;
    if (!_ctx.motor.lift[i]) return;

    _ctx.data.lift_calib_state[i]      = lift_calib_state_t::CALIBRATING;
    _ctx.data.lift_calib_retry[i]      = 0;
    _ctx.data.lift_stall_timer[i]      = 0;
    _ctx.data.lift_calib_start_time[i] = xTaskGetTickCount();
    _ctx.data.lift_zero_valid[i]       = false;

    if (_ctx.pid.lift_vel_pid[i]) _ctx.pid.lift_vel_pid[i]->clear();
}

// =========================================================
// 摇臂校准：状态机每周期执行
// =========================================================
void engineer_chassis_t::_lift_calibrate_tick(int i)
{
    if (!_ctx.motor.lift[i] || !_ctx.pid.lift_vel_pid[i]) return;

    float raw_ang = _ctx.data.current_lift_angle[i];
    float raw_spd = fabs(_ctx.data.current_lift_speed[i]);
    uint32_t now  = xTaskGetTickCount();

    switch (_ctx.data.lift_calib_state[i])
    {
    case lift_calib_state_t::IDLE:
    case lift_calib_state_t::CALIB_DONE:
    case lift_calib_state_t::CALIB_FAILED:
       _ctx.data.out_lift_torque[i] = 0.0f;
        break;

    // ===== 恒速找零点 =====
    case lift_calib_state_t::CALIBRATING: {
        float target_speed = LIFT_CALIB_SPEED * LIFT_CALIB_DIR[i];
        float torque = _ctx.pid.lift_vel_pid[i]->calculate(
            target_speed,
            _ctx.data.current_lift_speed[i]
        );
        _ctx.data.out_lift_torque[i] = torque;

        // 堵转检测
        if (raw_spd < LIFT_CALIB_STALL_SPEED) {
            _ctx.data.lift_stall_timer[i]++;
            if (_ctx.data.lift_stall_timer[i] >= LIFT_CALIB_STALL_MS) {
                _ctx.data.lift_calib_state[i] = lift_calib_state_t::VERIFYING;
                _ctx.data.lift_stall_timer[i] = 0;
            }
        } else {
            _ctx.data.lift_stall_timer[i] = 0;
        }

        // 超时保护
        if (now - _ctx.data.lift_calib_start_time[i] > LIFT_CALIB_TIMEOUT) {
            _ctx.data.lift_calib_state[i] = lift_calib_state_t::CALIB_FAILED;
            _ctx.data.out_lift_torque[i] = 0.0f;
        }
        break;
    }

    // ===== 区间核验 =====
    case lift_calib_state_t::VERIFYING: {
        _ctx.data.out_lift_torque[i] = 0.0f;
        //先卸力

        if (raw_ang >= LIFT_ZERO_EXPECTED_MIN &&
            raw_ang <= LIFT_ZERO_EXPECTED_MAX)
        {
            // 核验通过，记录零点
            _ctx.data.lift_zero_offset[i]  = raw_ang;
            _ctx.data.lift_zero_valid[i]   = true;
            // 重置圈数，让相对角度从0开始
            _ctx.data.lift_cycle_counter[i] = 0;
            _ctx.data.lift_last_raw_pos[i]  = raw_ang;
            _ctx.data.lift_unwrap_angle[i]  = raw_ang;
            _ctx.data.target_lift_angle[i]  = 0.0f;

            _ctx.data.lift_calib_state[i] = lift_calib_state_t::CALIB_DONE;
            _ctx.data.lift_calib_retry[i] = 0;
        }
        else
        {
            // 核验失败，准备重试
            _ctx.data.lift_calib_retry[i]++;
            if (_ctx.data.lift_calib_retry[i] >= LIFT_CALIB_MAX_RETRY) {
                _ctx.data.lift_calib_state[i] = lift_calib_state_t::CALIB_FAILED;
            } else {
                _ctx.data.lift_backoff_target[i] = raw_ang + LIFT_BACKOFF_ANGLE * LIFT_CALIB_DIR[i];
                _ctx.data.lift_calib_state[i] = lift_calib_state_t::RETRY_BACKOFF;
                _ctx.pid.lift_vel_pid[i]->clear();
            }
        }
        break;
    }

    // ===== 失败回退 =====

    // ===== 失败回退 =====
    case lift_calib_state_t::RETRY_BACKOFF: {
        float backoff_speed = LIFT_BACKOFF_SPEED * LIFT_CALIB_DIR[i];
        float torque = _ctx.pid.lift_vel_pid[i]->calculate(
            backoff_speed,
            _ctx.data.current_lift_speed[i]
        );
        _ctx.data.out_lift_torque[i] = torque;

        // 退到位，重新校准
        bool backoff_done;
        if (LIFT_CALIB_DIR[i] > 0) {
            backoff_done = (raw_ang >= _ctx.data.lift_backoff_target[i]);
        } else {
            backoff_done = (raw_ang <= _ctx.data.lift_backoff_target[i]);
        }
        if (backoff_done) {
            _ctx.data.lift_calib_state[i] = lift_calib_state_t::CALIBRATING;
            _ctx.data.lift_stall_timer[i] = 0;
            _ctx.data.lift_calib_start_time[i] = now;
            _ctx.pid.lift_vel_pid[i]->clear();
        }


        break;
    }
    }
}
void engineer_chassis_t::_lift_control()
{
  for (int i = 0; i < 2; i++) {
        // 判空，没创建就跳过
        if (!_ctx.motor.lift[i] || !_ctx.pid.lift_pos_pid[i] || !_ctx.pid.lift_vel_pid[i]) continue;
        //====校准中====
        if (_ctx.data.lift_calib_state[i] != lift_calib_state_t::IDLE &&
            _ctx.data.lift_calib_state[i] != lift_calib_state_t::CALIB_DONE)
        {
            _lift_calibrate_tick(i);   // 校准状态机自己处理发扭矩
            continue;                  // 不走下面的位置环
        }
        if(!_ctx.data.lift_zero_valid[i]){
            _ctx.data.out_lift_torque[i] = 0.0f;
            continue;
        }
        //1.根据命令算目标角度
        if (_ctx.cmd->lift.mode == lift_mode_t::MANUAL) {
            // 手动模式：增量式，每周期加一点
            float increment = 0.0f;
            if (i == 0) { // 左
                if (_ctx.cmd->lift.manual.left_mod == lift_manual_mod_t::UP)   increment =  0.002f;
                if (_ctx.cmd->lift.manual.left_mod == lift_manual_mod_t::DOWN) increment = -0.002f;
            } else { // 右
                if (_ctx.cmd->lift.manual.right_mod == lift_manual_mod_t::UP)   increment =  0.002f;
                if (_ctx.cmd->lift.manual.right_mod == lift_manual_mod_t::DOWN) increment = -0.002f;
            }
            _ctx.data.target_lift_angle[i] += increment;
            //===软件限位===
            if (_ctx.data.target_lift_angle[i] > _ctx.data.lift_max_angle)
            _ctx.data.target_lift_angle[i] = _ctx.data.lift_max_angle;
            if (_ctx.data.target_lift_angle[i] < _ctx.data.lift_min_angle)
            _ctx.data.target_lift_angle[i] = _ctx.data.lift_min_angle;
        } else {
            // 自动模式：DEPLOY 放下（目标0），RETRACT 收起（目标6）
            if (_ctx.cmd->lift.auto_action == lift_action_t::DEPLOY) {
                _ctx.data.target_lift_angle[i] = 0.0f;
            } else if (_ctx.cmd->lift.auto_action == lift_action_t::RETRACT) {
                _ctx.data.target_lift_angle[i] = 6.0f;
            }
            // HOLD 就保持当前目标不变
        }
        //软件限位
        if (_ctx.data.target_lift_angle[i] > _ctx.data.lift_max_angle)
            _ctx.data.target_lift_angle[i] = _ctx.data.lift_max_angle;
        if (_ctx.data.target_lift_angle[i] < _ctx.data.lift_min_angle)
            _ctx.data.target_lift_angle[i] = _ctx.data.lift_min_angle;
        // 2. 计算相对零点的反馈角度
        float relative_angle = (_ctx.data.lift_unwrap_angle[i] - _ctx.data.lift_zero_offset[i]) * LIFT_CALIB_DIR[i];

        
        // 3. 位置环 → 目标速度
        float target_vel = _ctx.pid.lift_pos_pid[i]->calculate(
            _ctx.data.target_lift_angle[i],
            relative_angle
        );

        // 4. 速度环 → 输出扭矩
        float feedback_speed = _ctx.data.current_lift_speed[i] * LIFT_CALIB_DIR[i];
        _ctx.data.out_lift_torque[i] = _ctx.pid.lift_vel_pid[i]->calculate(
            target_vel,
            feedback_speed
        );
        // 5. 扭矩输出乘方向，统一电机方向
        _ctx.data.out_lift_torque[i] *= LIFT_CALIB_DIR[i];
    }
    // 然后位置环PID → 速度环PID → 输出扭矩
}
// =========================================================
// _magazine_control()：矿仓控制
// =========================================================
void engineer_chassis_t::_magazine_control()
{
    // 1. 档位转换
    switch(_ctx.cmd->magazine.target_pos){
        case magazine_pos_t::POS_1: _ctx.data.target_magazine_angle = 0.0f;       break;
        case magazine_pos_t::POS_2: _ctx.data.target_magazine_angle = 1.5708f;    break;  // π/2
        case magazine_pos_t::POS_3: _ctx.data.target_magazine_angle = 3.1416f;    break;  // π
        case magazine_pos_t::POS_4: _ctx.data.target_magazine_angle = 4.7124f;    break;  // 3π/2
        default: break;
    }
    //2. 位置环
    float error = _ctx.data.target_magazine_angle - _ctx.data.current_magazine_angle;
    if(error > PI){
        error -= 2*PI;
    }
    if(error < -PI){
        error += 2*PI;
    }
    float virtual_target = _ctx.data.current_magazine_angle + error;

    _ctx.data.target_magazine_speed = _ctx.pid.magazine_pos_pid->calculate(
        virtual_target,
        _ctx.data.current_magazine_angle
    );
    //3. 速度环
    _ctx.data.out_magazine_torque = _ctx.pid.magazine_vel_pid->calculate(
        _ctx.data.target_magazine_speed,
        _ctx.data.current_magazine_speed);
    static uint8_t stable_cnt = 0;
    if(fabs(error) < 0.1f){
        if(stable_cnt < 20) stable_cnt++;
        _ctx.data.magazine_ready = (stable_cnt >= 20);
    }
    else{
        stable_cnt = 0;
        _ctx.data.magazine_ready = false;
    }

}

// =========================================================
// _power_control()：功率限制
// =========================================================
void engineer_chassis_t::_power_control()
{
    // 1. 把当前扭矩、转速、温度填入功率节点
    for (int i = 0; i < 4; ++i) {
        if (_ctx.power_motor_data[i] == nullptr) continue;
        _ctx.power_motor_data[i]->target_cmd   = _ctx.data.out_wheel_torque[i];
        _ctx.power_motor_data[i]->uncontrolled_cmd = 0.0f;
        _ctx.power_motor_data[i]->rpm  = _ctx.data.current_wheel_rpm[i];
        _ctx.power_motor_data[i]->temp = _ctx.data.current_wheel_temp[i];
    }

    // 2. 调用功率控制器求解
    //    无裁判系统时用固定功率限制80W、缓冲能量60J
    //    后续接入裁判系统后，应从 referee 模块读取真实值
    float referee_power_limit  = 80.0f;
    float current_buffer_energy = 60.0f;
    power_controller_t::get_instance().solve(
        referee_power_limit, current_buffer_energy);

    // 3. 把限制后的安全扭矩写回，覆盖 PID 输出
    for (int i = 0; i < 4; ++i) {
        if (_ctx.power_motor_data[i] == nullptr) continue;
        _ctx.data.out_wheel_torque[i] = _ctx.power_motor_data[i]->safe_cmd;
    }

    // 4. 记录总预测功率（便于调试观察）
    _ctx.data.total_predicted_power =
        power_controller_t::get_instance().get_total_predicted_power();
}

// =========================================================
// _send_motor_command()：发送所有电机指令
// =========================================================
void engineer_chassis_t::_send_motor_command() const
{
    // 麦轮
    _ctx.motor.mecanum[0]->send_torque(_ctx.data.out_wheel_torque[0]);
    
    _ctx.motor.mecanum[1]->send_torque(-_ctx.data.out_wheel_torque[1]);
    
    _ctx.motor.mecanum[2]->send_torque(_ctx.data.out_wheel_torque[2]);
    
    _ctx.motor.mecanum[3]->send_torque(-_ctx.data.out_wheel_torque[3]);
    
    //矿仓

    _ctx.motor.magazine->send_torque(_ctx.data.out_magazine_torque);


    for (int i = 0; i < 2; i++) {
        if(_ctx.motor.lift[i]){
            _ctx.motor.lift[i]->send_torque(_ctx.data.out_lift_torque[i]);
        }
    }

  
}

// =========================================================
// _fsm_execute()：状态机执行
// =========================================================
void engineer_chassis_t::_fsm_execute()
{
    // 1. 把当前命令指针存到 ctx
    _ctx.cmd = &_current_cmd;  // _current_cmd 也是基类的 protected 成员

    // 2. 根据 mode 切换顶层状态
    if (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
    {
        _main_fsm.change_state(&_state_active);
    }
    else
    {
        _main_fsm.change_state(&_state_passive);
    }

    // 3. 执行当前状态
    _main_fsm.execute(this);
}

} // namespace pyro
