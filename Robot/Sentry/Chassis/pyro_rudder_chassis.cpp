#include "pyro_rudder_chassis.h"
#include "pyro_algo_common.h"
#include <arm_math.h> // 引入 CMSIS-DSP 库
#include "pyro_dji_motor_drv.h"
#include "pyro_can_drv.h"

#include "pyro_power_control.h"
#include "pyro_board_comm.h"


float test_imutarget;
float a1=0;
float a2=0;
namespace pyro
{
rudder_chassis_t::rudder_chassis_t() : module_base_t("rudder")
{
    _ctx = {};
}




status_t rudder_chassis_t::_init()
{
    _ctx.motor  = _module_deps.motor_deps;//指针
    _ctx.pid    = _module_deps.pid_deps;


    _ctx.data.rudder_offset_rad[0] = FL_OFFSET_RAD;
    _ctx.data.rudder_offset_rad[1] = FR_OFFSET_RAD;  
    _ctx.data.rudder_offset_rad[2] = BL_OFFSET_RAD;
    _ctx.data.rudder_offset_rad[3] = BR_OFFSET_RAD;

    // 使用 config.h 中的参数初始化运动学模型S

    
    _kinematics = new rudder_kin_t(
                                   RUDDER_WHEELBASE,
                                   RUDDER_FRONTBASE);
    // float x = 0.15f;
    // _kinematics = new hybrid_kin_t(TRACK_SPACING,
    //                            0.15f + x,0.15f + x,0.66f - x, 0.66f - x);

    return PYRO_OK;
}

void rudder_chassis_t::_power_control_init()
{
    power_fit_params_t coef[4];

    for (auto &[k1, k2, k3, k4, k5 , alpha] : coef)
    {
        // k1 = 0.0160f;//0.0155//0.0260
        // k2 = 0.0250f;//1.6000//0.0460
        // k3 = 0.1742f;//0.0010//0.1066
        // k4 = 0.5815f;//0.7500//0.7500
        k1 = 0.0260f; // 0.0155
        k2 = 0.0460f; // 1.6000
        k3 = 0.1100f; // 0.0010
        k4 = 0.7500f; // 0.7500
        k5= 0.0f;
        alpha = 0.0f;
    }

    power_controller_t::get_instance().register_motor(coef[0]);
    power_controller_t::get_instance().register_motor(coef[1]);
    power_controller_t::get_instance().register_motor(coef[2]);
    power_controller_t::get_instance().register_motor(coef[3]);
}

void rudder_chassis_t::_update_feedback(){

        // 1. 更新所有电机反馈
    for (const auto &i : _ctx.motor.rudder)
        i->update_feedback();
    for (const auto &i : _ctx.motor.wheel)
        i->update_feedback();

    _ctx.motor.yaw->update_feedback();

    // // 2. 读取 IMU 数据作为底盘姿态反馈
    // ins_drv_t::get_instance()->get_rads_n(&_ctx.data.current_yaw_rad,
    //                                       &_ctx.data.current_pitch_rad,
    //                                       &_ctx.data.current_roll_rad);
    // _ctx.data.current_pitch_rad -= PITCH_OFFSET_RAD;
    // _ctx.data.current_roll_rad -= ROLL_OFFSET_RAD;
    // 2. 读取 IMU 数据作为底盘姿态反馈
    // float raw_yaw, raw_pitch, raw_roll;
    // ins_drv_t::get_instance()->get_rads_n(&raw_yaw, &raw_pitch, &raw_roll);


    // raw_pitch -= PITCH_OFFSET_RAD;
    // raw_roll  -= ROLL_OFFSET_RAD;


    // // --- 一阶低通滤波 (LPF) ---
    // // 为了快速验证，这里使用 static 变量保存上一次的滤波状态
    // // 如果确认有效，建议将它们移到 _ctx.data 结构体中
    // static float filtered_pitch = 0.0f;
    // static float filtered_roll  = 0.0f;
    // static float filtered_yaw   = 0.0f;
    // static bool  is_first_run   = true;

    // // 滤波系数 alpha：(0, 1]
    // // alpha = 1.0 表示完全不滤波；alpha 越小，抗噪声能力越强，但相位延迟越大。
    // // 对于 500Hz~1000Hz 的控制循环，0.1f ~ 0.3f 通常是一个比较理想的甜点值。
    // const float LPF_ALPHA = 0.15f;

    // if (is_first_run)
    // {
    //     filtered_pitch = raw_pitch;
    //     filtered_roll  = raw_roll;
    //     filtered_yaw   = raw_yaw;
    //     is_first_run   = false;
    // }
    // else
    // {
    //     low_pass_filter(raw_pitch, &filtered_pitch, LPF_ALPHA);
    //     low_pass_filter(raw_roll,  &filtered_roll,  LPF_ALPHA);
    //     low_pass_filter(raw_yaw,   &filtered_yaw,   LPF_ALPHA);
    // }
    // _ctx.imu_data.current_pitch_rad = filtered_pitch;
    // _ctx.imu_data.current_roll_rad  = filtered_roll;    
    // _ctx.imu_data.current_yaw_rad = filtered_yaw;
    
    
    //---暂不滤波---
    // _ctx.imu_data.current_pitch_rad = raw_pitch;
    // _ctx.imu_data.current_roll_rad  = raw_roll;    
    // _ctx.imu_data.current_yaw_rad = raw_yaw;

    _ctx.data.current_data.yaw_error = loop_fp32_constrain(
                                            _ctx.motor.yaw->get_current_position()
                                        -   YAW_OFFSET_RAD,
                                            -PI, PI);
    _ctx.data.current_data.yaw_spd = _ctx.motor.yaw->get_current_rotate();
    _ctx.data.yaw_online = _ctx.motor.yaw->is_online();
    // 3. 提取舵电机反馈数据（位置、速度、扭矩）
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.current_data.rudder_pos[i]    = loop_fp32_constrain(
                                                _ctx.motor.rudder[i]->get_current_position() 
                                            -   _ctx.data.rudder_offset_rad[i],
                                                -PI, PI);

        _ctx.data.current_data.rudder_spd[i]    = _ctx.motor.rudder[i]->get_current_rotate();   
        _ctx.data.current_data.rudder_torque[i] = _ctx.motor.rudder[i]->get_current_torque();
    }

    // 4. 提取轮电机反馈数据（位置、速度、扭矩）
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.current_data.wheel_pos[i]    = _ctx.motor.wheel[i]->get_current_position();
        _ctx.data.current_data.wheel_spd[i]    = _ctx.motor.wheel[i]->get_current_rotate();
        //单位 rad/m
        _ctx.data.current_data.wheel_torque[i] = _ctx.motor.wheel[i]->get_current_torque();
    }
    //5. 将当前舵轮状态转换为运动学状态
        for (int i = 0; i < 4; i++)
    {
        _ctx.data.current_states.modules[i].angle = _ctx.data.current_data.rudder_pos[i];
        _ctx.data.current_states.modules[i].speed = _ctx.data.current_data.wheel_spd[i]
                                            * dji_m3508_motor_drv_t::reciprocal_reduction_ratio_xroll
                                            * WHEEL_SIZE;  
    }



//6. 读取IMU数据
    // static float filtered_yaw_radps   = 0.0f;
    // static float filtered_yaw_rad     = 0.0f;
    
    // static float filtered_yaw_cos     = 0.0f;
    // static float filtered_yaw_sin     = 0.0f;

    // static bool  is_first_run   = true;

    // 滤波系数 alpha：(0, 1]
    // alpha = 1.0 表示完全不滤波；alpha 越小，抗噪声能力越强，但相位延迟越大。
    // 对于 500Hz~1000Hz 的控制循环，0.1f ~ 0.3f 通常是一个比较理想的甜点值。
    // const float LPF_ALPHA = 0.15f;

    // if (is_first_run)
    // {

    //     filtered_yaw_radps   = _ctx.cmd->imu_yaw_radps;
    //     filtered_yaw_cos      = cosf(_ctx.cmd->imu_yaw_rad);
    //     filtered_yaw_sin      = sinf(_ctx.cmd->imu_yaw_rad);
    //     is_first_run   = false;
    // }
    // else
    // {
    //     low_pass_filter(_ctx.cmd->imu_yaw_radps, &filtered_yaw_radps, LPF_ALPHA);
    //     low_pass_filter(cosf(_ctx.cmd->imu_yaw_rad), &filtered_yaw_cos, LPF_ALPHA);
    //     low_pass_filter(sinf(_ctx.cmd->imu_yaw_rad), &filtered_yaw_sin, LPF_ALPHA);
        
    // }
    
    // _ctx.imu_data.current_yaw_radps = filtered_yaw_radps;
    _ctx.data.imu_data.current_yaw_radps = _ctx.cmd->imu_yaw_radps;
    _ctx.data.imu_data.current_yaw_rad = _ctx.cmd->imu_yaw_rad;
    //_ctx.imu_data.current_yaw_rad = atan2f(filtered_yaw_sin, filtered_yaw_cos);

    
}
void rudder_chassis_t::_kinematics_solve(){

    float follow_wz=0;
    float final_wz=0;
    if(_ctx.cmd->follow_yaw == true)
    {
        _ctx.data.target_data.yaw_error = 0.0f;
        float _delta_yaw =    loop_fp32_constrain(
                            _ctx.data.target_data.yaw_error - 
                            _ctx.data.current_data.yaw_error,
                            -PI, PI);
        // if(abs(_delta_yaw) < 0.02f)
        // {    
        //         _delta_yaw = 0.0f;
        // }
            
        

        follow_wz =- _ctx.pid.follow_yaw_pid
                            ->calculate( _delta_yaw,
                                         0.0f);
        final_wz = follow_wz;
                if(abs(_delta_yaw) < 0.02f)
        {    
                final_wz = 0.0f;
        }
    }
    //覆盖————可能有bug
    else if(_ctx.cmd->spinning == true)
    { 
        final_wz = 2.5f;
    }
    else
    {     
        follow_wz = 0.0f;
        final_wz = follow_wz ;//+_ctx.cmd->wz;
    }//follow关闭时，底盘不动

    

    const float theta   = _ctx.data.current_data.yaw_error -0.09f*final_wz;
    // const float theta   = 0;

    const float c_theta = arm_cos_f32(theta);
    const float s_theta = arm_sin_f32(theta);

    // 旋转矩阵公式 (逆时针旋转 theta)
    float vx_chassis    = _ctx.cmd->vx * c_theta + _ctx.cmd->vy * s_theta;
    float vy_chassis    = -_ctx.cmd->vx * s_theta + _ctx.cmd->vy * c_theta;  
    
    // ==================== 速度变化率限制 (防翻车) ====================
    // // 限制遥控器指令的每帧变化量，矢量差 delta_v 本身已反映变化幅度，
    // // 前进→倒车自然比前进→停车更大，无需额外按方向角加权。
    // static float last_vx      = 0.0f;
    // static float last_vy      = 0.0f;
    // static bool  is_first_time = true;
    // if (is_first_time)
    // {
    //     last_vx      = vx_chassis;
    //     last_vy      = vy_chassis;
    //     is_first_time = false;
    // }

    // float delta_vx = vx_chassis - last_vx;
    // float delta_vy = vy_chassis - last_vy;
    // float delta_v  = hypotf(delta_vx, delta_vy);

    // if (delta_v > ACCEL_LIMIT_PER_FRAME)
    // {
    //     float angle = atan2f(delta_vy, delta_vx);
    //     delta_v = ACCEL_LIMIT_PER_FRAME;
    //     vx_chassis = last_vx + delta_v * cosf(angle);
    //     vy_chassis = last_vy + delta_v * sinf(angle);
    // }
    

    _ctx.data.target_states = _kinematics->solve(vx_chassis, vy_chassis, final_wz,_ctx.data.current_states);
                            
    // if(_ctx.cmd->follow_yaw){
    //     if((current_v <CONTROL_V) && (current_v > 0.05F) && hypotf(vx_chassis,vy_chassis) < 0.01f){
    //         _ctx.target_states.modules[0].angle = PI/2;
    //         _ctx.target_states.modules[1].angle = PI/2;
    //         _ctx.target_states.modules[2].angle = 0;
    //         _ctx.target_states.modules[3].angle = 0;
    //     }
    // }


    // last_vx = vx_chassis;
    // last_vy = vy_chassis;

    _ctx.data.imu_data.target_yaw_rad += _ctx.cmd->delta_yaw ;
    _ctx.data.imu_data.target_yaw_rad = loop_fp32_constrain(_ctx.data.imu_data.target_yaw_rad, -PI, PI);

    
    
    // //state转data，正反转在solve中,直接使用state可注释
    // _ctx.target_data.rudder_pos[0] = _ctx.target_states.modules[rudder_kin_t::FL].angle;
    // _ctx.target_data.rudder_pos[1] = _ctx.target_states.modules[rudder_kin_t::FR].angle;
    // _ctx.target_data.rudder_pos[2] = _ctx.target_states.modules[rudder_kin_t::BL].angle;
    // _ctx.target_data.rudder_pos[3] = _ctx.target_states.modules[rudder_kin_t::BR].angle;

    // _ctx.target_data.wheel_spd[0] = _ctx.target_states.modules[rudder_kin_t::FL].speed;
    // _ctx.target_data.wheel_spd[1] = _ctx.target_states.modules[rudder_kin_t::FR].speed;
    // _ctx.target_data.wheel_spd[2] = _ctx.target_states.modules[rudder_kin_t::BL].speed;
    // _ctx.target_data.wheel_spd[3] = _ctx.target_states.modules[rudder_kin_t::BR].speed;

}
// void rudder_chassis_t::_power_control()
// {
// //舵电机优先
// float uncontrol_energy = 0.0f;
//     for (int i = 0; i < 4; i++)
//     {
//         _ctx.data.power_rmotor_data[i].rpm                  = _ctx.data.current_data.rudder_spd[i];
//         _ctx.data.power_rmotor_data[i].last_controlled_cmd  = _ctx.data.current_data.rudder_torque[i];
//         _ctx.data.power_rmotor_data[i].target_cmd           = _ctx.data.target_data.rudder_torque[i];
//         _ctx.data.power_rmotor_data[i].uncontrolled_cmd=
//         _ctx.data.power_rmotor_data[i].power_predict =
//             power_controller_t::get_instance().get_total_predicted_power(
//                 i + 4, _ctx.data.power_rmotor_data[i].torque_cmd,
//                 _ctx.data.power_rmotor_data[i].gyro);

//                 // 不控制
//                 uncontrol_energy += _ctx.data.ower_rmotor_data[i].power_predict;
//     }





//     for (int i = 0; i < 4; i++)
//     {
//         _ctx.power_wmotor_data[i].gyro       = _ctx.target_data.wheel_spd[i];
//         _ctx.power_wmotor_data[i].torque_cmd = _ctx.current_data.wheel_torque[i];
//         _ctx.power_wmotor_data[i].power_predict =
//             power_controller_t::get_instance().motor_power_predict(
//                 i, _ctx.power_wmotor_data[i].torque_cmd,
//                 _ctx.power_wmotor_data[i].gyro);
//     }
//     // if (_ctx.cap_feedback.vot_cap >= 1800)
//     // {
//     //     power_controller_t::get_instance().calculate_restricted_torques(
//     //         _ctx.power_motor_data, 4,
//     //         static_cast<float>(referee_drv_t::get_instance()
//     //                                ->get_data()
//     //                                .robot_status.chassis_power_limit) +
//     //             100.0f);
//     // }
//     // else
//     // {
//     power_controller_t::get_instance().calculate_restricted_torques(
//         _ctx.power_wmotor_data , 4 , 240 - uncontrol_energy , 60);
//     // }
//     for (int i = 0; i < 4; i++)
//         _ctx.current_data.wheel_torque[i] =
//             _ctx.power_wmotor_data[i].restricted_torque;
// }
void rudder_chassis_t::_rudder_control(){

        
        for (int i = 0; i < 4; i++)
        {   _ctx.data.target_data.wheel_torque[i] = 
                            _ctx.pid.wheel_pid[i]->calculate(   
                                _ctx.data.target_states.modules[i].speed,
                                _ctx.data.current_states.modules[i].speed);
        } 
        for (int i = 0; i < 4; i++)
        {   
            float error_angle = loop_fp32_constrain(
                            _ctx.data.target_states.modules[i].angle - 
                            _ctx.data.current_states.modules[i].angle,
                            -PI, PI);
            const float rud_pos_out = 
                            _ctx.pid.rudder_pos_pid[i]->calculate(   
                                error_angle,
                                0);
                            
                        _ctx.data.target_data.rudder_torque[i] = 
                            _ctx.pid.rudder_spd_pid[i]->calculate(   
                                rud_pos_out,
                                _ctx.data.current_data.rudder_spd[i]);
        } 



}

void rudder_chassis_t::_yaw_control(){

            float imu_error_angle = loop_fp32_constrain(
                            _ctx.data.imu_data.target_yaw_rad - 
                            _ctx.data.imu_data.current_yaw_rad,
                            -PI, PI);
            if(abs(imu_error_angle)<0.03f)
            {
                //if(abs(imu_error_angle)<0.003f){
                //    imu_error_angle *= 0.0f;
                //}
                //imu_error_angle *=0.9f;
            }
            const float rud_pos_out = 
                            _ctx.pid.yaw_pos_pid->calculate(   
                                imu_error_angle,
                                0);
            // float test_rud_pos_out = _ctx.cmd->delta_yaw*1000;

            // // if(_ctx.cmd->delta_yaw>0){test_rud_pos_out = -2.0f;}
            
            // _ctx.target_data.yaw_spd = -test_rud_pos_out;
                        _ctx.data.target_data.yaw_torque = 
                            _ctx.pid.yaw_spd_pid->calculate(   
                                -rud_pos_out,
                                _ctx.data.imu_data.current_yaw_radps/*_ctx.current_data.yaw_spd*/);
            a1=_ctx.data.imu_data.target_yaw_rad;

        //_ctx.out_data.yaw_torque = 0.0f;
}
void rudder_chassis_t::_send_motor_command() const
{

        for (int i = 0; i < 4; i++)
    {
        _ctx.motor.rudder[i]->send_torque(_ctx.data.target_data.rudder_torque[i]);
        _ctx.motor.wheel[i]->send_torque(_ctx.data.target_data.wheel_torque[i]);
    }
    _ctx.motor.yaw->send_torque(_ctx.data.target_data.yaw_torque);

};

void rudder_chassis_t::_communicate_gimbal() const
{
    c2g_msg_t msg{};
    msg.data[0] = _ctx.data.bus_tx_data[0];
    msg.data[1] = _ctx.data.bus_tx_data[1];
    msg.data[2] = _ctx.data.bus_tx_data[2];
    msg.data[3] = _ctx.data.bus_tx_data[3];

    pyro::board_comm_t::instance().send(msg);
}

void rudder_chassis_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;
    if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);
    else if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}


}