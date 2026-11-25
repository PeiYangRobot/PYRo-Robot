#include "pyro_wl_gimbal.h"
#include "gimbal_config.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"
#include "dsp/fast_math_functions.h"


namespace pyro
{

wl_gimbal_t::wl_gimbal_t() : module_base_t("wl_gimbal")
{
}

status_t wl_gimbal_t::_init()
{
    _ctx = {};

    return PYRO_OK;
}

void wl_gimbal_t::_update_feedback()
{
    // 读取 IMU 数据作为云台姿态反馈
    ins_drv_t::get_instance()->get_rads_n(&_ctx.data.imu.yaw,
                                          &_ctx.data.imu.pitch,
                                          &_ctx.data.imu.roll);

    ins_drv_t::get_instance()->get_gyro_b(&_ctx.data.imu.gyro[0],
                                          &_ctx.data.imu.gyro[1],
                                          &_ctx.data.imu.gyro[2]);

    ins_drv_t::get_instance()->get_accel_b(&_ctx.data.imu.accel[0],
                                           &_ctx.data.imu.accel[1],
                                           &_ctx.data.imu.accel[2]);

    _ctx.data.imu.timestamp = xTaskGetTickCount();

    //更新电机反馈
    _module_deps.motor_deps.pitch->update_feedback();
    _module_deps.motor_deps.yaw->update_feedback();


    _ctx.data.state.pitch.online = _module_deps.motor_deps.pitch->is_online();
    _ctx.data.state.pitch.pos    = _module_deps.motor_deps.pitch->get_current_position();
    _ctx.data.state.pitch.vel    = _module_deps.motor_deps.pitch->get_current_rotate();
    _ctx.data.state.pitch.temp   = _module_deps.motor_deps.pitch->get_temperature();
    _ctx.data.state.pitch.torque = _module_deps.motor_deps.pitch->get_current_torque();


    _ctx.data.state.yaw.online   = _module_deps.motor_deps.yaw->is_online();
    _ctx.data.state.yaw.pos      = _module_deps.motor_deps.yaw->get_current_position();
    _ctx.data.state.yaw.vel      = _module_deps.motor_deps.yaw->get_current_rotate();
    _ctx.data.state.yaw.temp     = _module_deps.motor_deps.yaw->get_temperature();
    _ctx.data.state.yaw.torque   = _module_deps.motor_deps.yaw->get_current_torque();


    _ctx.data.telem.target_yaw_vel     = _current_cmd.yawVel;
    _ctx.data.telem.target_pitch_vel   = _current_cmd.pitchVel;
    _ctx.data.motionState              = _current_cmd.state_cmd;
    _ctx.data.mode                     = _current_cmd.mode;
    _ctx.data.chassis_is_ready         = _current_cmd.chassis_is_ready;
    _ctx.data.chassis_is_rescuing      = _current_cmd.chassis_is_rescuing;

    static uint32_t dwtCnt;
    _ctx.data.dt = pyro::dwt_drv_t::get_delta_t(&dwtCnt);

}



void wl_gimbal_t::updatePitch()
{
    // 在线检测
    if (!_module_deps.motor_deps.pitch->is_online())
    {
        _ctx.data.output.pitchTorque            = 0.0f;
        _ctx.data.output.pitchEn                = false;
        return;
    }

    //计算imu角度和电机角度的差值，以此作为电机角度的偏移值
    //现在offset相当于是p轴比imu多出来的量
    float offsetPitch = _ctx.data.state.pitch.pos - _ctx.data.imu.pitch;

    //这个得到的是电机的p轴角度
    float targetMotorRaw = _ctx.data.telem.targetPitchRad + offsetPitch;


    if(targetMotorRaw <= PITCH_LIMIT_MAX)
    {
        targetMotorRaw = PITCH_LIMIT_MAX;
    }
    else if(targetMotorRaw >= PITCH_LIMIT_MIN)
    {
        targetMotorRaw = PITCH_LIMIT_MIN;
    }
    _ctx.data.telem.targetPitchRad = targetMotorRaw - offsetPitch;


    float pos_error = _ctx.data.state.pitch.pos - targetMotorRaw;
    pos_error = wrapAngle(pos_error);
    float tgt_spd = _module_deps.pid_deps.pitch_pos->calculate(0.0f, pos_error);

    //速度环
    static constexpr float omega_0 = 30.0f;
    static constexpr float b0    = 11.0f;
    static constexpr float beta1 = 2 * omega_0;
    static constexpr float beta2 = omega_0 * omega_0;
    static float rot_hat         = 0.0f;
    static float disrupt_hat     = 0.0f;

    //计算估算误差
    float spd_error = rot_hat - _ctx.data.imu.gyro[1];
    //更新观测器状态
    rot_hat = rot_hat + _ctx.data.dt * (disrupt_hat - beta1 * spd_error + b0 * _ctx.data.output.pitchTorque);
    disrupt_hat = disrupt_hat + _ctx.data.dt * (-beta2 * spd_error);

    float pitch_torque = _module_deps.pid_deps.pitch_spd->calculate(tgt_spd, _ctx.data.imu.gyro[1]);

    //云台俯仰轴的重力补偿和位置控制
    float gravityFf           = PITCH_K_GRAVITY_COS * arm_cos_f32(_ctx.data.imu.pitch) + PITCH_K_GRAVITY_SIN * arm_sin_f32(_ctx.data.imu.pitch);

    _ctx.data.output.pitchTorque            = gravityFf + pitch_torque - (disrupt_hat) / b0 * 0.2f;
    _ctx.data.output.pitchEn                = true;
}



void wl_gimbal_t::updateYaw()
{
    /*1. 在线检测 */
    if (!_ctx.data.state.yaw.online) {
        _ctx.data.telem.targetYawRad = _ctx.data.imu.yaw;
        _module_deps.pid_deps.yaw_pos->clear();
        _module_deps.pid_deps.yaw_spd->clear();
        _ctx.data.output.yawCurrent = 0.0f;
        return;
    }


    //控制部分


    //角度环
    float error_rad = wrapAngle(_ctx.data.imu.yaw - _ctx.data.telem.targetYawRad);
    float tgt_yaw_spd       = _module_deps.pid_deps.yaw_pos->calculate(0.0f, error_rad);
    //速度环
    static constexpr float omega_0 = 30.0f;
    static constexpr float b0    = 30.122f;
    static constexpr float beta1 = 2 * omega_0;
    static constexpr float beta2 = omega_0 * omega_0;
    static float rot_hat         = 0.0f;
    static float disrupt_hat     = 0.0f;
    //计算估算误差
    float error = rot_hat - _ctx.data.imu.gyro[0];
    //更新观测器状态

    rot_hat = rot_hat + _ctx.data.dt * (disrupt_hat - beta1 * error + b0 * _ctx.data.output.yawCurrent);
    disrupt_hat = disrupt_hat + _ctx.data.dt * (-beta2 * error);

    float yawSpdOut       = _module_deps.pid_deps.yaw_spd->calculate(tgt_yaw_spd, _ctx.data.imu.gyro[0]);
    //最终输出
    _ctx.data.output.yawCurrent = yawSpdOut - (disrupt_hat) / b0 * 0.1f;


}

void wl_gimbal_t::align_updatePitch()
{
    // 在线检测
    if (!_module_deps.motor_deps.pitch->is_online())
    {
        _ctx.data.output.pitchTorque            = 0.0f;
        _ctx.data.output.pitchEn                = false;
        return;
    }

    float pos_error = _ctx.data.state.pitch.pos - PITCH_ALIGN_TARGET_RAD;
    pos_error = wrapAngle(pos_error);
    float tgt_spd = _module_deps.pid_deps.pitch_pos->calculate(0.0f, pos_error);

    float spd_error = _ctx.data.imu.gyro[1] - tgt_spd;
    float pitch_torque = _module_deps.pid_deps.pitch_spd->calculate(0.0f, spd_error);
    //云台俯仰轴的重力补偿和位置控制
    float gravityFf = PITCH_K_GRAVITY_COS * arm_cos_f32(_ctx.data.imu.pitch) + PITCH_K_GRAVITY_SIN * arm_sin_f32(_ctx.data.imu.pitch);


    _ctx.data.output.pitchTorque = gravityFf + pitch_torque;
    _ctx.data.output.pitchEn                = true;
}
void wl_gimbal_t::align_updateYaw(float target_motor_yaw)
{
    /*1. 在线检测 */
    if (!_ctx.data.state.yaw.online) {
        _ctx.data.telem.targetYawRad = _ctx.data.imu.yaw;
        _module_deps.pid_deps.yaw_pos->clear();
        _module_deps.pid_deps.yaw_spd->clear();
        _ctx.data.output.yawCurrent = 0.0f;
        return;
    }

    //角度归一化
    float error_rad = wrapAngle((_ctx.data.state.yaw.pos - YAW_ALIGN_TARGET_RAD) + target_motor_yaw);
    //死区判断
    if(fabs(error_rad) <= 0.02f)
    {
        _ctx.data.output.yawCurrent = 0.0f;
        return;
    }
    float tgt_yaw_spd       = _module_deps.pid_deps.yaw_pos->calculate(0.0f, error_rad);
    //速度环
    static constexpr float omega_0 = 30.0f;
    static constexpr float b0    = 30.122f;
    static constexpr float beta1 = 2 * omega_0;
    static constexpr float beta2 = omega_0 * omega_0;
    static float rot_hat         = 0.0f;
    static float disrupt_hat     = 0.0f;
    //计算估算误差
    float error = rot_hat - _ctx.data.imu.gyro[0];
    //更新观测器状态

    rot_hat = rot_hat + _ctx.data.dt * (disrupt_hat - beta1 * error + b0 * _ctx.data.output.yawCurrent);
    disrupt_hat = disrupt_hat + _ctx.data.dt * (-beta2 * error);

    if(tgt_yaw_spd >= 6.0f)
    {
        tgt_yaw_spd = 6.0f;
    }
    if(tgt_yaw_spd <= -6.0f)
    {
        tgt_yaw_spd = -6.0f;
    }

    float yawSpdOut       = _module_deps.pid_deps.yaw_spd->calculate(tgt_yaw_spd, _ctx.data.imu.gyro[0]);
    //最终输出
    _ctx.data.output.yawCurrent = yawSpdOut - (disrupt_hat) / b0 * 0.5f;


}


void wl_gimbal_t::_fsm_execute()
{

    if (_ctx.data.mode == cmd_base_t::mode_t::ACTIVE)
    {
        _main_fsm.change_state(&_state_active);
    }
    else
    {
        _main_fsm.change_state(&_state_passive);
    }
    _main_fsm.execute(this);
    wl_gimbal_t::_send_motor_command();
}

void wl_gimbal_t::set_pitchstate(bool enable)
{
    if (enable)
    {
        instance()->_module_deps.motor_deps.pitch->enable();
    }
    else
    {
        instance()->_module_deps.motor_deps.pitch->disable();
    }
}

void wl_gimbal_t::set_yawstate(bool enable)
{
    if (enable)
    {
        instance()->_module_deps.motor_deps.yaw->enable();
    }
    else
    {
        instance()->_module_deps.motor_deps.yaw->disable();
    }
}

void wl_gimbal_t::_send_motor_command()
{
    if(_ctx.data.output.pitchTorque >= 7.0f)
    {
        _ctx.data.output.pitchTorque = 7.0f;
    }
    else if(_ctx.data.output.pitchTorque <= -7.0f)
    {
        _ctx.data.output.pitchTorque = -7.0f;
    }
    _module_deps.motor_deps.pitch->send_torque(_ctx.data.output.pitchTorque);
    _module_deps.motor_deps.yaw->send_torque(_ctx.data.output.yawCurrent);
}

float wl_gimbal_t::wrapAngle(float angle) {
    while(angle<-PI)
    {
        angle+=2*PI;
    }
    while(angle>PI)
    {
        angle-=2*PI;
    }
    return angle;
}
}
