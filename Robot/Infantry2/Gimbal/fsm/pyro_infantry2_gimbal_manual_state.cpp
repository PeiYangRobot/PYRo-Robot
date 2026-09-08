#include "pyro_infantry2_gimbal.h"
#include <algorithm>

namespace pyro {

void infantry2_gimbal_t::fsm_active_t::state_manual_t::enter(owner *owner) {}

void infantry2_gimbal_t::fsm_active_t::state_manual_t::execute(owner *owner) {
    if(owner->_ctx.cmd->is_imu_control) {
        //imu的yaw轴方向相反，用减法
        owner->_ctx.data.target_pitch_rad -= owner->_ctx.cmd->target_pitch_delta_angle;
        owner->_ctx.data.target_yaw_rad -= owner->_ctx.cmd->target_yaw_delta_angle;
        owner->_ctx.data.target_pitch_rad = std::clamp(owner->_ctx.data.target_pitch_rad,
            infantry2_gimbal::PITCH_MIN_MOTOR_RAD, infantry2_gimbal::PITCH_MAX_MOTOR_RAD);
        
        //角度超限
        const float yaw_error = owner->_ctx.data.target_yaw_rad - owner->_ctx.data.current_yaw_imu_rad;
        if (yaw_error > PI)
            owner->_ctx.data.target_yaw_rad -= 2.0f * PI;
        else if (yaw_error < -PI)
            owner->_ctx.data.target_yaw_rad += 2.0f * PI;

        _imu_control(&owner->_ctx);
    } else {
        // 电机机械角度yaw和pitch都与实际方向相反，所以用减
        owner->_ctx.data.target_pitch_rad -= owner->_ctx.cmd->target_pitch_delta_angle;
        owner->_ctx.data.target_yaw_rad -= owner->_ctx.cmd->target_yaw_delta_angle;
        //按电机机械角度控制来限幅
        owner->_ctx.data.target_pitch_rad = std::clamp(owner->_ctx.data.target_pitch_rad,
            infantry2_gimbal::PITCH_MIN_MOTOR_RAD, infantry2_gimbal::PITCH_MAX_MOTOR_RAD);

        //角度超限
        const float yaw_error = owner->_ctx.data.target_yaw_rad - owner->_ctx.data.current_yaw_motor_rad;
        if (yaw_error > PI)
            owner->_ctx.data.target_yaw_rad -= 2.0f * PI;
        else if (yaw_error < -PI)
            owner->_ctx.data.target_yaw_rad += 2.0f * PI;
        
        _mec_control(&owner->_ctx);
    }
    _send_motor_command(&owner->_ctx);
}

void infantry2_gimbal_t::fsm_active_t::state_manual_t::exit(owner *owner) {}

} // namespace pyro