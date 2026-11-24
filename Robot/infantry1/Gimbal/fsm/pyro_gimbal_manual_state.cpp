#include "pyro_wl_gimbal.h"
#include "gimbal_config.h"




void pyro::wl_gimbal_t::fsm_active_t::state_manual_t::enter(owner *owner)
{
    owner->_ctx.data.telem.targetPitchRad = 0.0f;
    owner->_ctx.data.telem.targetYawRad   = owner->_ctx.data.imu.yaw;
    owner->_ctx.data.output.pitchEn       = true;
    owner->_ctx.data.output.yawEn         = true;
    instance()->set_pitchstate(owner->_ctx.data.output.pitchEn);
    instance()->set_yawstate(owner->_ctx.data.output.yawEn);
}

void pyro::wl_gimbal_t::fsm_active_t::state_manual_t::execute(owner *owner)
{
    //如果底盘此时需要重新复位，立刻切回复位状态
    if(!owner->_ctx.data.chassis_is_ready)
    {
        request_switch(&owner->_state_active._state_align);
    }

    //判断自瞄数据是否有效
    bool is_autoaim_cmd_useless = false;
    if (abs(owner->_ctx.data.telem.targetPitchRad) > 2.1f * PI ||
        abs(owner->_ctx.data.telem.targetYawRad) > 2.1f * PI)
    {
        is_autoaim_cmd_useless = true;
    }
    else
    {
        is_autoaim_cmd_useless = false;
    }

    //目标角度更新,需区分自瞄和手动
    if(owner->_ctx.data.motionState == MotionState::Auto && (!is_autoaim_cmd_useless))
    {
        owner->_ctx.data.telem.targetPitchRad = owner->_ctx.data.telem.targetPitchRad;
        owner->_ctx.data.telem.targetYawRad   = owner->_ctx.data.telem.targetYawRad;
    }
    else if(owner->_ctx.data.motionState == MotionState::Manual || is_autoaim_cmd_useless)
    {
        owner->_ctx.data.telem.targetPitchRad +=
                    owner->_ctx.data.telem.target_pitch_vel * owner->_ctx.data.dt;
        owner->_ctx.data.telem.targetYawRad = owner->wrapAngle(
                    owner->_ctx.data.telem.targetYawRad -
                    owner->_ctx.data.telem.target_yaw_vel * owner->_ctx.data.dt);
    }

    owner->updatePitch();
    owner->updateYaw();

}

void pyro::wl_gimbal_t::fsm_active_t::state_manual_t::exit(owner *owner)
{
    (void)owner;
}
