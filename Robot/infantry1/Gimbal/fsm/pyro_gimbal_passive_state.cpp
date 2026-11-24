#include "pyro_wl_gimbal.h"

void pyro::wl_gimbal_t::state_passive_t::enter(owner* owner) {
    owner->_ctx.data.output.pitchTorque = 0.0f;
    owner->_ctx.data.output.yawCurrent             = 0.0f;
    owner->_ctx.data.telem.targetYawRad            = owner->_ctx.data.imu.yaw;
    owner->_ctx.data.telem.targetPitchRad          = owner->_ctx.data.imu.pitch;
    owner->_ctx.data.output.pitchEn                = false;
    owner->_ctx.data.output.yawEn                  = false;


    owner->_module_deps.pid_deps.yaw_pos->clear();
    owner->_module_deps.pid_deps.yaw_spd->clear();
    owner->set_pitchstate(owner->_ctx.data.output.pitchEn);
    owner->set_yawstate(owner->_ctx.data.output.yawEn);
}

void pyro::wl_gimbal_t::state_passive_t::execute(owner* owner)
{
    owner->set_pitchstate(owner->_ctx.data.output.pitchEn);
    owner->_ctx.data.telem.targetYawRad            = owner->_ctx.data.imu.yaw;
    owner->_ctx.data.telem.targetPitchRad          = owner->_ctx.data.imu.pitch;
    owner->_ctx.data.output.pitchTorque = 0.0f;
    if(owner->_ctx.data.mode == cmd_base_t::mode_t::ACTIVE)
    {
        request_switch(&instance()->_state_active);
    }
}

void pyro::wl_gimbal_t::state_passive_t::exit(owner* owner)
{

}