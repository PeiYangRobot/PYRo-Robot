#include "pyro_wl_gimbal.h"

void pyro::wl_gimbal_t::fsm_active_t::on_enter(owner* owner) {
    owner->_ctx.data.output.pitchEn                = true;
    owner->_ctx.data.output.yawEn                  = true;

    instance()->set_pitchstate(owner->_ctx.data.output.pitchEn);
    instance()->set_yawstate(owner->_ctx.data.output.yawEn);

    change_state(&owner->_state_active._state_align);
}

void pyro::wl_gimbal_t::fsm_active_t::on_execute(owner* owner)
{
}

void pyro::wl_gimbal_t::fsm_active_t::on_exit(owner* owner)
{

}