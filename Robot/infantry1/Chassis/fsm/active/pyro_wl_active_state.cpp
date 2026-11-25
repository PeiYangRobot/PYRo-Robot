#include "pyro_dm_motor_drv.h"
#include "pyro_wl_chassis.h"

namespace pyro
{

void wl_chassis_t::fsm_active_t::on_enter(wl_chassis_t *owner)
{
    owner->_getover_elapsed = 0.0f;
    owner->_getover_rearm_elapsed = GETOVER_REARM_DELAY;
    owner->_getover_target_length[leg_def::L] = NORMAL_LENGTH_TARGET;
    owner->_getover_target_length[leg_def::R] = NORMAL_LENGTH_TARGET;

    owner->_ctx.data.airborne.landing_recovery = false;
    owner->_ctx.data.airborne.takeoff_counter = 0;
    owner->_ctx.data.airborne.landing_counter = 0;
    static_cast<dm_motor_drv_t*>(owner->_ctx.motor.joint[leg_def::L][joint_def::HIP])->clear_error();
    static_cast<dm_motor_drv_t*>(owner->_ctx.motor.joint[leg_def::L][joint_def::KNEE])->clear_error();
    static_cast<dm_motor_drv_t*>(owner->_ctx.motor.joint[leg_def::R][joint_def::HIP])->clear_error();
    static_cast<dm_motor_drv_t*>(owner->_ctx.motor.joint[leg_def::R][joint_def::KNEE])->clear_error();
    owner->_ctx.motor.joint[leg_def::L][joint_def::HIP]->enable();
    owner->_ctx.motor.joint[leg_def::L][joint_def::KNEE]->enable();
    owner->_ctx.motor.joint[leg_def::R][joint_def::HIP]->enable();
    owner->_ctx.motor.joint[leg_def::R][joint_def::KNEE]->enable();

}


void wl_chassis_t::fsm_active_t::on_execute(wl_chassis_t *ctx)
{
    if(ctx->_current_cmd.cmd_continus_state == pyro::chassis_active_state_t::MANUAL)
    {
        change_state(&_state_manual);
    }
    else if(ctx->_current_cmd.cmd_continus_state == pyro::chassis_active_state_t::NORMAL)
    {
        change_state(&_state_normal);
    }
}

void wl_chassis_t::fsm_active_t::on_exit(wl_chassis_t *ctx)
{
    (void)ctx;
}

} // namespace pyro
