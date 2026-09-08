#include "pyro_sentry_gimbal.h"
#include "pyro_dm_motor_drv.h"

namespace pyro
{

void sentry_gimbal_t::state_active_t::enter(owner *owner)
{
    owner->_ctx.motor.motor_yaw->enable();

    /* DM电机必须先清错误再使能，否则clear_error会复位移除使能状态 */
    auto *pitch_motor = static_cast<dm_motor_drv_t *>(owner->_ctx.motor.motor_pitch);
    if (dm_motor_drv_t::error_code::ok != pitch_motor->get_error_code())
    {
        pitch_motor->clear_error();
    }
    owner->_ctx.motor.motor_pitch->enable();
}

void sentry_gimbal_t::state_active_t::execute(owner *owner)
{

    
    //owner->_communicate_gimbal();
    owner->_solve();
    owner->_gimbal_control();
    owner->_send_motor_command();
    

}

void sentry_gimbal_t::state_active_t::exit(owner *owner)
{
}

} // namespace pyro