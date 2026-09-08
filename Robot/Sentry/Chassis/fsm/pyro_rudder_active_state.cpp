#include "pyro_rudder_chassis.h"
#include "pyro_dm_motor_drv.h"

namespace pyro
{

void rudder_chassis_t::state_active_t::enter(owner *owner)
{
    owner->_ctx.motor.wheel[0]->enable();
    owner->_ctx.motor.wheel[1]->enable();
    owner->_ctx.motor.wheel[2]->enable();
    owner->_ctx.motor.wheel[3]->enable();

    owner->_ctx.motor.rudder[0]->enable();
    owner->_ctx.motor.rudder[1]->enable();
    owner->_ctx.motor.rudder[2]->enable();
    owner->_ctx.motor.rudder[3]->enable();

    static_cast<dm_motor_drv_t *>(owner->_ctx.motor.yaw)->get_error_code();
    static_cast<dm_motor_drv_t *>(owner->_ctx.motor.yaw)->clear_error();
    owner->_ctx.motor.yaw->enable();
}

void rudder_chassis_t::state_active_t::execute(owner *owner)
{

    
    owner->_communicate_gimbal();

    owner->_kinematics_solve();
    owner->_rudder_control();
    owner->_yaw_control();
    owner->_send_motor_command();
    

}

void rudder_chassis_t::state_active_t::exit(owner *owner)
{
}

} // namespace pyro