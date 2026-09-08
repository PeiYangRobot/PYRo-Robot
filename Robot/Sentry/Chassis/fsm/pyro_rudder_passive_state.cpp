#include "pyro_rudder_chassis.h"
#include "rudder_config.h"
namespace pyro
{

void rudder_chassis_t::state_passive_t::enter(owner *owner)
{
    owner->_ctx.motor.wheel[0]->disable();
    owner->_ctx.motor.wheel[1]->disable();
    owner->_ctx.motor.wheel[2]->disable();
    owner->_ctx.motor.wheel[3]->disable();

    owner->_ctx.motor.rudder[0]->disable();
    owner->_ctx.motor.rudder[1]->disable();
    owner->_ctx.motor.rudder[2]->disable();
    owner->_ctx.motor.rudder[3]->disable();

    owner->_ctx.motor.yaw->disable();

}

void rudder_chassis_t::state_passive_t::execute(owner *owner)
{
    owner->_ctx.data.imu_data.target_yaw_rad = owner->_ctx.data.imu_data.current_yaw_rad;
    

    owner->_ctx.motor.wheel[0]->send_torque(0);
    owner->_ctx.motor.wheel[1]->send_torque(0);
    owner->_ctx.motor.wheel[2]->send_torque(0);
    owner->_ctx.motor.wheel[3]->send_torque(0);

    owner->_ctx.motor.rudder[0]->send_torque(0);
    owner->_ctx.motor.rudder[1]->send_torque(0);
    owner->_ctx.motor.rudder[2]->send_torque(0);
    owner->_ctx.motor.rudder[3]->send_torque(0);

    owner->_ctx.motor.yaw->send_torque(0);
}

void rudder_chassis_t::state_passive_t::exit(owner *owner)
{
}

} // namespace pyro