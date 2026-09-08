#include "pyro_sentry_gimbal.h"
#include "gimbal_config.h"
namespace pyro
{

void sentry_gimbal_t::state_passive_t::enter(owner *owner)
{


    owner->_ctx.motor.motor_yaw->disable();
    owner->_ctx.motor.motor_pitch->disable();

}

void sentry_gimbal_t::state_passive_t::execute(owner *owner)
{
    owner->_ctx.data.imu_data.target_yaw_rad = owner->_ctx.data.imu_data.current_yaw_rad;
    owner->_ctx.data.target_data.pitch_pos = owner->_ctx.data.current_data.pitch_pos;

    owner->_ctx.motor.motor_yaw->send_torque(0);
    owner->_ctx.motor.motor_pitch->send_torque(0);


}

void sentry_gimbal_t::state_passive_t::exit(owner *owner)
{
}

} // namespace pyro