#include "pyro_wl_chassis.h"

namespace pyro
{

void wl_chassis_t::state_passive_t::enter(wl_chassis_t *owner)
{
    owner->_ctx.motor.joint[leg_def::L][joint_def::HIP]  ->disable();
    owner->_ctx.motor.joint[leg_def::L][joint_def::KNEE] ->disable();
    owner->_ctx.motor.joint[leg_def::R][joint_def::HIP] ->disable();
    owner->_ctx.motor.joint[leg_def::R][joint_def::KNEE]->disable();

    owner->_ctx.motor.wheel[leg_def::L]->disable();
    owner->_ctx.motor.wheel[leg_def::R]->disable();



    for (auto &leg : owner->_ctx.data.leg)
    {
        leg.target_leg_length = leg.current_leg_length;
        leg.target_leg_rad = leg.current_leg_rad;
        leg.target_leg_speed = leg.current_leg_speed;
        leg.target_leg_radps = leg.current_leg_radps;
        leg.out_F_L = 0;
        leg.out_T_p = 0;
        leg.out_joint_torque[joint_def::HIP]  = 0;
        leg.out_joint_torque[joint_def::KNEE] = 0;
    }


}

void wl_chassis_t::state_passive_t::execute(wl_chassis_t *owner)
{
    for (auto &leg : owner->_ctx.data.leg)
    {
        leg.target_leg_length = leg.current_leg_length;
        leg.target_leg_rad = leg.current_leg_rad;
        leg.target_leg_speed = leg.current_leg_speed;
        leg.target_leg_radps = leg.current_leg_radps;
    }
    owner->_ctx.motor.joint[leg_def::L][joint_def::HIP]  ->send_torque(0);
    owner->_ctx.motor.joint[leg_def::L][joint_def::KNEE] ->send_torque(0);
    owner->_ctx.motor.joint[leg_def::R][joint_def::HIP] ->send_torque(0);
    owner->_ctx.motor.joint[leg_def::R][joint_def::KNEE]->send_torque(0);
    owner->_ctx.motor.wheel[leg_def::L]->send_torque(0);
    owner->_ctx.motor.wheel[leg_def::R]->send_torque(0);


}

void wl_chassis_t::state_passive_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
