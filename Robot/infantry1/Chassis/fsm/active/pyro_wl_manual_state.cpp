#include "pyro_algo_common.h"
#include "pyro_wl_chassis.h"

#include <algorithm>


namespace pyro
{


void wl_chassis_t::fsm_active_t::state_manual_t::enter(wl_chassis_t *owner)
{
    for (auto &leg : owner->_ctx.data.leg)
    {
        leg.target_leg_length                 = leg.current_leg_length;
        leg.target_leg_rad                    = leg.current_leg_rad;
        leg.target_leg_speed                  = leg.current_leg_speed;
        leg.target_leg_radps                  = leg.current_leg_radps;
        leg.out_F_L                           = 0;
        leg.out_T_p                           = 0;
        leg.out_joint_torque[joint_def::HIP]  = 0;
        leg.out_joint_torque[joint_def::KNEE] = 0;
    }
    owner->_ctx.motor.wheel[leg_def::L]->disable();
    owner->_ctx.motor.wheel[leg_def::R]->disable();
}

void wl_chassis_t::fsm_active_t::state_manual_t::execute(wl_chassis_t *owner)
{

    //加上遥控器的小量
    owner->_ctx.data.leg[leg_def::L].target_leg_length +=
        owner->_current_cmd.delta_leg_length[leg_def::L];
    owner->_ctx.data.leg[leg_def::R].target_leg_length +=
        owner->_current_cmd.delta_leg_length[leg_def::R];

    //限幅
    owner->_ctx.data.leg[leg_def::L].target_leg_length =
        std::clamp(owner->_ctx.data.leg[leg_def::L].target_leg_length,
                   MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    owner->_ctx.data.leg[leg_def::R].target_leg_length =
        std::clamp(owner->_ctx.data.leg[leg_def::R].target_leg_length,
                   MIN_LEG_LENGTH, MAX_LEG_LENGTH);


    owner->_ctx.data.leg[leg_def::L].target_leg_rad =
        loop_fp32_constrain(owner->_ctx.data.leg[leg_def::L].target_leg_rad +
                           owner->_current_cmd.delta_leg_rad[leg_def::L],
                       -PI, PI);
    owner->_ctx.data.leg[leg_def::L].error_leg_rad =
        loop_fp32_constrain(owner->_ctx.data.leg[leg_def::L].current_leg_rad -
                                owner->_ctx.data.leg[leg_def::L].target_leg_rad,
                            -PI, PI);

    owner->_ctx.data.leg[leg_def::R].target_leg_rad =
        loop_fp32_constrain(owner->_ctx.data.leg[leg_def::R].target_leg_rad +
                           owner->_current_cmd.delta_leg_rad[leg_def::R],
                       -PI, PI);
    owner->_ctx.data.leg[leg_def::R].error_leg_rad =
        loop_fp32_constrain(owner->_ctx.data.leg[leg_def::R].current_leg_rad -
                                owner->_ctx.data.leg[leg_def::R].target_leg_rad,
                            -PI, PI);


    owner->_manual_control();
    owner->_vmc_trans_v2j(false);
    owner->_send_joint_torque();
    owner->_ctx.motor.wheel[leg_def::L]->send_torque(0);
    owner->_ctx.motor.wheel[leg_def::R]->send_torque(0);
}

void wl_chassis_t::fsm_active_t::state_manual_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
