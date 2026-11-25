#include "pyro_algo_common.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_wl_chassis.h"

#include <algorithm>

namespace pyro
{



    static constexpr float ALIGN_MAX_RAD       = 1.65f;
    static constexpr float ALIGN_MIN_RAD       = 0.75f;
    static constexpr float ALIGN_TARGET_RAD    = 0.9f;
    static constexpr float ALIGN_TARGET_LENGTH = 0.11f;
    static constexpr float ALIGN_DELTA_LENGTH  = 0.0003f;
    static constexpr float ALIGN_DELTA_RAD     = 0.003f;


    void wl_chassis_t::fsm_active_t::state_normal_t::state_align_t::enter(wl_chassis_t *owner)
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
        owner->_ctx.data.flag.chassis_is_align_ready =false;
    }

    void wl_chassis_t::fsm_active_t::state_normal_t::state_align_t::execute(wl_chassis_t *owner)
    {
        //判断双腿是否在可以起身的位置
        static uint8_t keep_tick =0;
        if(owner->_ctx.data.leg[leg_def::L].current_leg_rad <= ALIGN_MAX_RAD &&
           owner->_ctx.data.leg[leg_def::L].current_leg_rad >= ALIGN_MIN_RAD &&
           owner->_ctx.data.leg[leg_def::R].current_leg_rad <= ALIGN_MAX_RAD &&
           owner->_ctx.data.leg[leg_def::R].current_leg_rad >= ALIGN_MIN_RAD &&
           owner->_ctx.data.leg[leg_def::L].current_leg_length <=MIN_LEG_LENGTH+0.06f&&
           owner->_ctx.data.leg[leg_def::R].current_leg_length <=MIN_LEG_LENGTH+0.06f&&
           abs(owner->_ctx.data.leg[leg_def::L].current_leg_rad-owner->_ctx.data.leg[leg_def::R].current_leg_rad)<=0.2f)
        {
            if(keep_tick >=20)
            {
                owner->_ctx.data.flag.chassis_is_align_ready = true;
                request_switch(&owner->_state_active._state_normal._state_balance);
            }
            keep_tick++;
        }
        else
        {
            keep_tick = 0;
        }

        #if Using_Gimbal_Cmd
        //判断只有yaw电机在目标复位角度内才能进行腿部复位
        if(fabs(owner->_ctx.data.yaw.pos) <= 0.3f)
        #endif
        {
            if(owner->_ctx.data.leg[leg_def::L].target_leg_length >= ALIGN_TARGET_LENGTH)
            {
                owner->_ctx.data.leg[leg_def::L].target_leg_length -= ALIGN_DELTA_LENGTH;
            }
            if(owner->_ctx.data.leg[leg_def::R].target_leg_length >= ALIGN_TARGET_LENGTH)
            {
                owner->_ctx.data.leg[leg_def::R].target_leg_length -= ALIGN_DELTA_LENGTH;
            }
            if(owner->_ctx.data.leg[leg_def::L].target_leg_rad  >= ALIGN_MAX_RAD ||
               owner->_ctx.data.leg[leg_def::L].target_leg_rad  <= ALIGN_TARGET_RAD )
            {
                owner->_ctx.data.leg[leg_def::L].target_leg_rad  += ALIGN_DELTA_RAD;
            }
            if(owner->_ctx.data.leg[leg_def::R].target_leg_rad  >= ALIGN_MAX_RAD ||
               owner->_ctx.data.leg[leg_def::R].target_leg_rad  <= ALIGN_TARGET_RAD)
            {
                owner->_ctx.data.leg[leg_def::R].target_leg_rad  += ALIGN_DELTA_RAD;
            }
        }



        //限幅
        owner->_ctx.data.leg[leg_def::L].target_leg_rad =
                loop_fp32_constrain(owner->_ctx.data.leg[leg_def::L].target_leg_rad, 0, 2*PI);

        //限幅
        owner->_ctx.data.leg[leg_def::R].target_leg_rad =
                loop_fp32_constrain(owner->_ctx.data.leg[leg_def::R].target_leg_rad, 0, 2*PI);

        //限幅
        owner->_ctx.data.leg[leg_def::L].target_leg_length =
            std::clamp(owner->_ctx.data.leg[leg_def::L].target_leg_length,
                       MIN_LEG_LENGTH, MAX_LEG_LENGTH);

        //限幅
            owner->_ctx.data.leg[leg_def::R].target_leg_length =
            std::clamp(owner->_ctx.data.leg[leg_def::R].target_leg_length,
                       MIN_LEG_LENGTH, MAX_LEG_LENGTH);

        owner->_ctx.data.leg[leg_def::L].error_leg_rad =
            loop_fp32_constrain(owner->_ctx.data.leg[leg_def::L].current_leg_rad -
                                owner->_ctx.data.leg[leg_def::L].target_leg_rad,
                            -PI, PI);


        owner->_ctx.data.leg[leg_def::R].error_leg_rad =
            loop_fp32_constrain(owner->_ctx.data.leg[leg_def::R].current_leg_rad -
                                owner->_ctx.data.leg[leg_def::R].target_leg_rad,
                            -PI, PI);


        owner->_manual_control();
        owner->_vmc_trans_v2j();
        owner->_send_joint_torque();
        owner->_ctx.motor.wheel[leg_def::L]->send_torque(0);
        owner->_ctx.motor.wheel[leg_def::R]->send_torque(0);
    }

    void wl_chassis_t::fsm_active_t::state_normal_t::state_align_t::exit(wl_chassis_t *owner)
    {
        owner->_ctx.data.flag.chassis_is_align_ready = true;
    }

}