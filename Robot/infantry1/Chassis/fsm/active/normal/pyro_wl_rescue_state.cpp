#include "pyro_algo_common.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_wl_chassis.h"

#include <algorithm>

namespace pyro
{

    static constexpr float RESCUE_TARGET_LENGTH = 0.32f;
    static constexpr float RESCUE_DELTA_LENGTH  = 0.0006f;
    static constexpr float RESCUE_DELTA_RAD     = 0.003f;
    enum class falling_type
    {
        FALL_FORWARD,
        FALL_BACKWARD
    };
    static falling_type reason;
    static int rescue_ok_count;


    void wl_chassis_t::fsm_active_t::state_normal_t::state_rescue_t::enter(wl_chassis_t *owner)
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
        owner->_ctx.data.flag.chassis_is_rescuing = true;

        //已废弃

        // // 判断翻倒类型
        // // 翻倒类型有：
        // // 1.roll = 0， pitch在-90到90间（前倾或者后倾角度比较小）
        // // 2.roll = 180度左右，pitch在-90到90间（前倾或后倾角度过大，超过90度）
        // // 3.roll = 90度左右，pitch值在0度左右（侧翻）

        // // 判断逻辑：
        // // 首先判断roll 如果roll轴绝对值在45到150度以内，全都定性为侧翻
        // // 接下来，如果roll小于45，就通过判断pitch来判断是前翻还是后翻
        // // 如果roll大于150，则为pitch翻过的角度大于90度导致的解算结果，对其进行处理

        //已废弃


        reason = owner->_ctx.data.ins.euler_rad[2] >= 0.0f ?
                    falling_type::FALL_FORWARD : falling_type::FALL_BACKWARD;
    }

    void wl_chassis_t::fsm_active_t::state_normal_t::state_rescue_t::execute(wl_chassis_t *owner)
    {
        //判断此时是否在完成复位的条件内
        if (abs(owner->_ctx.data.ins.euler_rad[1]) <= PI / 3.0f &&
            abs(owner->_ctx.data.ins.euler_rad[2]) <= PI / 4.0f)
        {
            if(rescue_ok_count >= 100)
            {
                request_switch(&owner->_state_active._state_normal._state_align);
            }
            rescue_ok_count++;
        }
        else
        {
            rescue_ok_count = 0;
            //不在目标条件，继续执行自救流程
            //先把腿长伸到最长，再进行摆腿
            if(owner->_ctx.data.leg[leg_def::L].target_leg_length <= RESCUE_TARGET_LENGTH ||
                owner->_ctx.data.leg[leg_def::R].target_leg_length <= RESCUE_TARGET_LENGTH)
            {
                //腿长还没有伸到最长，继续伸长
                if(owner->_ctx.data.leg[leg_def::L].target_leg_length <= RESCUE_TARGET_LENGTH)
                {
                    owner->_ctx.data.leg[leg_def::L].target_leg_length += RESCUE_DELTA_LENGTH;
                }
                if(owner->_ctx.data.leg[leg_def::R].target_leg_length <= RESCUE_TARGET_LENGTH)
                {
                    owner->_ctx.data.leg[leg_def::R].target_leg_length += RESCUE_DELTA_LENGTH;
                }
            }
            else
            {
                //从车的左侧看去，此时车子朝向左方，腿顺时针转为正
                // switch(reason)
                // {
                //     case falling_type::FALL_FORWARD:
                //         // 此时需要腿逆时针转动
                //         //两腿先到达同一角度再一起运动，由相对摆动方向落后的那个腿先动
                //         if (abs(owner->_ctx.data.leg[leg_def::L].target_leg_rad -
                //                 owner->_ctx.data.leg[leg_def::R].target_leg_rad) <= 0.1f)
                //         {
                //             owner->_ctx.data.leg[leg_def::R].target_leg_rad -= RESCUE_DELTA_RAD;
                //             owner->_ctx.data.leg[leg_def::L].target_leg_rad -= RESCUE_DELTA_RAD;
                //         }
                //         else
                //         {
                //             if (loop_fp32_constrain(owner->_ctx.data.leg[leg_def::L].target_leg_rad -
                //                 owner->_ctx.data.leg[leg_def::R].target_leg_rad, -PI, PI) <= 0.0f)
                //             {
                //                 //先动右腿
                //                 owner->_ctx.data.leg[leg_def::R].target_leg_rad -= RESCUE_DELTA_RAD;
                //             }
                //             else
                //             {
                //                 //先动左腿
                //                 owner->_ctx.data.leg[leg_def::L].target_leg_rad -= RESCUE_DELTA_RAD;
                //             }
                //         }
                //         break;
                //     case falling_type::FALL_BACKWARD:
                        // 此时需要腿顺时针转动
                        //同理
                        if (abs(owner->_ctx.data.leg[leg_def::L].target_leg_rad -
                                owner->_ctx.data.leg[leg_def::R].target_leg_rad) <= 0.1f)
                        {
                            owner->_ctx.data.leg[leg_def::R].target_leg_rad += RESCUE_DELTA_RAD;
                            owner->_ctx.data.leg[leg_def::L].target_leg_rad += RESCUE_DELTA_RAD;
                        }
                        else
                        {
                            if (loop_fp32_constrain(owner->_ctx.data.leg[leg_def::L].target_leg_rad -
                                owner->_ctx.data.leg[leg_def::R].target_leg_rad, -PI, PI) <= 0.0f)
                            {
                                //先动左腿
                                owner->_ctx.data.leg[leg_def::L].target_leg_rad += RESCUE_DELTA_RAD;
                            }
                            else
                            {
                                //先动左腿
                                owner->_ctx.data.leg[leg_def::R].target_leg_rad += RESCUE_DELTA_RAD;
                            }
                        }
                //         break;
                // }
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

    void wl_chassis_t::fsm_active_t::state_normal_t::state_rescue_t::exit(wl_chassis_t *owner)
    {
        owner->_ctx.data.flag.chassis_is_align_ready = true;
        owner->_ctx.data.flag.chassis_is_rescuing = false;
    }

}