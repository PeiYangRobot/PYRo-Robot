#include "pyro_algo_common.h"
#include "pyro_wl_chassis.h"

#include <algorithm>


namespace pyro
{

//最短腿长的摆角：1.18 -- 2.1

static constexpr float STEP_PHASE1_MAX_RAD       = 0.2f;
static constexpr float STEP_PHASE1_TARGET_RAD    = 0.1f;
static constexpr float STEP_PHASE1_TARGET_LENGTH = 0.35f;

static constexpr float STEP_PHASE2_TARGET_LENGTH = 0.19f;

static constexpr float STEP_PHASE3_TARGET_RAD    = 1.2f;
static constexpr float STEP_PHASE3_MIN_RAD       = 0.75f;

static constexpr float STEP_DELTA_RAD            = 0.006f;
static constexpr float STEP_DELTA_LENGTH         = 0.001f;



static int stepping_phase = 0;

static uint8_t phase_1_ticks;
static uint8_t phase_2_ticks;
static uint8_t phase_3_ticks;

void wl_chassis_t::fsm_active_t::state_normal_t::state_step_t::enter(wl_chassis_t *owner)
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


    stepping_phase = 0;
    phase_1_ticks  = 0;
    phase_2_ticks  = 0;
    phase_3_ticks  = 0;

}

void wl_chassis_t::fsm_active_t::state_normal_t::state_step_t::execute(wl_chassis_t *owner)
{
    if(stepping_phase == 0)
    {
        if(owner->_ctx.data.leg[leg_def::L].current_leg_length >= STEP_PHASE1_TARGET_LENGTH - 0.03f &&
           owner->_ctx.data.leg[leg_def::R].current_leg_length >= STEP_PHASE1_TARGET_LENGTH - 0.03f &&
           owner->_ctx.data.leg[leg_def::L].current_leg_rad    <= STEP_PHASE1_MAX_RAD &&
           owner->_ctx.data.leg[leg_def::R].current_leg_rad    <= STEP_PHASE1_MAX_RAD )
        {
            if(phase_1_ticks >= 10)
            {
                owner->_ctx.data.leg[leg_def::L].target_leg_rad = STEP_PHASE1_TARGET_RAD;
                owner->_ctx.data.leg[leg_def::R].target_leg_rad = STEP_PHASE1_TARGET_RAD;
                stepping_phase++;
            }
            phase_1_ticks++;
        }
        else
        {
            phase_1_ticks = 0;
        }

        //腿角向后摆并加长腿长
        if(owner->_ctx.data.leg[leg_def::L].target_leg_length <= STEP_PHASE1_TARGET_LENGTH)
        {
            owner->_ctx.data.leg[leg_def::L].target_leg_length += STEP_DELTA_LENGTH;
        }
        if(owner->_ctx.data.leg[leg_def::R].target_leg_length <= STEP_PHASE1_TARGET_LENGTH)
        {
            owner->_ctx.data.leg[leg_def::R].target_leg_length += STEP_DELTA_LENGTH;
        }
        if(owner->_ctx.data.leg[leg_def::L].target_leg_rad  >= STEP_PHASE1_TARGET_RAD)
        {
            owner->_ctx.data.leg[leg_def::L].target_leg_rad  -= STEP_DELTA_RAD;
        }
        if(owner->_ctx.data.leg[leg_def::R].target_leg_rad  >= STEP_PHASE1_TARGET_RAD)
        {
            owner->_ctx.data.leg[leg_def::R].target_leg_rad  -= STEP_DELTA_RAD;
        }

    }
    else if(stepping_phase == 1)
    {
        if(owner->_ctx.data.leg[leg_def::L].current_leg_length <= STEP_PHASE2_TARGET_LENGTH + 0.02f &&
           owner->_ctx.data.leg[leg_def::R].current_leg_length <= STEP_PHASE2_TARGET_LENGTH + 0.02f )
        {
            if(phase_2_ticks >= 10)
            {
                owner->_ctx.data.leg[leg_def::L].target_leg_length = STEP_PHASE2_TARGET_LENGTH;
                owner->_ctx.data.leg[leg_def::R].target_leg_length = STEP_PHASE2_TARGET_LENGTH;
                stepping_phase++;
            }
            phase_2_ticks++;
        }
        else
        {
            phase_2_ticks = 0;
        }

        //缩短腿长
        if(owner->_ctx.data.leg[leg_def::L].target_leg_length >= STEP_PHASE2_TARGET_LENGTH)
        {
            owner->_ctx.data.leg[leg_def::L].target_leg_length -= STEP_DELTA_LENGTH;
        }
        if(owner->_ctx.data.leg[leg_def::R].target_leg_length >= STEP_PHASE2_TARGET_LENGTH)
        {
            owner->_ctx.data.leg[leg_def::R].target_leg_length -= STEP_DELTA_LENGTH;
        }


    }
    else if(stepping_phase == 2)
    {
        if(owner->_ctx.data.leg[leg_def::L].current_leg_rad >= STEP_PHASE3_MIN_RAD &&
           owner->_ctx.data.leg[leg_def::R].current_leg_rad >= STEP_PHASE3_MIN_RAD )
        {
            if(phase_3_ticks >= 10)
            {
                //退出该状态
                request_switch(&owner->_state_active._state_normal._state_balance);
                stepping_phase++;
            }
            phase_3_ticks++;
        }
        else
        {
            phase_3_ticks = 0;
        }
        //腿角回归
        if(owner->_ctx.data.leg[leg_def::L].target_leg_rad  <= STEP_PHASE3_TARGET_RAD)
        {
            owner->_ctx.data.leg[leg_def::L].target_leg_rad  += STEP_DELTA_RAD;
        }
        if(owner->_ctx.data.leg[leg_def::R].target_leg_rad  <= STEP_PHASE3_TARGET_RAD)
        {
            owner->_ctx.data.leg[leg_def::R].target_leg_rad  += STEP_DELTA_RAD;
        }
    }
    else if(stepping_phase == 3)
    {
        //退出该状态
        request_switch(&owner->_state_active._state_normal._state_balance);
    }

    //限幅
    owner->_ctx.data.leg[leg_def::L].target_leg_rad =
            loop_fp32_constrain(owner->_ctx.data.leg[leg_def::L].target_leg_rad +
            owner->_current_cmd.delta_leg_rad[leg_def::L],
                   -PI, PI);

    //限幅
    owner->_ctx.data.leg[leg_def::R].target_leg_rad =
            loop_fp32_constrain(owner->_ctx.data.leg[leg_def::R].target_leg_rad +
            owner->_current_cmd.delta_leg_rad[leg_def::R],
                   -PI, PI);

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

void wl_chassis_t::fsm_active_t::state_normal_t::state_step_t::exit(wl_chassis_t *owner)
{
    stepping_phase = 0;
    (void)owner;
}

} // namespace pyro
