#include "pyro_wl_chassis.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

constexpr static float WZ = 12.0f;
constexpr static float TARGET_RECOVERY_WZ = 5.0f;
constexpr static float SPIN_RECOVERY_ANGLE_EPSILON    = 1.0f; // rad

static float spin_speed_ref = 0.0f;
static bool if_exit_spin = false;
static int convert_count = 0;
static bool is_add_speed = 0; // 是否是加速阶段

void wl_chassis_t::fsm_active_t::state_normal_t::state_spin_t::enter(wl_chassis_t *owner)
{
    owner->_ctx.data.roll_pid = {};
    owner->_ctx.data.mean_leg_length_pid = {};
    owner->_ctx.data.odom.real_x = 0.0f;
    owner->_reset_forward_kf();
    auto &target = owner->_ctx.data.target_state;
    target.x = owner->_ctx.data.measured_state.x;
    target.dot_x = 0.0f;
    target.psi = owner->_ctx.data.measured_state.psi;
    target.dot_psi = owner->_ctx.data.ins.gyro[0];
    spin_speed_ref = owner->_ctx.data.ins.gyro[0];
    owner->_ctx.data.target_aux.L = owner->_ctx.data.airborne.landing_recovery
                   ? owner->_ctx.data.airborne.L_ref
                   : NORMAL_LENGTH_TARGET;
    owner->_ctx.data.target_aux.dot_L = 0.0f;
    target.theta = 0.0f;
    target.dot_theta = 0.0f;
    owner->_ctx.data.target_aux.phi = 0.0f;
    owner->_ctx.data.target_aux.dot_phi = 0.0f;
    target.beta1 = 0.0f;
    target.dot_beta1 = 0.0f;
    target.beta2 = 0.0f;
    target.dot_beta2 = 0.0f;

    for(float &feedforward : owner->_ctx.data.U0)
    {
        feedforward = 0.0f;
    }

    const float avg_length =
        0.5f * (owner->_ctx.data.leg[leg_def::L].current_leg_length +
                owner->_ctx.data.leg[leg_def::R].current_leg_length);
    for(auto &leg : owner->_ctx.data.leg)
    {
        leg.target_leg_rad = leg.current_leg_rad;
        leg.target_leg_speed = leg.current_leg_speed;
        leg.target_leg_radps = leg.current_leg_radps;
        leg.target_leg_length = avg_length;
        leg.out_F_L = 0.0f;
        leg.out_T_p = 0.0f;
        leg.out_joint_torque[joint_def::HIP] = 0.0f;
        leg.out_joint_torque[joint_def::KNEE] = 0.0f;
    }

    owner->_ctx.motor.wheel[leg_def::L]->enable();
    owner->_ctx.motor.wheel[leg_def::R]->enable();

    if_exit_spin = false;
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_spin_t::execute(wl_chassis_t *owner)
{
    //退出机制
    if(owner->_ctx.data.current_function == chassis_function_state_t::SPIN_TOGGLE || if_exit_spin)
    {
        //之后都要来这里
        if_exit_spin = true;

        const float gimbal_psi = owner->_ctx.data.measured_state.psi;
        if ((std::fabs(gimbal_psi) <= SPIN_RECOVERY_ANGLE_EPSILON) &&
            owner->_ctx.data.measured_state.dot_psi <= TARGET_RECOVERY_WZ + 1.0f)
        {
            owner->_ctx.data.target_state.psi = 0.0f;
            owner->_ctx.data.target_state.dot_psi = 0.0f;
            if_exit_spin = false;
            request_switch(&owner->_state_active._state_normal._state_balance);
        }
        else
        {
            float directed_error = -gimbal_psi;
            if(directed_error < 0.0f)
            {
                directed_error += 2.0f * PI;
            }


            if(spin_speed_ref >= TARGET_RECOVERY_WZ)
            {
                const float spin_accel_step = 4.0f * SPIN_YAW_ACCEL * owner->_ctx.data._dt;
                spin_speed_ref -= spin_accel_step;
            }
            else
            {
                spin_speed_ref = TARGET_RECOVERY_WZ;
            }

        }

    }
    else
    {
        //此时不是退出
        //紧急下力
        static uint16_t reset_count = 0;
        if(std::fabs(owner->_ctx.data.ins.euler_rad[1]) >= PI / 4.0f ||
           std::fabs(owner->_ctx.data.ins.euler_rad[2]) >= PI / 9.0f)
        {
            if(reset_count >= 50)
            {
                owner->_ctx.data.flag.leg_is_should_restart = true;
                return;
            }
            ++reset_count;
        }
        else
        {
            reset_count = 0;
        }

        //落地恢复
        if(owner->_ctx.data.airborne.landing_recovery)
        {
            owner->_execute_landing_recovery();
        }
        else
        {
            const float target_dot_L = owner->_current_cmd.dot_L;
            owner->_ctx.data.target_aux.dot_L = target_dot_L;
            owner->_ctx.data.target_aux.L = std::clamp(
                owner->_ctx.data.target_aux.L +
                    target_dot_L * owner->_ctx.data._dt,
                MIN_LEG_LENGTH, MAX_LEG_LENGTH);
        }

        //缓加速
        if(is_add_speed)
        {
            //小陀螺加速时的小量
            const float spin_accel_step = SPIN_YAW_ACCEL * owner->_ctx.data._dt;
            if(spin_speed_ref < WZ - spin_accel_step)
            {
                spin_speed_ref += spin_accel_step;
            }
            else if(spin_speed_ref > WZ + spin_accel_step)
            {
                spin_speed_ref -= spin_accel_step;
            }
            else
            {
                spin_speed_ref = WZ;
                convert_count++;

                convert_count = 0;
                is_add_speed = false;

            }
        }
        else
        {
            spin_speed_ref = owner->_ctx.data.measured_state.dot_psi;
            if(owner->_ctx.data.measured_state.dot_psi <= 6.0f)
            {
                is_add_speed = true;
            }
        }



    }



    //无视其它状态
    owner->_ctx.data.measured_state.psi = owner->_ctx.data.ins.euler_rad[0];
    owner->_ctx.data.measured_state.dot_psi = owner->_ctx.data.ins.gyro[0];
    owner->_update_forward_kf();
    owner->_ctx.data.target_state.x = owner->_ctx.data.measured_state.x;
    owner->_ctx.data.target_state.dot_x = 0.0f;
    owner->_ctx.data.target_state.psi = owner->_ctx.data.measured_state.psi;
    owner->_ctx.data.target_state.dot_psi = spin_speed_ref;

    owner->_gain_calculate();
    owner->_balance_control(true, true);

    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_spin_t::exit(wl_chassis_t *owner)
{
    owner->_ctx.data.roll_pid = {};
    owner->_ctx.data.mean_leg_length_pid = {};
}

} // namespace pyro
