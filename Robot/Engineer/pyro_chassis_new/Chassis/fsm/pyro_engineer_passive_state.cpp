#include "pyro_engineer_chassis.h"
#include "engineer_config.h"


// =========================================================
// FSM：被动状态
// =========================================================
namespace pyro{


void engineer_chassis_t::state_passive_t::enter(owner *owner)
{
    // 进入被动状态：禁用所有电机、清零数据、清空PID

    // 麦轮
    for (auto *motor : owner->_ctx.motor.mecanum)
    {
        motor->disable();
    }
    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.data.target_wheel_rpm[i] = 0.0f;
        owner->_ctx.data.out_wheel_torque[i] = 0.0f;
    }
    for (auto *pid : owner->_ctx.pid.mecanum_pid)
    {
        pid->clear();
    }

    //矿仓
    owner->_ctx.motor.magazine->disable();
    owner->_ctx.data.target_magazine_angle = 0.0f;
    owner->_ctx.data.target_magazine_speed = 0.0f;
    owner->_ctx.data.out_magazine_torque = 0.0f;
    owner->_ctx.pid.magazine_pos_pid->clear();
    owner->_ctx.pid.magazine_vel_pid->clear();

    //摇臂
    for(auto *motor : owner ->_ctx.motor.lift){
        motor->disable();
    }
    for(int i = 0; i < 2; i ++){
        owner->_ctx.data.target_lift_angle[i] = 0.0f;
        owner->_ctx.data.out_lift_torque[i] = 0.0f;
    }
    for(auto *pid : owner -> _ctx.pid.lift_pos_pid){
        pid->clear();      
    }
    for(auto *pid : owner -> _ctx.pid.lift_vel_pid){
        pid->clear();
    }

}

void engineer_chassis_t::state_passive_t::execute(owner *owner)
{
    // 被动状态每周期执行：发零扭矩保证安全

    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.data.out_wheel_torque[i] = 0.0f;
    }
    owner->_ctx.data.out_magazine_torque = 0.0f;
    for(int i=0;i<2;i++){
        owner->_ctx.data.out_lift_torque[i] = 0.0f;
    }

    owner->_send_motor_command();
}

void engineer_chassis_t::state_passive_t::exit(owner *owner)
{
    // 退出被动状态（一般空的）
}
}