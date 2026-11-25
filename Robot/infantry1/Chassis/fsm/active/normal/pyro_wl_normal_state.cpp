#include "pyro_algo_common.h"
#include "pyro_wl_chassis.h"

#include <algorithm>
#include <cmath>


namespace pyro
{



void wl_chassis_t::fsm_active_t::state_normal_t::on_enter(wl_chassis_t *owner)
{
    // 这里切入自救态分为两种状况：
    // 1.车子翻到后，我没有去下力，而是直接拨杆触发自救
    // 2.车子翻到后，我下力了，但是我没有去把它摆正，直接上力了，此时我需要让它重新上力时检测一下需不需要自救

    // 判断是否进入自救态
    if (owner->_current_cmd.cmd_function_state == chassis_function_state_t::RESCUE ||
        abs(owner->_ctx.data.ins.euler_rad[1]) >= PI / 4.0f ||
        abs(owner->_ctx.data.ins.euler_rad[2]) >= PI / 4.0f)
    {
        change_state(&_state_rescue);
    }
    else
    {
        change_state(&_state_align);
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::on_execute(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    //判断是否下达了功能状态的指令
    static chassis_function_state_t last_function = chassis_function_state_t::NONE;
    if(owner->_current_cmd.cmd_function_state != last_function)
    {
        data.current_function = owner->_current_cmd.cmd_function_state;
    }
    else
    {
        data.current_function = chassis_function_state_t::NONE;
    }
    last_function = owner->_current_cmd.cmd_function_state;
    // 连跳期间屏蔽其他功能切换，距离变化也不重新使能触发。
    if (_active_state == &_state_jump_duo)
    {
        return;
    }

    const auto distance_front = data.distance_front;
    const bool jump_duo_enabled = owner->_current_cmd.jump_duo_enabled;
    if (!jump_duo_enabled || distance_front >= JUMP_DUO_TRIGGER_DISTANCE_MM)
    {
        _jump_duo_triggered = false;
    }

    // Jump is atomic; do not let STEP/SPIN replace a jump phase.
    if (_active_state == &_state_jump)
    {
        return;
    }

    if(data.current_function == chassis_function_state_t::STEP)
    {
        change_state(&_state_step);
    }
    else if(data.current_function == chassis_function_state_t::JUMP)
    {
        data.jump_trigger_source = jump_trigger_source_t::SINGLE_COMMAND;
        data.jump_trigger_distance_front = distance_front;
        change_state(&_state_jump);
    }
    else if(data.current_function == chassis_function_state_t::SPIN_TOGGLE)
    {
        change_state(&_state_spin);
    }
    else if (_active_state == &_state_balance && jump_duo_enabled &&
             !_jump_duo_triggered && distance_front > 0U &&
             distance_front < JUMP_DUO_TRIGGER_DISTANCE_MM)
    {
        _jump_duo_triggered = true;
        data.jump_trigger_source = jump_trigger_source_t::FRONT_DISTANCE;
        data.jump_trigger_distance_front = distance_front;
        change_state(&_state_jump_duo);
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::on_exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
