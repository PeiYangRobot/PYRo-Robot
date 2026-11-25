#include "pyro_wl_chassis.h"

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_duo_t::enter(wl_chassis_t *owner)
{
    _second_jump = false;
    state_jump_t::enter(owner);
}

bool wl_chassis_t::fsm_active_t::state_normal_t::state_jump_duo_t::handle_landing(wl_chassis_t *owner)
{
    // 下向测距还没有收到有效读数时，继续等待落地。
    if (owner->_ctx.data.distance_down == 0U)
    {
        return false;
    }

    if (_second_jump)
    {
        return state_jump_t::handle_landing(owner);
    }

    // 第一次落地直接开始第二次预压缩，重新采样航向和前进速度。
    _second_jump = true;
    start_jump(owner);
    return true;
}

} // namespace pyro
