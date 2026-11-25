#include "pyro_wl_booster.h"

void pyro::wl_booster_t::fsm_active_t::on_enter(owner* owner)
{

    change_state(&owner->_state_active._state_spin_up);
}

void pyro::wl_booster_t::fsm_active_t::on_execute(owner* owner)
{

}

void pyro::wl_booster_t::fsm_active_t::on_exit(owner* owner)
{

}