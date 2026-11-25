#include "heat_control.h"
#include "pyro_dwt_drv.h"

using namespace pyro;

HeatControl::HeatControl()
{
    dwtCnt = 0;
    dwt_drv_t::get_delta_t(&dwtCnt);
    current_heat = 0;
    heat_limit = 40;
    p = 0;
    cooling_rate = 0;
}


bool HeatControl::CanShootSingle()
{
    if(heat_limit - current_heat < SAFE_BULLET_NUM * HEAT_PER_BULLET)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void HeatControl::LocalShootAdd()
{
    current_heat += 10;
    shoot_event_timestamp_buffer[p] = dwt_drv_t::get_timeline_ms();
    p++;
    p %= BUFFER_STACK_LENGTH;
}



void HeatControl::LocalHeatCompute()
{
    float dt = pyro::dwt_drv_t::get_delta_t(&dwtCnt);
    current_heat -= dt * cooling_rate;
    if(current_heat < 0.0f)
    {
        current_heat = 0.0f;
    }
}


void HeatControl::UpdateParameter(int heat_limit, int cooling_rate)
{
    this->heat_limit = heat_limit;
    this->cooling_rate = cooling_rate;
}


void HeatControl::UpdateRefereeHeat(int heat)
{
    int temp_p = (p - 1 + BUFFER_STACK_LENGTH) % BUFFER_STACK_LENGTH;
    float current_time = dwt_drv_t::get_timeline_ms();
    int reserve_bullet_num = 0;
    while((current_time - shoot_event_timestamp_buffer[temp_p]) <= (float)CHECKING_TIME)
    {
        reserve_bullet_num ++;
        temp_p--;
        if(temp_p < 0)
        {
            temp_p += BUFFER_STACK_LENGTH;
        }
        if(temp_p == p)
        {
            //已经回绕一整圈了。直接退出。
            break;
        }
    }
    float max_heat = heat + reserve_bullet_num * HEAT_PER_BULLET;
    if(current_heat <= heat)
    {
        //此时直接取裁判系统的热量
        current_heat = heat;
    }
    else
    {
        current_heat = (max_heat) > current_heat ? max_heat : current_heat;
    }

}
