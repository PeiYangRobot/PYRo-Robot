#include "pyro_wl_gimbal.h"
#include "gimbal_config.h"




void pyro::wl_gimbal_t::fsm_active_t::state_align_t::enter(owner *owner)
{
    owner->_module_deps.pid_deps.yaw_pos->clear();
    owner->_module_deps.pid_deps.yaw_spd->clear();
    owner->_ctx.data.output.pitchEn    = true;
    owner->_ctx.data.output.yawEn      = true;
    owner->_ctx.data.output.yawCurrent = 0.0f;
    instance()->set_pitchstate(owner->_ctx.data.output.pitchEn);
    instance()->set_yawstate(owner->_ctx.data.output.yawEn);

}

void pyro::wl_gimbal_t::fsm_active_t::state_align_t::execute(owner *owner)
{
    //判断底盘是否在自救状态，如果在自救状态，则可选择正前或者正反方向进行就近复位，不管pitch轴

    if(owner->_ctx.data.chassis_is_rescuing)
    {
        //判断最短路径
        float shortest_path = owner->_ctx.data.state.yaw.pos - YAW_ALIGN_TARGET_RAD;
        while(shortest_path > PI)
        {
            shortest_path -= 2*PI;
        }
        while(shortest_path < -PI)
        {
            shortest_path += 2*PI;
        }
        //判断是到PI还是0
        if(abs(shortest_path) > PI/2.0f)
        {
            owner->align_updateYaw(PI);
        }
        else
        {
            owner->align_updateYaw(0.0f);
        }


        owner->align_updateYaw(PI);
    }
    else
    {
        owner->align_updatePitch();
        if(owner->_ctx.data.state.pitch.pos<PITCH_LIMIT_MAX+0.4f)
        {
            float error_rad = owner->_ctx.data.state.yaw.pos - YAW_ALIGN_TARGET_RAD;
            owner->align_updateYaw(0.0f);

            static int count = 0;
            if(fabs(error_rad) < 0.1f)
            {
                if(count >= 50)
                {
                    if(owner->_ctx.data.chassis_is_ready)
                    {
                        request_switch(&owner->_state_active._state_manual);
                    }
                }
                count++;
            }
            else
            {
                count = 0;
            }
        }
    }


}

void pyro::wl_gimbal_t::fsm_active_t::state_align_t::exit(owner *owner)
{
    (void)owner;
}