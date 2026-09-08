#include "pyro_sentry_booster.h"

#include "booster_config.h"
#include "pyro_algo_common.h"
#include <arm_math.h>
int lspd =0;
int rspd =0;

float pos1 =0.0f;
float pos2 =0.0f;

float spd1 =0.0f;
float spd2 =0.0f;

float pos4 =0.0f;
float pos5 =0.0f;

float pos_error =0.0f;

bool ad = false;
int8_t error_count1 = 0;
int8_t count1 = 0;
namespace pyro{


    sentry_booster_t::sentry_booster_t():module_base_t("sentry_booster"){
        _ctx={};
    }
;
    status_t sentry_booster_t::_init(){
        _ctx.motor=_module_deps.motor_deps;
        _ctx.pid=_module_deps.pid_deps;


        return PYRO_OK;
    }

    void sentry_booster_t::_update_feedback(){
        _ctx.motor.fric_left->update_feedback();
        _ctx.motor.fric_right->update_feedback();
        _ctx.motor.trigger->update_feedback();

        static float last_rotate = 0.0f;
        static float last_position = 0.0f;

        _ctx.data.current_data.left_spd=
                    _ctx.motor.fric_left->get_current_rotate();
        _ctx.data.current_data.right_spd=
                    _ctx.motor.fric_right->get_current_rotate();
        
        _ctx.data.current_data.left_torque=
                    _ctx.motor.fric_left->get_current_torque();
        _ctx.data.current_data.right_torque=
                    _ctx.motor.fric_right->get_current_torque();

        _ctx.data.current_data.trigger_spd=
                    _ctx.motor.trigger->get_current_rotate();
        _ctx.data.current_data.trigger_torque=
                    _ctx.motor.trigger->get_current_torque();
        _ctx.data.current_data.trigger_pos=
                    _ctx.motor.trigger->get_current_position();

        //正转
        float error_angle =  _ctx.data.current_data.trigger_pos - last_position;
        if(_ctx.data.current_data.trigger_spd > 1.0f && last_rotate > 1.0f){
            
            if (error_angle > 0.1f)
            {
                /* code */
            }
            else if (error_angle < -3.0f)
            {
                pos4 = _ctx.data.current_data.trigger_pos;
                pos5 = last_position;
                spd1 = _ctx.data.current_data.trigger_spd;
                spd2 = last_rotate;
                pos_error = error_angle ;

               _ctx.data.current_data.trigger_count++; /* code */
            }
        }
        else if(_ctx.data.current_data.trigger_spd < -1.0f && last_rotate < -1.0f){
            
            if (error_angle > 3.0f)
            {
               _ctx.data.current_data.trigger_count--;
            }
            else
            {
              
            }
        }
        else if((_ctx.data.current_data.trigger_spd < 0 && last_rotate > 0)||(_ctx.data.current_data.trigger_spd > 0 && last_rotate < 0)){
            if(fabsf(error_angle) >3.0f )
            _ctx.data.error_count++;
        }

        if(_ctx.data.current_data.trigger_count > 36){
            _ctx.data.current_data.trigger_count = 0;
        }
        if(_ctx.data.current_data.trigger_count < 0){
            _ctx.data.current_data.trigger_count = 36;
        }



        _ctx.data.current_data.trigger_equal_pos=((_ctx.data.current_data.trigger_count*2*PI)+_ctx.data.current_data.trigger_pos)/36.0f;

        _ctx.data.current_data.trigger_equal_pos = loop_fp32_constrain(_ctx.data.current_data.trigger_equal_pos,-PI, PI); 

        last_rotate = _ctx.data.current_data.trigger_spd;
        last_position = _ctx.data.current_data.trigger_pos;

       pos1 = _ctx.data.current_data.trigger_equal_pos;
       pos2 = _ctx.data.target_data.trigger_equal_pos;
    count1 = _ctx.data.current_data.trigger_count;
    error_count1 = _ctx.data.error_count;
    //---暂不滤波---
    // _ctx.imu_data.current_pitch_rad = raw_pitch;
    // _ctx.imu_data.current_roll_rad  = raw_roll;    
    // _ctx.imu_data.current_yaw_rad = raw_yaw;
    }

// void sentry_booster_t::_solve(){


//     _ctx.target_data.left_spd=_ctx.cmd->fric_target_speed;
//     _ctx.target_data.right_spd=-_ctx.cmd->fric_target_speed;


//     //_ctx.target_data.trigger_spd=_ctx.cmd->trig_target_spd;

// }

void sentry_booster_t::_fric_control(){
    if(_ctx.cmd->fric_on){
        _ctx.data.target_data.left_spd =       BOOSTER_SHOOT_FRIC_RADPS;//有裁判系统后速度换成计算值
        _ctx.data.target_data.right_spd =    - BOOSTER_SHOOT_FRIC_RADPS;
    }
    else{
        _ctx.data.target_data.left_spd   =       0.0f;//有裁判系统后速度换成计算值
        _ctx.data.target_data.right_spd  =       0.0f;
    }
    // _ctx.target_data.left_spd=_ctx.cmd->fric_target_speed;
    // _ctx.target_data.right_spd=-_ctx.cmd->fric_target_speed;
    
    
    _ctx.data.out_data.l_fric_torque=_ctx.pid.l_fric_pid
                        ->calculate(_ctx.data.target_data.left_spd,
                                    _ctx.data.current_data.left_spd);
    _ctx.data.out_data.r_fric_torque=_ctx.pid.r_fric_pid
                        ->calculate(_ctx.data.target_data.right_spd,
                                    _ctx.data.current_data.right_spd);


    // if(!_ctx.cmd->fric_on){ 
        
    //     _ctx.out_data.l_fric_torque=0.0f;
    //     _ctx.out_data.r_fric_torque=0.0f;

    // }
    
}

void sentry_booster_t::_trigger_control(){


    lspd =  _ctx.cmd->fire_count;
    rspd =  _ctx.data.current_fire_count;
    

if(_ctx.cmd->fric_on){

    if(_ctx.cmd->multi_shoot == true){
        float distance =loop_fp32_constrain(_ctx.data.target_data.trigger_equal_pos - _ctx.data.current_data.trigger_equal_pos, -PI,PI) ;
        
         if(distance <PI/2){
        _ctx.data.target_data.trigger_equal_pos += PI/4;
        }
    }    
    else if(_ctx.cmd->fire_count  >  _ctx.data.current_fire_count){
        _ctx.data.target_data.trigger_equal_pos += PI/4;
        _ctx.data.current_fire_count++;
    }
    
}
    _ctx.data.target_data.trigger_equal_pos = loop_fp32_constrain(_ctx.data.target_data.trigger_equal_pos, -PI, PI);
    float error_trigger_pos    = 
                    loop_fp32_constrain(_ctx.data.target_data.trigger_equal_pos - _ctx.data.current_data.trigger_equal_pos,
                                         -PI, PI);

    float out_trigger_spd       = _ctx.pid.trig_pos_pid
                        ->calculate(error_trigger_pos,0.0f);

    if(_ctx.cmd->fric_on && _ctx.cmd->multi_shoot == true){

        out_trigger_spd = 12*BOOSTER_TRIGGER_RATE*BOOSTER_ONE_SHOOT_ANGLE;

    }
    _ctx.data.out_data.trigger_torque= _ctx.pid.trig_spd_pid
                        ->calculate(out_trigger_spd,
                                    _ctx.data.current_data.trigger_spd);
                                
        
    }

void sentry_booster_t::_send_fric_command() const{ 
    _ctx.motor.fric_left->send_torque(_ctx.data.out_data.l_fric_torque);
    _ctx.motor.fric_right->send_torque(_ctx.data.out_data.r_fric_torque);

    }
void sentry_booster_t::_send_trig_command() const{
    _ctx.motor.trigger->send_torque(_ctx.data.out_data.trigger_torque);

}
void sentry_booster_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;
    if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);
    else if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}

} // namespace pyro
