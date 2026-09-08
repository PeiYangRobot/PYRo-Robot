#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"

#include "pyro_sentry_booster.h"
#include "booster_config.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_motor_base.h"
#include "pyro_bsp_can.h"
#include "pyro_can_drv.h"

using namespace pyro;

// 定义任务通知的位掩码 (Event Bits)
constexpr uint32_t EVENT_BIT_FRIC_ON = (1 << 0);
constexpr uint32_t EVENT_BIT_SINGGLE_SHOOT   = (1 << 1);
constexpr uint32_t EVENT_BIT_FIRE = (1 << 2); 

static TaskHandle_t booster_task_handle                          = nullptr;
static pyro::sentry_booster_t *sentry_booster_ptr                = nullptr;
static pyro::sentry_booster_cmd_t *booster_cmd_ptr               = nullptr;
static pyro::sentry_booster_deps_t *booster_deps_ptr             = nullptr;




extern "C" {
static void deps_init();
static void booster_dr162cmd(uint32_t notify_value);


void deps_init()
    {
        booster_deps_ptr = new pyro::sentry_booster_deps_t();

        
        booster_deps_ptr->motor_deps.fric_left = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,
                                        pyro::bsp_can::can1); 
        booster_deps_ptr->motor_deps.fric_right = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,
                                        pyro::bsp_can::can1);
        booster_deps_ptr->motor_deps.trigger = new pyro::dji_m2006_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                        pyro::bsp_can::can1);





        booster_deps_ptr->pid_deps.l_fric_pid = 
                                new pyro::pid_t(0.05f, 0.0f, 0.0f, 0.8f, 20.0f);
        booster_deps_ptr->pid_deps.r_fric_pid =  
                                new pyro::pid_t(0.05f, 0.0f, 0.0f, 0.8f, 20.0f);
        booster_deps_ptr->pid_deps.trig_pos_pid = 
                                new pyro::pid_t(250.0f     -1  , 1.0f, 0.00f, 0.00f, 250.0f);
        booster_deps_ptr->pid_deps.trig_spd_pid = 
                                new pyro::pid_t(0.065f  , 0.05f, 0.0002f, 0.09f, 20.0f);

    }
    


void booster_dr162cmd(uint32_t notify_value)
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    static uint32_t multi_count = 0;

    if (pyro::sw_pos_t::UP == vrc.switches.right.current_pos)
    {
        booster_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        booster_cmd_ptr->fric_on = 0;
        booster_cmd_ptr->fire_count   = 0;
        booster_cmd_ptr->real_hit_speed   = 0;
        booster_cmd_ptr->multi_shoot   =  0;
        
        return;
    }
    
    else{
        booster_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::ACTIVE;
        ;
        if(notify_value & EVENT_BIT_FRIC_ON)
        {   
            booster_cmd_ptr->fric_on                =  !booster_cmd_ptr->fric_on;
            

           //booster_cmd_ptr->trig_target_spd        =  BOOSTER_SHOOT_TRIG_SPEED;

        }
        booster_cmd_ptr->multi_shoot        =  0;
        
    if(booster_cmd_ptr->fric_on){
        if(pyro::sw_pos_t::DOWN == vrc.switches.left.current_pos){
            
            
            if(multi_count > 1000){
            booster_cmd_ptr->multi_shoot          =  true;
            }
            else{
                
                multi_count ++;
            }
        }
        if(notify_value & EVENT_BIT_FIRE)
        {   
            //booster_cmd_ptr->singgle_shoot          =  !booster_cmd_ptr->singgle_shoot;
            booster_cmd_ptr->multi_shoot          =  false;
            multi_count = 0;
            
        }
        

        if(notify_value & EVENT_BIT_SINGGLE_SHOOT)
        {   
            //booster_cmd_ptr->singgle_shoot          =  !booster_cmd_ptr->singgle_shoot;
            
            booster_cmd_ptr->fire_count ++;
            booster_cmd_ptr->multi_shoot        =  0;
        }
    }


    }

}
    void sentry_booster_thread(void *argument)
    {
        while (true)
        {   
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);


            //chassis_rxcmd();
            // 如果后续希望由底盘板直接解算 RC，可以取消下面这行的注释
             booster_dr162cmd(notify_val);
            // booster_dr162chassis_cmd();

            
            sentry_booster_ptr->set_command(*booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void sentry_booster_init(void)
    {
        //========版间通信订阅========
        //pyro::can_rx_drv_t::subscribe(pyro::can_hub_t::which_can::can3, 0x103);
        

        
        booster_cmd_ptr     = new pyro::sentry_booster_cmd_t();
        sentry_booster_ptr = pyro::sentry_booster_t::instance();

        

        deps_init();
        sentry_booster_ptr->configure(*booster_deps_ptr);
        sentry_booster_ptr->start();


        xTaskCreate(sentry_booster_thread, "start_sentry_booster_thread", 256,
                    nullptr, configMAX_PRIORITIES - 1, &booster_task_handle);
            // --- DR16 拨杆绑定 ---
        auto &vrc = pyro::rc_drv_t::read();
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::UP_TO_MID, booster_task_handle, EVENT_BIT_FRIC_ON);
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::DOWN_TO_MID, booster_task_handle, EVENT_BIT_SINGGLE_SHOOT);
        pyro::sw_broker::subscribe(&vrc.switches.left, pyro::sw_event_t::MID_TO_DOWN, booster_task_handle, EVENT_BIT_FIRE);

        vTaskDelete(nullptr);
    }






}
