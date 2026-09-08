#include "arm_control_execute.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_databoard.h"
#include "pyro_typedef.h"
#include "pyro_algo_common.h"

extern "C" void engineer_arm_mission(void* args);
extern pyro::databoard global_databoard;
PYRO_ArmControlMode_t control_mode = PYRO_ARM_MODE_JOINT_DIRECT;

namespace pyro
{
float axis_current_pos[7];
float motor_current_current[7];
float motor_current_pos[7];
float motor_current_rot[7];
float motor_current_torque[7];
float axis_target_pos[7] = {80,80,80,80,80,80,-1.5f};
uint32_t axis_current_pos_id[7];
uint32_t axis_target_pos_id[7];
uint32_t control_mode_id;

Axis_control_t::Axis_control_t(motor_base_t *motor, pid_t *pos_pid, pid_t *rot_pid) : _motor(motor), _pos_pid(pos_pid), _rot_pid(rot_pid)
{
    _target_position = 0.0f;
    _target_rotate   = 0.0f;
    axis_limit      = CONSTRAINT;
}
Axis_control_t::~Axis_control_t(){}

void Axis_control_t::set_target(float position)
{
    if(axis_limit == NO_CONSTRAINT)
    {
        if(position < -PI)
        {
            position = position - 2 * PI;
        }
        else if(position > PI)
        {
            position = position + 2 * PI;
        }
    }
    else
    {
        if(position < _lower_limit)
        {
            position = _lower_limit;
        }
        else if(position > _upper_limit)
        {
            position = _upper_limit;
        }
    }
    _target_position = position;
}

void Axis_control_t::update_feedback()
{
    _motor->update_feedback();
    _feedback_position = _motor->get_current_position() - _pos_offset;
    _feedback_rotate   = _motor->get_current_rotate();
    _feedback_position = _feedback_position;
    _feedback_position = loop_fp32_constrain(_feedback_position, -PI, PI);
}

//target归一化，feedback归一化，考虑无限位时转动不要超过一圈，pid计算
void Axis_control_t::pid_control()
{
    if(axis_limit == NO_CONSTRAINT)
    {
        if(_target_position - _feedback_position > PI)
        {
            _target_position -= 2 * PI;
        }
        else if(_target_position - _feedback_position < -PI)
        {
            _target_position += 2 * PI;
        }
    }
    float rot = _pos_pid->calculate(_target_position, _feedback_position);
    float torque = _rot_pid->calculate(rot, _feedback_rotate);
    _motor->send_torque(torque);
}

dm_motor_drv_t *axis_motor[6] = {
    new dm_motor_drv_t(0x2, 0x1, bsp_can::can1),
    new dm_motor_drv_t(0x4, 0x3, bsp_can::can1),
    new dm_motor_drv_t(0x6, 0x5, bsp_can::can3),
    new dm_motor_drv_t(0x8, 0x7, bsp_can::can3),
    new dm_motor_drv_t(0xa, 0x9, bsp_can::can2),
    new dm_motor_drv_t(0xc, 0xb, bsp_can::can2)
};
dm_motor_drv_t *end_motor = new dm_motor_drv_t(0xe, 0xd, bsp_can::can2);

pid_t *axis_pos_pid[6] = {
    new pid_t(10,0.0,0.0,0.0,52),
    new pid_t(20,0,0.0,20.0,150),
    new pid_t(15,0.0,0.0,0.0,160),
    new pid_t(15.7,0,0.0,6,200),
    new pid_t(10.3,0.1,0.0,20,200),
    new pid_t(9,0.0,0.0,0.0,200)
};
pid_t *axis_rot_pid[6] = {
    new pid_t(8.8,12.0,0.0,0.0,27),
    new pid_t(20,0,0.0,20.0,150),
    new pid_t(11,0.2,0.0,5.0,40),
    new pid_t(1.0,0.2,0.001,3,7),
    new pid_t(1.0,0.01,0.00,4,7),
    new pid_t(0.8,0.1,0.0,1,7)
};
pid_t *end_pos_pid = new pid_t(9,0.0,0.0,0,200);
pid_t *end_rot_pid = new pid_t(0.8,0.0,0.0,0,7);

Axis_control_t *axis_control[6] = {
    new Axis_control_t(axis_motor[0], axis_pos_pid[0], axis_rot_pid[0]),
    new Axis_control_t(axis_motor[1], axis_pos_pid[1], axis_rot_pid[1]),
    new Axis_control_t(axis_motor[2], axis_pos_pid[2], axis_rot_pid[2]),
    new Axis_control_t(axis_motor[3], axis_pos_pid[3], axis_rot_pid[3]),
    new Axis_control_t(axis_motor[4], axis_pos_pid[4], axis_rot_pid[4]),
    new Axis_control_t(axis_motor[5], axis_pos_pid[5], axis_rot_pid[5])
};
Axis_control_t *end_axis = new Axis_control_t(end_motor, end_pos_pid, end_rot_pid);

void arm_control_init(float (*position_range)[2], 
                      float (*rotate_range)[2], 
                      float (*torque_range)[2]
                    )
{
    for(int i = 0; i < 6; i++)
    {
        axis_motor[i]->set_position_range(position_range[i][0], position_range[i][1]);
        axis_motor[i]->set_rotate_range(rotate_range[i][0], rotate_range[i][1]);
        axis_motor[i]->set_torque_range(torque_range[i][0], torque_range[i][1]);
    }
    end_motor->set_position_range(-PI, PI);
    end_motor->set_rotate_range(-200, 200); 
    end_motor->set_torque_range(-7, 7);

    axis_control[0]->set_limit(-2.1599, PI);
    axis_control[1]->set_limit(-0.3, 1);
    axis_control[2]->set_limit(-0.3, 1.35);
    axis_control[3]->disable_constraint(); // R4 is a free joint, no limit
    axis_control[4]->set_limit(-0.5f*PI, 0.5f*PI);
    axis_control[5]->set_limit(-0.5f*PI, 0.5f*PI);
    end_axis->set_limit(-PI, 0);


    axis_control[0]->set_feedback_pos_offset(0.65045);
    axis_control[1]->set_feedback_pos_offset(-2.3948);
    axis_control[2]->set_feedback_pos_offset(-0.328); 
    axis_control[3]->set_feedback_pos_offset(2.34739);
    axis_control[4]->set_feedback_pos_offset(2.46375);
    axis_control[5]->set_feedback_pos_offset(2.74246);
    end_axis->set_feedback_pos_offset(1.8587);
    //设置限位关节的限位值
}

//轴是控制量，进行pid的控制，控制motor
void engineer_arm_update()
{
    for(int i = 0; i < 6; i++)
    {
        axis_control[i]->update_feedback();
        axis_control[i]->get_current_position(axis_current_pos[i]);
        motor_current_pos[i] = axis_motor[i]->get_current_position();
        motor_current_rot[i] = axis_motor[i]->get_current_rotate();
        motor_current_torque[i] = axis_motor[i]->get_current_torque();
    }
    end_axis -> update_feedback();
    end_axis->get_current_position(axis_current_pos[6]);
    motor_current_pos[6] = end_motor->get_current_position();
    motor_current_rot[6] = end_motor->get_current_rotate();
    motor_current_torque[6] = end_motor->get_current_torque();

    global_databoard.write_topic(axis_current_pos_id[0],
            *((pyro::genenral_data_t*)&(axis_current_pos[0])));
    global_databoard.write_topic(axis_current_pos_id[1],
            *((pyro::genenral_data_t*)&(axis_current_pos[1])));
    global_databoard.write_topic(axis_current_pos_id[2],
            *((pyro::genenral_data_t*)&(axis_current_pos[2])));
    global_databoard.write_topic(axis_current_pos_id[3],
            *((pyro::genenral_data_t*)&(axis_current_pos[3])));
    global_databoard.write_topic(axis_current_pos_id[4],
            *((pyro::genenral_data_t*)&(axis_current_pos[4])));
    global_databoard.write_topic(axis_current_pos_id[5],
            *((pyro::genenral_data_t*)&(axis_current_pos[5])));
    global_databoard.write_topic(axis_current_pos_id[6],
            *((pyro::genenral_data_t*)&(axis_current_pos[6])));
    //将旋转轴当前逻辑位置拷贝到规划线程中的参数(当前逻辑位置)中，方便对机械臂进行规划
    //进行进程间的较大量，实时性要求较高的数据交互使用互斥锁
    //xSemaphoreTake(arm_control_mutex, portMAX_DELAY);
    //获取锁
    // memcpy(control_target_param->axis_current_pos,axis_current_pos,sizeof(float)*6);
    //xSemaphoreGive(arm_control_mutex);
    //释放锁
}

void engineer_arm_set_target()
{
    static TickType_t timestamp;
    global_databoard.read(axis_target_pos_id[0], (pyro::genenral_data_t*)&(axis_target_pos[0]), timestamp);
    global_databoard.read(axis_target_pos_id[1], (pyro::genenral_data_t*)&(axis_target_pos[1]), timestamp);
    global_databoard.read(axis_target_pos_id[2], (pyro::genenral_data_t*)&(axis_target_pos[2]), timestamp);
    global_databoard.read(axis_target_pos_id[3], (pyro::genenral_data_t*)&(axis_target_pos[3]), timestamp);
    global_databoard.read(axis_target_pos_id[4], (pyro::genenral_data_t*)&(axis_target_pos[4]), timestamp);
    global_databoard.read(axis_target_pos_id[5], (pyro::genenral_data_t*)&(axis_target_pos[5]), timestamp);
    global_databoard.read(axis_target_pos_id[6], (pyro::genenral_data_t*)&(axis_target_pos[6]), timestamp);

    for(int i=0; i<6; i++)
    {
        if(fabs(axis_target_pos[i])>=64)  
        {
            control_mode = PYRO_ARM_MODE_IDLE;
            return;
        }
        else{
            control_mode = PYRO_ARM_MODE_JOINT_DIRECT;
            axis_control[i]->set_target(axis_target_pos[i]);
        }
    }
    if(fabs(axis_target_pos[6])>=64)  
        {
            control_mode = PYRO_ARM_MODE_IDLE;
            return;
        }
        else{
            control_mode = PYRO_ARM_MODE_JOINT_DIRECT;
            end_axis->set_target(axis_target_pos[6]);
        }
}

void arm_control()
{
    for(int i = 0; i < 6; i++)
    {
        axis_control[i]->pid_control();
    }
    end_axis -> pid_control();
}

void engineer_arm_zeroforce()
{
    for(int i = 0; i < 6; i++)
    {
        axis_motor[i]->send_torque(0.0f);
    }
    end_motor -> send_torque(0.0f);
}


}

float position_range[6][2] = {{-pyro::PI, pyro::PI}, {-pyro::PI, pyro::PI}, {-pyro::PI, pyro::PI}, {-pyro::PI, pyro::PI}, {-pyro::PI, pyro::PI}, {-pyro::PI, pyro::PI}};
float rotate_range[6][2]   = {{-52, 52}, {-150, 150}, {-160, 160}, {-200, 200}, {-200, 200}, {-200, 200}};
float torque_range[6][2]   = {{-27, 27}, {-150, 150}, {-40, 40}, {-7, 7}, {-7, 7}, {-7, 7}};

extern "C" void engineer_arm_init(void* args)
{
    osDelay(10);

    pyro::arm_control_init(position_range, rotate_range, torque_range);

    pyro::axis_target_pos_id[0] = global_databoard.get_topic_id("arm_command_joint0");
    pyro::axis_target_pos_id[1] = global_databoard.get_topic_id("arm_command_joint1");
    pyro::axis_target_pos_id[2] = global_databoard.get_topic_id("arm_command_joint2");
    pyro::axis_target_pos_id[3] = global_databoard.get_topic_id("arm_command_joint3");
    pyro::axis_target_pos_id[4] = global_databoard.get_topic_id("arm_command_joint4");
    pyro::axis_target_pos_id[5] = global_databoard.get_topic_id("arm_command_joint5");
    pyro::axis_target_pos_id[6] = global_databoard.get_topic_id("arm_command_gripper");
    //获取旋转轴当前逻辑位置的topic id
    pyro::axis_current_pos_id[0] = global_databoard.get_topic_id("axis1_current_pos");
    pyro::axis_current_pos_id[1] = global_databoard.get_topic_id("axis2_current_pos");
    pyro::axis_current_pos_id[2] = global_databoard.get_topic_id("axis3_current_pos");
    pyro::axis_current_pos_id[3] = global_databoard.get_topic_id("axis4_current_pos");
    pyro::axis_current_pos_id[4] = global_databoard.get_topic_id("axis5_current_pos");
    pyro::axis_current_pos_id[5] = global_databoard.get_topic_id("axis6_current_pos");
    pyro::axis_current_pos_id[6] = global_databoard.get_topic_id("gripper_current_pos");

    //固定延时，等达妙电机上电完毕之后再对电机进行使能
    vTaskDelay(1500);

    pyro::axis_motor[0]->enable();
    vTaskDelay(1);
    pyro::axis_motor[1]->enable();
    vTaskDelay(1);
    pyro::axis_motor[2]->enable();
    vTaskDelay(1);
    pyro::axis_motor[3]->enable();
    vTaskDelay(1);
    pyro::axis_motor[4]->enable();
    vTaskDelay(1);
    pyro::axis_motor[5]->enable();
    vTaskDelay(1);
    pyro::end_motor->enable();
    vTaskDelay(1);

    //轴1的pid再调一调
    xTaskCreate(engineer_arm_mission, "engineer_arm_mission", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);

    vTaskDelete(nullptr);
}

extern "C" void engineer_arm_mission(void* args)
{
    for(;;)
    {
        pyro::engineer_arm_update();            //更新控制量和pid反馈
        pyro::engineer_arm_set_target();
        if(control_mode==PYRO_ARM_MODE_IDLE)
        {
            pyro::engineer_arm_zeroforce();
        }
        else if(control_mode==PYRO_ARM_MODE_JOINT_DIRECT)
        {
            pyro::arm_control();
        }
        vTaskDelay(1);
    }
}
