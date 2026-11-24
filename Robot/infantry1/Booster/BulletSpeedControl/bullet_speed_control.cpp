#include "bullet_speed_control.h"

using namespace pyro;


BulletSpeedControl::BulletSpeedControl(float pid_kp, float pid_ki, float pid_kd)
        : compensation_pid(pid_kp, pid_ki, pid_kd, 100.0f, 100.0f)
{
    for(int i = 0; i < BUFFER_NUM; i++)
    {
        speed_buffer[i] = 0.0f;
    }
    compensation_pid.clear();
}


void BulletSpeedControl::UpdateNewShootSpeed(float speed)
{
    if(speed < VALID_MIN_SPEED)
    {
        return;
    }
    for(int i = 0; i < BUFFER_NUM - 1; i++)
    {
        speed_buffer[i] = speed_buffer[i + 1];
    }
    speed_buffer[BUFFER_NUM - 1] = speed;
}

void BulletSpeedControl::CalculateSpeedCompensation()
{
    //计算平均弹速
    float average_speed = 0.0f;
    uint8_t valid_num = 0;
    for(int i = 0; i < BUFFER_NUM; i++)
    {
        if(speed_buffer[i] >= VALID_MIN_SPEED)
        {
            average_speed += speed_buffer[i];
            valid_num++;
        }
    }
    if(valid_num == 0)
    {
        final_compensation  = 0.0f;
    }
    else
    {
        average_speed /= valid_num;
        final_compensation = compensation_pid.calculate(PROJECTILE_TARGET_MUZZLE_VELOCITY, average_speed);
    }
}


float BulletSpeedControl::GetSpeedCompensation()
{
    return final_compensation;
}


void BulletSpeedControl::Reset()
{
    for(int i = 0; i < BUFFER_NUM; i++)
    {
        speed_buffer[i] = 0.0f;
    }
    compensation_pid.clear();
    final_compensation = 0;
}