#include "pyro_algo_pid.h"
#include "stdint.h"
#include "booster_config.h"




namespace pyro
{

class BulletSpeedControl
{
  public:
    BulletSpeedControl(float pid_kp, float pid_ki, float pid_kd);

    // 更新最新的弹速，会自动屏蔽掉异常的数据
    void UpdateNewShootSpeed(float speed);

    // 计算最新的弹速补偿量，直接加在目标弹速上
    void CalculateSpeedCompensation();
    // 返回弹速补偿
    float GetSpeedCompensation();
    // 重置
    void Reset();

  private:
    static constexpr uint8_t BUFFER_NUM = 8;
    pid_t compensation_pid;
    float speed_buffer[BUFFER_NUM];
    float final_compensation = 0;

};

}


/*
弹速闭环控制器思路
target: 对外界提供输入弹速的函数以及补偿目标输出的函数

需要的类内成员：
    一个数组，先进先出，用于储存最近几次发弹的速度
    一个pid，用来计算输出
*/
