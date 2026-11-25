#include "stdint.h"
#include "booster_config.h"
#include "pyro_dwt_drv.h"



namespace pyro
{


class HeatControl
{
  public:
    HeatControl();
    // 本地冷却推演
    void LocalHeatCompute();
    // 更新裁判系统发来的数据
    void UpdateParameter(int heat_limit, int cooling_rate);
    void UpdateRefereeHeat(int heat);
    // 是否可以打弹
    bool CanShootSingle();
    // 本地检测到发弹后调用此函数，热量自增
    void LocalShootAdd();


  private:
    struct shoot_event
    {
        int time_stamp;
    };

    static constexpr int HEAT_PER_BULLET = 10; //每颗子弹增加的热量
    static constexpr int BUFFER_STACK_LENGTH = 20; // 环形缓冲区大小
    static constexpr int CHECKING_TIME = 500; //更新冷却值时保留最近的Xms内的发弹事件作为余量
    //建立环形缓冲区，用来储存最近的发弹事件。
    float shoot_event_timestamp_buffer[BUFFER_STACK_LENGTH] = {0};
    uint8_t p;

    //用于热量推算更新时获取dt
    uint32_t dwtCnt;

    float current_heat; // 本地保存的当前热量
    int heat_limit;     // 当前热量上限
    int cooling_rate;   // 当前冷却速率


};


}
