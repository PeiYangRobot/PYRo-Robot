#ifndef PYRO_BOARD_COMM_CONFIG_H
#define PYRO_BOARD_COMM_CONFIG_H

// =========================================================
// 版间 CAN 通信「失联超时保护」配置
//
// 每个模块一个独立开关，方便分别调试：
//   - *_ENABLE = 1   启用该消息的失联保护
//   - *_ENABLE = 0   关闭（失联后保持最后一帧值，即旧行为）
//   - *_MS           失联判定阈值（毫秒），消息以 1kHz 发送，
//                    默认 10ms = 连续丢 10 帧判为失联
// =========================================================

// g2c 云台→底盘 控制指令：失联后归零并切 PASSIVE（停车）
#define BOARD_COMM_TIMEOUT_G2C_ENABLE   0
#define BOARD_COMM_TIMEOUT_G2C_MS       10

// imu 云台IMU→底盘 航向：失联后航向角/角速度清零
#define BOARD_COMM_TIMEOUT_IMU_ENABLE   0
#define BOARD_COMM_TIMEOUT_IMU_MS       10

// c2g 底盘→云台 数据：失联后 chassis_data 清零（格式待定，默认关闭）
#define BOARD_COMM_TIMEOUT_C2G_ENABLE   1
#define BOARD_COMM_TIMEOUT_C2G_MS       10

#endif // PYRO_BOARD_COMM_CONFIG_H
