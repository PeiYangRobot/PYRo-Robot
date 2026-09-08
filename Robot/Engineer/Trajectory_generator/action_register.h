/**
 * @file action_register.h
 * @brief 动作注册表：管理动作编号与 Action 对象的映射
 * @note 静态分配，无动态内存，适合嵌入式环境
 */

#ifndef ACTION_REGISTER_H
#define ACTION_REGISTER_H

#include <cstdint>
#include "motion_planner.h"

namespace motion {

// 最大支持的动作数量（可根据需要调整）
constexpr uint8_t MAX_ACTIONS = 10;

/**
 * @brief 初始化动作注册表（在整车初始化时调用一次）
 * @note 此函数会注册编号0为空动作，其余编号置空
 */
void initActionRegistry();

/**
 * @brief 注册一个动作到指定编号
 * @param id 动作编号（0 ~ MAX_ACTIONS-1）
 * @param action 指向已注册的 Action 对象的指针（需保证生命周期）
 * @return true 注册成功
 */
bool registerAction(uint8_t id, const Action* action);

/**
 * @brief 根据编号获取动作
 * @param id 动作编号
 * @return 指向 Action 的指针，若未注册则返回 nullptr
 */
const Action* getAction(uint8_t id);

} // namespace motion

#endif // ACTION_REGISTER_H