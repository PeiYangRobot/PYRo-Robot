/**
 * @file virtual_controller.h
 * @brief 虚拟控制器：轨迹执行器，管理时间推进与校验同步
 * @note 无动态内存分配，所有状态静态存储
 */

#ifndef VIRTUAL_CONTROLLER_H
#define VIRTUAL_CONTROLLER_H

#include "motion_planner.h"   // 包含 Checkpoint 定义
#include <cstdint>

namespace motion {

    /**
     * @brief 虚拟控制器状态枚举
     */
    enum class ControllerStatus : uint8_t {
        IDLE = 0,
        READY,
        RUNNING,
        PAUSED,
        FINISHED,
        ERROR
    };

    /**
     * @brief 虚拟控制器类
     * @note 不拥有 Action 对象，只持有指针；调用者需保证 Action 生命周期长于控制器
     */
    class VirtualController {
    public:
        VirtualController();

        // 生命周期管理
        bool loadAction(const Action* action);
        bool start();
        bool pause();
        bool resume();
        void stop();
        void reset();

        // 实时更新
        ControllerStatus update(double dt);
        bool getCurrentTarget(double* joints_out, double* gripper_out) const;

        // 校验接口
        bool setVerificationResult(bool verified);
        bool isWaitingForVerification() const;
        void setVerificationTimeout(double timeout);
        double getVerificationTimeout() const;

        // 状态查询
        ControllerStatus getStatus() const { return status_; }
        double getCurrentTime() const { return current_time_; }
        double getTotalDuration() const;
        bool isFinished() const { return status_ == ControllerStatus::FINISHED; }
        bool isError() const { return status_ == ControllerStatus::ERROR; }
        bool hasAction() const { return action_ != nullptr; }

    private:
        void buildCheckpoints();
        bool isAtCheckpoint(double time) const;
        double getCurrentCheckpointTolerance() const;
        void resetInternal();

        const Action* action_;
        ControllerStatus status_;
        double current_time_;
        double verification_timeout_;
        bool waiting_for_verification_;
        double checkpoint_time_;
        double checkpoint_tolerance_;
        double time_at_checkpoint_start_;
        uint32_t verification_fail_count_;

        static constexpr size_t MAX_CHECKPOINTS = 100;
        Checkpoint checkpoints_[MAX_CHECKPOINTS];
        size_t num_checkpoints_;
    };

} // namespace motion

#endif // VIRTUAL_CONTROLLER_H