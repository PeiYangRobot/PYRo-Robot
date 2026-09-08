/**
 * @file virtual_controller.cpp
 * @brief 虚拟控制器实现
 */

#include "virtual_controller.h"
#include <cstring>   // for memset
#include <algorithm> // for min/max
#include <cmath>     // for fabs

namespace motion {

    // -------------------- 构造与析构 --------------------
    VirtualController::VirtualController()
        : action_(nullptr),
        status_(ControllerStatus::IDLE),
        current_time_(0.0),
        verification_timeout_(1.0),      // 默认 1 秒超时
        waiting_for_verification_(false),
        checkpoint_time_(0.0),
        checkpoint_tolerance_(-1.0),
        time_at_checkpoint_start_(0.0),
        verification_fail_count_(0),
        num_checkpoints_(0) {
        std::memset(checkpoints_, 0, sizeof(checkpoints_));
    }

    // -------------------- 生命周期管理 --------------------
    bool VirtualController::loadAction(const Action* action) {
        if (!action || !action->isValid()) {
            return false;
        }

        // 如果当前正在运行，先停止
        if (status_ != ControllerStatus::IDLE && status_ != ControllerStatus::READY) {
            stop();
        }

        action_ = action;
        current_time_ = 0.0;
        waiting_for_verification_ = false;
        verification_fail_count_ = 0;
        status_ = ControllerStatus::READY;

        // 构建校验点列表
        buildCheckpoints();

        return true;
    }

    bool VirtualController::start() {
        if (status_ != ControllerStatus::READY) {
            return false;
        }
        if (!action_ || !action_->isValid()) {
            return false;
        }

        // 重置时间（从头开始）
        current_time_ = 0.0;
        waiting_for_verification_ = false;
        verification_fail_count_ = 0;
        status_ = ControllerStatus::RUNNING;

        return true;
    }

    bool VirtualController::pause() {
        if (status_ != ControllerStatus::RUNNING && status_ != ControllerStatus::PAUSED) {
            return false;
        }
        if (status_ == ControllerStatus::PAUSED) {
            return true; // 已经是暂停状态
        }
        status_ = ControllerStatus::PAUSED;
        return true;
    }

    bool VirtualController::resume() {
        if (status_ != ControllerStatus::PAUSED) {
            return false;
        }
        status_ = ControllerStatus::RUNNING;
        return true;
    }

    void VirtualController::stop() {
        resetInternal();
        status_ = ControllerStatus::IDLE;
    }

    void VirtualController::reset() {
        action_ = nullptr;
        num_checkpoints_ = 0;
        std::memset(checkpoints_, 0, sizeof(checkpoints_));
        resetInternal();
        status_ = ControllerStatus::IDLE;
    }

    // -------------------- 实时更新接口 --------------------
    ControllerStatus VirtualController::update(double dt) {
        if (dt < 0.0) {
            return status_; // 忽略负时间增量
        }

        // 只允许在 RUNNING 状态下推进
        if (status_ != ControllerStatus::RUNNING) {
            return status_;
        }

        // 如果正在等待校验，不推进时间，但检查是否超时
        if (waiting_for_verification_) {
            double elapsed_since_check = current_time_ - time_at_checkpoint_start_;
            if (elapsed_since_check > verification_timeout_) {
                // 校验超时 -> 进入错误状态
                status_ = ControllerStatus::ERROR;
                waiting_for_verification_ = false;  // 清除等待标志
                return status_;
            }
            // 仍然等待中，不推进时间
            return status_;
        }

        // 正常推进时间
        current_time_ += dt;

        // 检查是否超出总时长
        double total_dur = getTotalDuration();
        if (current_time_ >= total_dur) {
            current_time_ = total_dur;
            status_ = ControllerStatus::FINISHED;
            return status_;
        }

        // 检查是否到达校验点
        if (isAtCheckpoint(current_time_)) {
            // 进入校验等待状态
            waiting_for_verification_ = true;
            checkpoint_time_ = current_time_;
            checkpoint_tolerance_ = getCurrentCheckpointTolerance();
            time_at_checkpoint_start_ = current_time_;
            verification_fail_count_ = 0;
            // 注意：此时不修改 status_，依然是 RUNNING
            // 但外界通过 isWaitingForVerification() 可知需要校验
        }

        return status_;
    }

    bool VirtualController::getCurrentTarget(double* joints_out, double* gripper_out) const {
        if (!action_ || !action_->isValid()) {
            return false;
        }
        if (!joints_out || !gripper_out) {
            return false;
        }

        // 如果处于错误状态，仍然可以获取目标（但上层应判断状态）
        return action_->getTarget(current_time_, joints_out, gripper_out);
    }

    // -------------------- 校验接口 --------------------
    bool VirtualController::setVerificationResult(bool verified) {
        if (!waiting_for_verification_) {
            return false;
        }

        if (verified) {
            // 校验通过：清除等待状态，时间继续推进
            waiting_for_verification_ = false;
            verification_fail_count_ = 0;
            return true;
        }
        else {
            // 校验失败：回滚时间到校验点
            current_time_ = checkpoint_time_;
            verification_fail_count_++;

            // 如果连续失败次数过多（比如 10 次），直接进入错误状态
            // 这是一种防抖动机制，防止因为偶然的测量噪声导致无限循环
            if (verification_fail_count_ > 10) {
                status_ = ControllerStatus::ERROR;
                waiting_for_verification_ = false;
                return true;
            }

            // 仍处于等待状态，时间已回滚
            return true;
        }
    }

    bool VirtualController::isWaitingForVerification() const {
        return waiting_for_verification_;
    }

    void VirtualController::setVerificationTimeout(double timeout) {
        if (timeout > 0.0) {
            verification_timeout_ = timeout;
        }
    }

    double VirtualController::getVerificationTimeout() const {
        return verification_timeout_;
    }

    // -------------------- 状态查询 --------------------
    double VirtualController::getTotalDuration() const {
        if (!action_ || !action_->isValid()) {
            return 0.0;
        }
        return action_->getTotalDuration();
    }

    // -------------------- 内部辅助函数 --------------------
    void VirtualController::buildCheckpoints() {
        num_checkpoints_ = 0;
        if (!action_ || !action_->isValid()) {
            return;
        }

        // 从 Action 中读取校验点列表
        // 注意：Action 必须提供 getCheckpoints() 接口
        num_checkpoints_ = action_->getCheckpoints(checkpoints_, MAX_CHECKPOINTS);
    }

    bool VirtualController::isAtCheckpoint(double time) const {
        // 检查当前时间是否正好落在某个校验点
        // 由于浮点数比较，使用一个小的容差窗口
        const double EPS = 1e-6;
        for (size_t i = 0; i < num_checkpoints_; ++i) {
            if (std::fabs(time - checkpoints_[i].time) < EPS) {
                return true;
            }
        }
        return false;
    }

    double VirtualController::getCurrentCheckpointTolerance() const {
        const double EPS = 1e-6;
        for (size_t i = 0; i < num_checkpoints_; ++i) {
            if (std::fabs(current_time_ - checkpoints_[i].time) < EPS) {
                return checkpoints_[i].tolerance;
            }
        }
        return -1.0; // 无校验点
    }

    void VirtualController::resetInternal() {
        current_time_ = 0.0;
        waiting_for_verification_ = false;
        checkpoint_time_ = 0.0;
        checkpoint_tolerance_ = -1.0;
        time_at_checkpoint_start_ = 0.0;
        verification_fail_count_ = 0;
        status_ = ControllerStatus::IDLE;
    }

} // namespace motion