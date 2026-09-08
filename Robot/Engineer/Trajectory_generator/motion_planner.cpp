/**
 * @file motion_planner.cpp
 * @brief 五次多项式轨迹生成器实现
 */

#include "motion_planner.h"
#include <cstring>   // for memcpy, memset
#include <algorithm> // for min/max (嵌入式可替换为自定义)

namespace motion {

    // -------------------- 构造函数与重置 --------------------
    Action::Action()
        : numSegments_(0),
        numGripperEvents_(0),
        numCheckpoints_(0),
        totalDuration_(0.0),
        isValid_(false) {
        reset();
    }

    void Action::reset() {
        numSegments_ = 0;
        numGripperEvents_ = 0;
        numCheckpoints_ = 0;
        totalDuration_ = 0.0;
        isValid_ = false;
        std::memset(segments_, 0, sizeof(segments_));
        std::memset(gripperEvents_, 0, sizeof(gripperEvents_));
        std::memset(checkpoints_, 0, sizeof(checkpoints_));
    }

    // -------------------- 核心算法：五次多项式系数计算 --------------------
    bool Action::computeSegmentCoeffs(const Pose& p0, const Pose& p1, double coeffs[NUM_JOINTS][6]) const {
        double T = p1.time - p0.time;
        if (T <= 0.0) {
            return false;
        }

        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            double q0 = p0.joints[j];
            double v0 = p0.vel[j];
            double a0 = p0.acc[j];
            double q1 = p1.joints[j];
            double v1 = p1.vel[j];
            double a1 = p1.acc[j];

            double T2 = T * T;
            double T3 = T2 * T;
            double T4 = T3 * T;
            double T5 = T4 * T;

            double a0_coeff = q0;
            double a1_coeff = v0;
            double a2_coeff = 0.5 * a0;

            double delta_q = q1 - q0;
            double tmp1 = (8.0 * v1 + 12.0 * v0) * T;
            double tmp2 = (3.0 * a0 - a1) * T2;
            double a3_coeff = (20.0 * delta_q - tmp1 - tmp2) / (2.0 * T3);

            double tmp3 = (14.0 * v1 + 16.0 * v0) * T;
            double tmp4 = (3.0 * a0 - 2.0 * a1) * T2;
            double a4_coeff = (-30.0 * delta_q + tmp3 + tmp4) / (2.0 * T4);

            double tmp5 = (6.0 * v1 + 6.0 * v0) * T;
            double tmp6 = (a0 - a1) * T2;
            double a5_coeff = (12.0 * delta_q - tmp5 - tmp6) / (2.0 * T5);

            coeffs[j][0] = a5_coeff;
            coeffs[j][1] = a4_coeff;
            coeffs[j][2] = a3_coeff;
            coeffs[j][3] = a2_coeff;
            coeffs[j][4] = a1_coeff;
            coeffs[j][5] = a0_coeff;
        }
        return true;
    }

    // -------------------- 注册方法一：从 Pose 列表自动计算 --------------------
    bool Action::registerFromPoses(const Pose* poses, size_t count) {
        if (!poses || count < 2) {
            return false;
        }
        if (count - 1 > MAX_SEGMENTS) {
            return false;
        }

        reset();

        // 1. 确保第一个 Pose 时间必须为 0
        if (poses[0].time != 0.0) {
            return false;
        }

        // 2. 遍历计算每一段的系数，同时记录校验点
        for (size_t i = 0; i < count - 1; ++i) {
            const Pose& p0 = poses[i];
            const Pose& p1 = poses[i + 1];

            if (p1.time <= p0.time) {
                reset();
                return false;
            }

            CoeffSegment& seg = segments_[numSegments_];
            seg.startTime = p0.time;
            seg.duration = p1.time - p0.time;

            if (!computeSegmentCoeffs(p0, p1, seg.coeffs)) {
                reset();
                return false;
            }
            numSegments_++;

            // 记录夹爪阶跃事件
            if (numGripperEvents_ == 0 ||
                gripperEvents_[numGripperEvents_ - 1].value != p1.gripper) {
                gripperEvents_[numGripperEvents_].time = p1.time;
                gripperEvents_[numGripperEvents_].value = p1.gripper;
                numGripperEvents_++;
            }
        }

        // 补上第一个夹爪值
        if (numGripperEvents_ == 0) {
            gripperEvents_[0].time = 0.0;
            gripperEvents_[0].value = poses[0].gripper;
            numGripperEvents_ = 1;
        }

        // 提取校验点（从第二个 Pose 开始，因为起点通常不需要校验）
        numCheckpoints_ = 0;
        for (size_t i = 1; i < count; ++i) {
            if (poses[i].tolerance >= 0.0f) {
                if (numCheckpoints_ < MAX_SEGMENTS) {
                    checkpoints_[numCheckpoints_].time = poses[i].time;
                    checkpoints_[numCheckpoints_].tolerance = poses[i].tolerance;
                    numCheckpoints_++;
                }
            }
        }

        totalDuration_ = poses[count - 1].time;
        isValid_ = true;
        return true;
    }

    // -------------------- 注册方法二：直接注入系数（外部计算） --------------------
    bool Action::registerFromCoeffs(const CoeffSegment* segments, size_t segCount,
        const GripperEvent* events, size_t evtCount) {
        if (!segments || segCount == 0 || segCount > MAX_SEGMENTS) {
            return false;
        }
        if (events && evtCount > MAX_SEGMENTS) {
            return false;
        }

        reset();

        std::memcpy(segments_, segments, sizeof(CoeffSegment) * segCount);
        numSegments_ = segCount;

        if (events && evtCount > 0) {
            std::memcpy(gripperEvents_, events, sizeof(GripperEvent) * evtCount);
            numGripperEvents_ = evtCount;
        }
        else {
            gripperEvents_[0].time = 0.0;
            gripperEvents_[0].value = 0.0;
            numGripperEvents_ = 1;
        }

        // 外部注入时不提供校验点（因为系数表中没有校验信息）
        numCheckpoints_ = 0;
        // 但也可以选择允许用户传入校验点列表，此处暂不处理

        totalDuration_ = segments_[segCount - 1].startTime + segments_[segCount - 1].duration;

        for (size_t i = 1; i < segCount; ++i) {
            if (segments_[i].startTime <= segments_[i - 1].startTime) {
                reset();
                return false;
            }
        }

        isValid_ = true;
        return true;
    }

    // -------------------- 获取校验点列表 --------------------
    size_t Action::getCheckpoints(Checkpoint* out, size_t max_count) const {
        if (!out || max_count == 0) return 0;
        size_t cnt = (numCheckpoints_ < max_count) ? numCheckpoints_ : max_count;
        for (size_t i = 0; i < cnt; ++i) {
            out[i] = checkpoints_[i];
        }
        return cnt;
    }

    // -------------------- 运行时查询：二分查找分段索引 --------------------
    int Action::findSegmentIndex(double time) const {
        if (numSegments_ == 0) return -1;

        int low = 0, high = static_cast<int>(numSegments_) - 1;
        int idx = 0;

        while (low <= high) {
            int mid = (low + high) / 2;
            if (segments_[mid].startTime <= time) {
                idx = mid;
                low = mid + 1;
            }
            else {
                high = mid - 1;
            }
        }

        if (time > totalDuration_) {
            return static_cast<int>(numSegments_) - 1;
        }
        if (time < 0.0) {
            return 0;
        }
        return idx;
    }

    // -------------------- 运行时查询：二分查找夹爪事件索引 --------------------
    int Action::findGripperEventIndex(double time) const {
        if (numGripperEvents_ == 0) return -1;

        int low = 0, high = static_cast<int>(numGripperEvents_) - 1;
        int idx = 0;

        while (low <= high) {
            int mid = (low + high) / 2;
            if (gripperEvents_[mid].time <= time) {
                idx = mid;
                low = mid + 1;
            }
            else {
                high = mid - 1;
            }
        }

        if (idx >= static_cast<int>(numGripperEvents_)) idx = numGripperEvents_ - 1;
        if (idx < 0) idx = 0;
        return idx;
    }

    // -------------------- 核心接口：获取目标值 --------------------
    bool Action::getTarget(double time, double* joints_out, double* gripper_out) const {
        if (!isValid_ || !joints_out || !gripper_out) {
            return false;
        }

        if (time < 0.0) time = 0.0;
        if (time > totalDuration_) time = totalDuration_;

        int segIdx = findSegmentIndex(time);
        if (segIdx < 0 || segIdx >= static_cast<int>(numSegments_)) {
            return false;
        }

        const CoeffSegment& seg = segments_[segIdx];
        double tau = time - seg.startTime;

        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            const double* c = seg.coeffs[j];
            double val = c[0] * tau + c[1];
            val = val * tau + c[2];
            val = val * tau + c[3];
            val = val * tau + c[4];
            val = val * tau + c[5];
            joints_out[j] = val;
        }

        int evtIdx = findGripperEventIndex(time);
        if (evtIdx >= 0 && evtIdx < static_cast<int>(numGripperEvents_)) {
            *gripper_out = gripperEvents_[evtIdx].value;
        }
        else {
            *gripper_out = 0.0;
        }

        return true;
    }

} // namespace motion