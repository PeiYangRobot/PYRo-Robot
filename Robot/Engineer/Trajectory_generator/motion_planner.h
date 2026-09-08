/**
 * @file motion_planner.h
 * @brief 嵌入式五次多项式轨迹生成器（仅速度+加速度连续）
 * @note 无动态内存分配，所有数组静态分配，适用于实时控制
 */

#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <cstddef>  // for size_t

namespace motion {

    // -------------------- 常量配置 --------------------
    constexpr size_t NUM_JOINTS = 6;          // 关节数量
    constexpr size_t MAX_SEGMENTS = 32;      // 最大分段数量（支持 101 个关键帧）

    // -------------------- 数据结构定义 --------------------

    /**
     * @brief 关键帧姿态（Pose）
     * @note 若未指定速度/加速度，注册时自动填 0
     * @note tolerance < 0 表示该校验点不启用校验
     */
    struct Pose {
        double time;                        // 绝对时刻（秒），首个必须为 0
        double joints[NUM_JOINTS];          // 6 个关节角度（弧度）
        double gripper;                     // 夹爪开度（0~1 或 0~100，仅阶跃）
        double vel[NUM_JOINTS];             // 关节速度（rad/s），默认 0
        double acc[NUM_JOINTS];             // 关节加速度（rad/s²），默认 0
        double tolerance;                   // 位置校验误差阈值，<0 表示不校验，默认为 -1
    };

    /**
     * @brief 预计算好的分段系数（外部注入用）
     */
    struct CoeffSegment {
        double startTime;                   // 该段起始时刻
        double duration;                    // 该段持续时间 (T)
        double coeffs[NUM_JOINTS][6];       // 每关节 6 个系数 [a5, a4, a3, a2, a1, a0]
        // q(t) = a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t + a0
    };

    /**
     * @brief 夹爪阶跃事件（外部注入用）
     */
    struct GripperEvent {
        double time;                        // 阶跃发生时刻
        double value;                       // 阶跃后的值
    };

    /**
     * @brief 校验点信息（与 VirtualController 共用）
     */
    struct Checkpoint {
        double time;        // 校验点时刻
        double tolerance;   // 允许误差（来自 Pose.tolerance）
    };

    // -------------------- 动作容器类 --------------------

    /**
     * @brief 动作容器：加载轨迹，执行插值
     */
    class Action {
    public:
        Action();

        /**
         * @brief 方法一：从 Pose 列表自动计算五次多项式系数（注册时计算）
         * @param poses 关键帧数组
         * @param count 关键帧数量（至少 2 个）
         * @return true 注册成功
         */
        bool registerFromPoses(const Pose* poses, size_t count);

        /**
         * @brief 方法二：直接注入预计算好的系数（外部算好直接塞进来）
         * @param segments 分段系数数组
         * @param segCount 分段数量
         * @param events 夹爪阶跃事件数组（可选，传 nullptr 表示无）
         * @param evtCount 事件数量
         * @return true 注册成功
         */
        bool registerFromCoeffs(const CoeffSegment* segments, size_t segCount,
            const GripperEvent* events, size_t evtCount);

        /**
         * @brief 给定时刻 t，计算目标关节角与夹爪值
         * @param time 当前时刻（秒）
         * @param joints_out 输出 6 个关节角（必须指向长度为 6 的数组）
         * @param gripper_out 输出夹爪值
         * @return true 计算成功（time 在轨迹范围内）
         */
        bool getTarget(double time, double* joints_out, double* gripper_out) const;

        /** @brief 总运行时长（最后一个 Pose 的时刻） */
        double getTotalDuration() const { return totalDuration_; }

        /** @brief 当前动作是否有效（已正确注册） */
        bool isValid() const { return isValid_; }

        /** @brief 重置容器，清空所有数据 */
        void reset();

        /** @brief 获取校验点列表（用于 VirtualController） */
        size_t getCheckpoints(Checkpoint* out, size_t max_count) const;

    private:
        /** @brief 计算单段五次多项式系数（内部方法） */
        bool computeSegmentCoeffs(const Pose& p0, const Pose& p1, double coeffs[NUM_JOINTS][6]) const;

        /** @brief 二分查找当前时刻所在的分段索引 */
        int findSegmentIndex(double time) const;

        /** @brief 二分查找当前时刻的夹爪事件索引 */
        int findGripperEventIndex(double time) const;

        // -------------------- 数据成员（全部静态分配） --------------------
        CoeffSegment segments_[MAX_SEGMENTS];     // 分段系数表
        GripperEvent gripperEvents_[MAX_SEGMENTS]; // 夹爪事件表
        Checkpoint checkpoints_[MAX_SEGMENTS];    // 校验点列表
        size_t numSegments_;                       // 实际分段数
        size_t numGripperEvents_;                  // 实际事件数
        size_t numCheckpoints_;                    // 实际校验点数
        double totalDuration_;                     // 总时长
        bool isValid_;                             // 注册成功标志
    };

} // namespace motion

#endif // MOTION_PLANNER_H