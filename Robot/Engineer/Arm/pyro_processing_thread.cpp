/***************************************************************************************
 *  处理任务：从 DataBoard 读取控制信号（arm_ctrl_*），根据模式处理后写入执行指令（arm_command_*）
 *  当前支持：零力模式（IDLE）、直接关节映射（JOINT_DIRECT）、一键操作（AUTO_SEQUENCE）
 *  频率：50Hz
 **************************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "pyro_typedef.h"
#include "pyro_databoard.h"
#include <cstring>
#include <cmath>                 // 提供 std::fabs

// 轨迹规划相关头文件
#include "action_register.h"
#include "virtual_controller.h"
#include "motion_planner.h"      // 提供 motion::NUM_JOINTS

// 限幅参数
#define JOINT_MIN_LIMIT  -3.1416f
#define JOINT_MAX_LIMIT   3.1416f
#define GRIPPER_MIN_LIMIT -1.5f
#define GRIPPER_MAX_LIMIT 0.0f

// 零力标志值（执行层识别后进入零力状态）
#define ZERO_FORCE_FLAG   80.0f

// ========== 一键操作过渡参数 ==========
#define TRANSITION_STEP_RAD     0.02f    // 每帧最大弧度增量（用于平滑过渡）
#define TRANSITION_TOLERANCE    0.1f    // 到达判定阈值（弧度）

// 外部全局 DataBoard 对象
extern pyro::databoard global_databoard;

// 本地缓存的话题 ID
static uint32_t ctrl_mode_id = 0;
static uint32_t ctrl_joint_id[6] = {0};
static uint32_t ctrl_gripper_id = 0;
static uint32_t cmd_joint_id[6] = {0};
static uint32_t cmd_gripper_id = 0;
static uint32_t fb_joint_id[6] = {0};
static uint32_t fb_gripper_id = 0;

// 一键操作控制输入 ID
static uint32_t tg1_start_id = 0;
static uint32_t tg1_choose_id = 0;

// 一键操作状态反馈 ID
static uint32_t tg1_status_id = 0;
static uint32_t tg1_current_action_id = 0;
static uint32_t tg1_progress_id = 0;
static uint32_t tg1_error_code_id = 0;
static uint32_t tg1_checkpoint_waiting_id = 0;
static uint32_t tg1_current_time_id = 0;
static uint32_t tg1_total_time_id = 0;

// 虚拟控制器实例（静态全局，生命周期贯穿整个线程）
static motion::VirtualController g_tg1_controller;

// 辅助函数：限幅
static inline float clamp(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// 初始化：获取所有话题 ID（如果话题不存在，则返回 0xFFFFFFFF 并持续等待）
static void processing_init() {
    // 持续尝试获取所有 ID，直到全部有效
    while (1) {
        bool all_valid = true;

        // 模式
        ctrl_mode_id = global_databoard.get_topic_id("arm_ctrl_mode");
        if (ctrl_mode_id == 0xFFFFFFFF) all_valid = false;

        // 控制输入（6个关节 + 夹爪）
        const char* ctrl_joint_names[] = {
            "arm_ctrl_joint0", "arm_ctrl_joint1", "arm_ctrl_joint2",
            "arm_ctrl_joint3", "arm_ctrl_joint4", "arm_ctrl_joint5"
        };
        for (int i = 0; i < 6; i++) {
            ctrl_joint_id[i] = global_databoard.get_topic_id(ctrl_joint_names[i]);
            if (ctrl_joint_id[i] == 0xFFFFFFFF) all_valid = false;
        }
        ctrl_gripper_id = global_databoard.get_topic_id("arm_ctrl_gripper");
        if (ctrl_gripper_id == 0xFFFFFFFF) all_valid = false;

        // 执行指令（6个关节 + 夹爪）
        const char* cmd_joint_names[] = {
            "arm_command_joint0", "arm_command_joint1", "arm_command_joint2",
            "arm_command_joint3", "arm_command_joint4", "arm_command_joint5"
        };
        for (int i = 0; i < 6; i++) {
            cmd_joint_id[i] = global_databoard.get_topic_id(cmd_joint_names[i]);
            if (cmd_joint_id[i] == 0xFFFFFFFF) all_valid = false;
        }
        cmd_gripper_id = global_databoard.get_topic_id("arm_command_gripper");
        if (cmd_gripper_id == 0xFFFFFFFF) all_valid = false;

        // 实际反馈（6个关节 + 夹爪）
        const char* fb_joint_names[] = {
            "axis1_current_pos", "axis2_current_pos", "axis3_current_pos",
            "axis4_current_pos", "axis5_current_pos", "axis6_current_pos"
        };
        for (int i = 0; i < 6; i++) {
            fb_joint_id[i] = global_databoard.get_topic_id(fb_joint_names[i]);
            if (fb_joint_id[i] == 0xFFFFFFFF) all_valid = false;
        }
        fb_gripper_id = global_databoard.get_topic_id("gripper_current_pos");
        if (fb_gripper_id == 0xFFFFFFFF) all_valid = false;

        // 一键操作控制输入
        tg1_start_id = global_databoard.get_topic_id("arm_ctrl_tg1_start");
        if (tg1_start_id == 0xFFFFFFFF) all_valid = false;
        tg1_choose_id = global_databoard.get_topic_id("arm_ctrl_tg1_choose");
        if (tg1_choose_id == 0xFFFFFFFF) all_valid = false;

        // 一键操作状态反馈
        tg1_status_id = global_databoard.get_topic_id("arm_tg1_status");
        if (tg1_status_id == 0xFFFFFFFF) all_valid = false;
        tg1_current_action_id = global_databoard.get_topic_id("arm_tg1_current_action");
        if (tg1_current_action_id == 0xFFFFFFFF) all_valid = false;
        tg1_progress_id = global_databoard.get_topic_id("arm_tg1_progress");
        if (tg1_progress_id == 0xFFFFFFFF) all_valid = false;
        tg1_error_code_id = global_databoard.get_topic_id("arm_tg1_error_code");
        if (tg1_error_code_id == 0xFFFFFFFF) all_valid = false;
        tg1_checkpoint_waiting_id = global_databoard.get_topic_id("arm_tg1_checkpoint_waiting");
        if (tg1_checkpoint_waiting_id == 0xFFFFFFFF) all_valid = false;
        tg1_current_time_id = global_databoard.get_topic_id("arm_tg1_current_time");
        if (tg1_current_time_id == 0xFFFFFFFF) all_valid = false;
        tg1_total_time_id = global_databoard.get_topic_id("arm_tg1_total_time");
        if (tg1_total_time_id == 0xFFFFFFFF) all_valid = false;

        if (all_valid) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== 更新一键操作状态话题 ====================
static void update_tg1_status(motion::ControllerStatus status, uint32_t action_id,
                              float progress, uint32_t error_code, bool waiting,
                              float current_time, float total_time) {
    pyro::genenral_data_t data_ui;
    data_ui.data_ui = static_cast<uint32_t>(status);
    global_databoard.write_topic(tg1_status_id, data_ui);

    data_ui.data_ui = action_id;
    global_databoard.write_topic(tg1_current_action_id, data_ui);

    data_ui.data_ui = error_code;
    global_databoard.write_topic(tg1_error_code_id, data_ui);

    data_ui.data_ui = waiting ? 1 : 0;
    global_databoard.write_topic(tg1_checkpoint_waiting_id, data_ui);

    pyro::genenral_data_t data_f;
    data_f.data_f = progress;
    global_databoard.write_topic(tg1_progress_id, data_f);

    data_f.data_f = current_time;
    global_databoard.write_topic(tg1_current_time_id, data_f);

    data_f.data_f = total_time;
    global_databoard.write_topic(tg1_total_time_id, data_f);
}

// ==================== 线程主函数 ====================
extern "C" void pyro_processing_thread(void *argument) {
    // 等待所有话题 ID 有效
    processing_init();

    // ========== 初始化所有控制信号到安全状态 ==========
    // 1. 模式置零（IDLE）
    pyro::genenral_data_t mode_data;
    mode_data.data_ui = 0;
    global_databoard.write_topic(ctrl_mode_id, mode_data);

    // 2. 执行指令全部置为零力标志
    pyro::genenral_data_t zero_data;
    zero_data.data_f = ZERO_FORCE_FLAG;
    for (int i = 0; i < 6; i++) {
        global_databoard.write_topic(cmd_joint_id[i], zero_data);
    }
    global_databoard.write_topic(cmd_gripper_id, zero_data);

    // 3. 一键操作控制输入复位
    pyro::genenral_data_t ui_zero;
    ui_zero.data_ui = 0;
    global_databoard.write_topic(tg1_start_id, ui_zero);
    global_databoard.write_topic(tg1_choose_id, ui_zero);

    // 4. 一键操作状态反馈初始化为0
    global_databoard.write_topic(tg1_status_id, ui_zero);
    global_databoard.write_topic(tg1_current_action_id, ui_zero);
    global_databoard.write_topic(tg1_error_code_id, ui_zero);
    global_databoard.write_topic(tg1_checkpoint_waiting_id, ui_zero);

    pyro::genenral_data_t f_zero;
    f_zero.data_f = 0.0f;
    global_databoard.write_topic(tg1_progress_id, f_zero);
    global_databoard.write_topic(tg1_current_time_id, f_zero);
    global_databoard.write_topic(tg1_total_time_id, f_zero);

    // ========== 本地变量 ==========
    uint32_t mode;
    float joint_in[6], gripper_in;
    float joint_out[6], gripper_out;
    TickType_t timestamp;

    // 一键操作状态机变量
    enum Tg1State {
        TG1_IDLE = 0,
        TG1_TRANSITION,
        TG1_RUNNING,
        TG1_PAUSED,
        TG1_FINISHED,
        TG1_ERROR
    };
    Tg1State tg1_state = TG1_IDLE;
    uint32_t tg1_current_action = 0;
    uint32_t tg1_error_code = 0;
    float tg1_progress = 0.0f;
    float tg1_current_time = 0.0f;
    float tg1_total_time = 0.0f;
    bool tg1_waiting = false;

    // 过渡专用变量
    float trans_start_pos[6];
    float trans_start_gripper;
    float trans_target_joints[6];
    float trans_target_gripper;
    float trans_progress = 0.0f;
    float trans_total_dist = 0.0f;
    bool trans_initialized = false;

    while (1) {
        // 1. 读取模式
        pyro::genenral_data_t data;
        auto status = global_databoard.read(ctrl_mode_id, &data, timestamp);
        if (status != pyro::topic::DATA_OK) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        mode = data.data_ui;

        // 2. 读取所有控制输入（用于 JOINT_DIRECT 等模式）
        for (int i = 0; i < 6; i++) {
            status = global_databoard.read(ctrl_joint_id[i], &data, timestamp);
                joint_in[i] = data.data_f;

        }
        status = global_databoard.read(ctrl_gripper_id, &data, timestamp);
        if (status == pyro::topic::DATA_OK) {
            gripper_in = data.data_f;
        } else {
            gripper_in = ZERO_FORCE_FLAG;
        }

        // 3. 根据模式生成输出
        bool write_commands = true;  // 默认写入

        switch (mode) {
            case PYRO_ARM_MODE_IDLE:
                // 零力模式
                for (int i = 0; i < 6; i++) joint_out[i] = ZERO_FORCE_FLAG;
                gripper_out = ZERO_FORCE_FLAG;
                // 如果之前在一键操作模式，强制复位
                if (tg1_state != TG1_IDLE) {
                    g_tg1_controller.stop();
                    tg1_state = TG1_IDLE;
                    tg1_current_action = 0;
                    tg1_error_code = 0;
                    tg1_progress = 0.0f;
                    tg1_current_time = 0.0f;
                    tg1_total_time = 0.0f;
                    tg1_waiting = false;
                    update_tg1_status(motion::ControllerStatus::IDLE, 0, 0.0f, 0, false, 0.0f, 0.0f);
                }
                break;

            case PYRO_ARM_MODE_JOINT_DIRECT:
                // 直接关节映射
                for (int i = 0; i < 6; i++) {
                    if (joint_in[i] == ZERO_FORCE_FLAG) {
                        joint_out[i] = ZERO_FORCE_FLAG;
                    } else {
                        joint_out[i] = clamp(joint_in[i], JOINT_MIN_LIMIT, JOINT_MAX_LIMIT);
                    }
                }
                if (gripper_in == ZERO_FORCE_FLAG) {
                    gripper_out = ZERO_FORCE_FLAG;
                } else {
                    gripper_out = clamp(gripper_in, GRIPPER_MIN_LIMIT, GRIPPER_MAX_LIMIT);
                }
                // 如果之前在一键操作模式，复位
                if (tg1_state != TG1_IDLE) {
                    g_tg1_controller.stop();
                    tg1_state = TG1_IDLE;
                    tg1_current_action = 0;
                    tg1_error_code = 0;
                    tg1_progress = 0.0f;
                    tg1_current_time = 0.0f;
                    tg1_total_time = 0.0f;
                    tg1_waiting = false;
                    update_tg1_status(motion::ControllerStatus::IDLE, 0, 0.0f, 0, false, 0.0f, 0.0f);
                }
                break;

            case PYRO_ARM_MODE_AUTO_SEQUENCE:
                // ========== 一键操作模式（含平滑过渡） ==========
                {
                    // 读取一键操作控制输入
                    pyro::genenral_data_t start_data, choose_data;
                    bool start_valid = (global_databoard.read(tg1_start_id, &start_data, timestamp) == pyro::topic::DATA_OK);
                    bool choose_valid = (global_databoard.read(tg1_choose_id, &choose_data, timestamp) == pyro::topic::DATA_OK);
                    uint32_t start_cmd = start_valid ? start_data.data_ui : 0;
                    uint32_t choose_cmd = choose_valid ? choose_data.data_ui : 0;

                    // 目标值存储
                    double target_joints[motion::NUM_JOINTS];
                    double target_gripper = 0.0;
                    float fb_joints[motion::NUM_JOINTS];
                    double target_joints_for_check[motion::NUM_JOINTS];
                    double target_gripper_dummy;

                    // 先处理状态转换
                    switch (tg1_state) {
                        case TG1_IDLE:
                            if (start_cmd == 1) {
                                const motion::Action* action = motion::getAction(choose_cmd);
                                if (action != nullptr) {
                                    if (g_tg1_controller.loadAction(action)) {
                                        // 获取动作起点（时间0）的目标值
                                        if (g_tg1_controller.getCurrentTarget(target_joints, &target_gripper)) {
                                            // 存储目标值（转为float）
                                            for (int i = 0; i < 6; i++) {
                                                trans_target_joints[i] = (float)target_joints[i];
                                            }
                                            trans_target_gripper = (float)target_gripper;
                                            trans_initialized = false;  // 将在进入 TRANSITION 时初始化
                                            tg1_state = TG1_TRANSITION;
                                            tg1_current_action = choose_cmd;
                                            tg1_error_code = 0;
                                            tg1_waiting = false;
                                            tg1_current_time = 0.0f;
                                            tg1_total_time = (float)g_tg1_controller.getTotalDuration();
                                            tg1_progress = 0.0f;
                                            // 反馈状态为 RUNNING（过渡中），进度 0
                                            update_tg1_status(motion::ControllerStatus::RUNNING,
                                                              tg1_current_action, tg1_progress,
                                                              tg1_error_code, tg1_waiting,
                                                              tg1_current_time, tg1_total_time);
                                        } else {
                                            tg1_error_code = 5; // 获取起点失败
                                            tg1_state = TG1_ERROR;
                                            update_tg1_status(motion::ControllerStatus::ERROR,
                                                              tg1_current_action, tg1_progress,
                                                              tg1_error_code, false,
                                                              tg1_current_time, tg1_total_time);
                                        }
                                    } else {
                                        tg1_error_code = 2; // 加载失败
                                        tg1_state = TG1_ERROR;
                                        update_tg1_status(motion::ControllerStatus::ERROR,
                                                          tg1_current_action, tg1_progress,
                                                          tg1_error_code, false,
                                                          tg1_current_time, tg1_total_time);
                                    }
                                } else {
                                    tg1_error_code = 3; // 动作未注册
                                    tg1_state = TG1_ERROR;
                                    update_tg1_status(motion::ControllerStatus::ERROR,
                                                      tg1_current_action, tg1_progress,
                                                      tg1_error_code, false,
                                                      tg1_current_time, tg1_total_time);
                                }
                            } else if (start_cmd == 2) {
                                // 暂停指令在IDLE状态无效
                            }
                            break;

                        case TG1_TRANSITION:
                            {
                                // 如果是第一次进入，初始化起点
                                if (!trans_initialized) {
                                    // 读取当前位置作为起点
                                    bool fb_ok = true;
                                    for (int i = 0; i < 6; i++) {
                                        pyro::genenral_data_t fb_data;
                                        if (global_databoard.read(fb_joint_id[i], &fb_data, timestamp) == pyro::topic::DATA_OK) {
                                            trans_start_pos[i] = fb_data.data_f;
                                        } else {
                                            fb_ok = false;
                                            break;
                                        }
                                    }
                                    if (fb_ok) {
                                        if (global_databoard.read(fb_gripper_id, &data, timestamp) == pyro::topic::DATA_OK) {
                                            trans_start_gripper = data.data_f;
                                        } else {
                                            fb_ok = false;
                                        }
                                    }
                                    if (!fb_ok) {
                                        // 读取失败，保持上一帧指令，等待下一次
                                        break;
                                    }
                                    // 计算总距离（最大距离）
                                    trans_total_dist = 0.0f;
                                    for (int i = 0; i < 6; i++) {
                                        float dist = fabsf(trans_target_joints[i] - trans_start_pos[i]);
                                        if (dist > trans_total_dist) trans_total_dist = dist;
                                    }
                                    float gripper_dist = fabsf(trans_target_gripper - trans_start_gripper);
                                    if (gripper_dist > trans_total_dist) trans_total_dist = gripper_dist;
                                    trans_progress = 0.0f;
                                    trans_initialized = true;
                                }

                                // 如果总距离为0，直接跳到启动
                                if (trans_total_dist < 1e-6f) {
                                    if (g_tg1_controller.start()) {
                                        tg1_state = TG1_RUNNING;
                                        tg1_current_time = 0.0f;
                                        tg1_progress = 0.0f;
                                        update_tg1_status(motion::ControllerStatus::RUNNING,
                                                          tg1_current_action, tg1_progress,
                                                          tg1_error_code, tg1_waiting,
                                                          tg1_current_time, tg1_total_time);
                                    } else {
                                        tg1_error_code = 1; // 启动失败
                                        tg1_state = TG1_ERROR;
                                        update_tg1_status(motion::ControllerStatus::ERROR,
                                                          tg1_current_action, tg1_progress,
                                                          tg1_error_code, false,
                                                          tg1_current_time, tg1_total_time);
                                    }
                                    break;
                                }

                                // 计算进度增量
                                float progress_increment = TRANSITION_STEP_RAD / trans_total_dist;
                                trans_progress += progress_increment;
                                if (trans_progress > 1.0f) trans_progress = 1.0f;

                                // 计算当前期望位置（插值）
                                for (int i = 0; i < 6; i++) {
                                    joint_out[i] = trans_start_pos[i] + trans_progress * (trans_target_joints[i] - trans_start_pos[i]);
                                }
                                gripper_out = trans_start_gripper + trans_progress * (trans_target_gripper - trans_start_gripper);

                                // ========== 结束条件：仅基于实际反馈误差 ==========
                                // 读取当前实际位置
                                bool fb_ok = true;
                                float fb_joints[6];
                                float fb_gripper = 0.0f;
                                for (int i = 0; i < 6; i++) {
                                    pyro::genenral_data_t fb_data;
                                    if (global_databoard.read(fb_joint_id[i], &fb_data, timestamp) == pyro::topic::DATA_OK) {
                                        fb_joints[i] = fb_data.data_f;
                                    } else {
                                        fb_ok = false;
                                        break;
                                    }
                                }
                                if (fb_ok) {
                                    if (global_databoard.read(fb_gripper_id, &data, timestamp) == pyro::topic::DATA_OK) {
                                        fb_gripper = data.data_f;
                                    } else {
                                        fb_ok = false;
                                    }
                                }

                                if (fb_ok) {
                                    bool all_reached = true;
                                    for (int i = 0; i < 6; i++) {
                                        if (fabsf(fb_joints[i] - trans_target_joints[i]) > TRANSITION_TOLERANCE) {
                                            all_reached = false;
                                            break;
                                        }
                                    }
                                    if (fabsf(fb_gripper - trans_target_gripper) > TRANSITION_TOLERANCE) {
                                        all_reached = false;
                                    }

                                    if (all_reached) {
                                        // 误差达标 → 过渡完成，启动控制器
                                        if (g_tg1_controller.start()) {
                                            tg1_state = TG1_RUNNING;
                                            tg1_current_time = 0.0f;
                                            tg1_progress = 0.0f;
                                            update_tg1_status(motion::ControllerStatus::RUNNING,
                                                              tg1_current_action, tg1_progress,
                                                              tg1_error_code, tg1_waiting,
                                                              tg1_current_time, tg1_total_time);
                                        } else {
                                            tg1_error_code = 1;
                                            tg1_state = TG1_ERROR;
                                            update_tg1_status(motion::ControllerStatus::ERROR,
                                                              tg1_current_action, tg1_progress,
                                                              tg1_error_code, false,
                                                              tg1_current_time, tg1_total_time);
                                        }
                                    }
                                    // 否则继续过渡（不退出）
                                }

                                // 如果未结束，更新反馈状态（过渡中，进度保持0）
                                if (tg1_state == TG1_TRANSITION) {
                                    update_tg1_status(motion::ControllerStatus::RUNNING,
                                                      tg1_current_action, 0.0f,
                                                      tg1_error_code, false,
                                                      0.0f, tg1_total_time);
                                }
                            }
                            break;

                        case TG1_RUNNING:
                            if (start_cmd == 0) {
                                // 停止并复位
                                g_tg1_controller.stop();
                                tg1_state = TG1_IDLE;
                                tg1_current_action = 0;
                                tg1_progress = 0.0f;
                                tg1_current_time = 0.0f;
                                tg1_total_time = 0.0f;
                                tg1_waiting = false;
                                tg1_error_code = 0;
                                update_tg1_status(motion::ControllerStatus::IDLE, 0, 0.0f, 0, false, 0.0f, 0.0f);
                            } else if (start_cmd == 2) {
                                // 暂停
                                if (g_tg1_controller.pause()) {
                                    tg1_state = TG1_PAUSED;
                                    update_tg1_status(motion::ControllerStatus::PAUSED,
                                                      tg1_current_action, tg1_progress,
                                                      tg1_error_code, tg1_waiting,
                                                      tg1_current_time, tg1_total_time);
                                }
                            }
                            // start_cmd == 1 继续运行
                            break;

                        case TG1_PAUSED:
                            if (start_cmd == 0) {
                                g_tg1_controller.stop();
                                tg1_state = TG1_IDLE;
                                tg1_current_action = 0;
                                tg1_progress = 0.0f;
                                tg1_current_time = 0.0f;
                                tg1_total_time = 0.0f;
                                tg1_waiting = false;
                                tg1_error_code = 0;
                                update_tg1_status(motion::ControllerStatus::IDLE, 0, 0.0f, 0, false, 0.0f, 0.0f);
                            } else if (start_cmd == 1) {
                                if (g_tg1_controller.resume()) {
                                    tg1_state = TG1_RUNNING;
                                    update_tg1_status(motion::ControllerStatus::RUNNING,
                                                      tg1_current_action, tg1_progress,
                                                      tg1_error_code, tg1_waiting,
                                                      tg1_current_time, tg1_total_time);
                                }
                            }
                            // start_cmd == 2 保持暂停
                            break;

                        case TG1_FINISHED:
                        case TG1_ERROR:
                            if (start_cmd == 0) {
                                g_tg1_controller.stop();
                                tg1_state = TG1_IDLE;
                                tg1_current_action = 0;
                                tg1_progress = 0.0f;
                                tg1_current_time = 0.0f;
                                tg1_total_time = 0.0f;
                                tg1_waiting = false;
                                tg1_error_code = 0;
                                update_tg1_status(motion::ControllerStatus::IDLE, 0, 0.0f, 0, false, 0.0f, 0.0f);
                            }
                            break;
                    }

                    // 如果当前处于 RUNNING 或 PAUSED 状态，更新轨迹（原有逻辑）
                    if (tg1_state == TG1_RUNNING || tg1_state == TG1_PAUSED) {
                        // 推进时间（仅在 RUNNING 时真正推进，PAUSED 时 update 不会推进时间）
                        (void)g_tg1_controller.update(0.02f);

                        // 获取目标值
                        if (g_tg1_controller.getCurrentTarget(target_joints, &target_gripper)) {
                            // 转换为 float 并限幅
                            for (int i = 0; i < motion::NUM_JOINTS; i++) {
                                joint_out[i] = (float)target_joints[i];
                                if (joint_out[i] != ZERO_FORCE_FLAG) {
                                    joint_out[i] = clamp(joint_out[i], JOINT_MIN_LIMIT, JOINT_MAX_LIMIT);
                                }
                            }
                            gripper_out = (float)target_gripper;
                            if (gripper_out != ZERO_FORCE_FLAG) {
                                gripper_out = clamp(gripper_out, GRIPPER_MIN_LIMIT, GRIPPER_MAX_LIMIT);
                            }
                        } else {
                            // 获取目标失败，保持上一帧
                        }

                        // 更新进度等信息
                        tg1_current_time = (float)g_tg1_controller.getCurrentTime();
                        tg1_total_time = (float)g_tg1_controller.getTotalDuration();
                        if (tg1_total_time > 0.0f) {
                            tg1_progress = tg1_current_time / tg1_total_time;
                        } else {
                            tg1_progress = 0.0f;
                        }
                        tg1_waiting = g_tg1_controller.isWaitingForVerification();

                        // 检查控制器状态变化
                        motion::ControllerStatus ctrl_status2 = g_tg1_controller.getStatus();
                        if (ctrl_status2 == motion::ControllerStatus::FINISHED && tg1_state == TG1_RUNNING) {
                            tg1_state = TG1_FINISHED;
                            tg1_progress = 1.0f;
                            update_tg1_status(motion::ControllerStatus::FINISHED,
                                              tg1_current_action, tg1_progress,
                                              tg1_error_code, tg1_waiting,
                                              tg1_current_time, tg1_total_time);
                        } else if (ctrl_status2 == motion::ControllerStatus::ERROR) {
                            tg1_state = TG1_ERROR;
                            tg1_error_code = 4; // 控制器内部错误
                            update_tg1_status(motion::ControllerStatus::ERROR,
                                              tg1_current_action, tg1_progress,
                                              tg1_error_code, tg1_waiting,
                                              tg1_current_time, tg1_total_time);
                        } else {
                            // 正常更新状态话题
                            update_tg1_status(ctrl_status2,
                                              tg1_current_action, tg1_progress,
                                              tg1_error_code, tg1_waiting,
                                              tg1_current_time, tg1_total_time);
                        }

                        // 如果处于校验等待，处理校验（原有逻辑）
                        if (tg1_waiting && tg1_state == TG1_RUNNING) {
                            bool fb_ok = true;
                            for (int i = 0; i < motion::NUM_JOINTS; i++) {
                                pyro::genenral_data_t fb_data;
                                if (global_databoard.read(fb_joint_id[i], &fb_data, timestamp) == pyro::topic::DATA_OK) {
                                    fb_joints[i] = fb_data.data_f;
                                } else {
                                    fb_ok = false;
                                    break;
                                }
                            }
                            if (fb_ok) {
                                g_tg1_controller.getCurrentTarget(target_joints_for_check, &target_gripper_dummy);
                                double max_error = 0.0;
                                for (int i = 0; i < motion::NUM_JOINTS; i++) {
                                    double err = std::fabs(target_joints_for_check[i] - fb_joints[i]);
                                    if (err > max_error) max_error = err;
                                }
                                const double CHECK_TOLERANCE = 0.01;
                                if (max_error < CHECK_TOLERANCE) {
                                    g_tg1_controller.setVerificationResult(true);
                                } else {
                                    g_tg1_controller.setVerificationResult(false);
                                }
                            } else {
                                g_tg1_controller.setVerificationResult(false);
                            }
                        }
                    } // end if RUNNING/PAUSED

                    // 若处于 TRANSITION 状态，write_commands 已经为 true，可直接写入
                    // 若处于 RUNNING/PAUSED，也通过上述逻辑设置了 joint_out/gripper_out
                }
                break;  // end case PYRO_ARM_MODE_AUTO_SEQUENCE

            case PYRO_ARM_MODE_CARTESIAN:
                // 暂未实现，保持上一帧命令（不更新）
                write_commands = false;
                break;

            default:
                // 未知模式：安全起见置零力标志
                for (int i = 0; i < 6; i++) joint_out[i] = ZERO_FORCE_FLAG;
                gripper_out = ZERO_FORCE_FLAG;
                break;
        }

        // 4. 写入执行指令（除非标记为不写入）
        if (write_commands) {
            for (int i = 0; i < 6; i++) {
                pyro::genenral_data_t out_data;
                out_data.data_f = joint_out[i];
                global_databoard.write_topic(cmd_joint_id[i], out_data);
            }
            pyro::genenral_data_t gripper_data;
            gripper_data.data_f = gripper_out;
            global_databoard.write_topic(cmd_gripper_id, gripper_data);
        }

        // 5. 循环延时，控制频率 50Hz
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}