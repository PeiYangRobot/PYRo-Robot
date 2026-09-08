#include "pyro_bsp_uart.h"
#include "pyro_bsp_can.h"
#include "pyro_can_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_image_drv.h"
#include "pyro_ins.h"
#include "pyro_supercap_drv.h"
#include "pyro_referee.h"
#include "pyro_sr04_drv.h"
#include "pyro_sr05_drv.h"
#include "pyro_us100_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_databoard.h"
#include "action_register.h"
pyro::databoard global_databoard;   // 定义全局 DataBoard 对象
namespace pyro
{

extern void upper_com_init(databoard *db_ptr);

// ==================== 初始化 DataBoard 话题 ====================
void globaldataboard_init()
{
    // ==================== 机械臂控制（ARM） ====================
    // 控制模式
    global_databoard.create_topic("arm_ctrl_mode",      pyro::data_type_t::UNSIGNED_INT);
    // 六个关节
    global_databoard.create_topic("arm_ctrl_joint0",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_ctrl_joint1",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_ctrl_joint2",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_ctrl_joint3",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_ctrl_joint4",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_ctrl_joint5",    pyro::data_type_t::FLOAT);
    // 笛卡尔坐标系
    global_databoard.create_topic("arm_ctrl_position_x",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_ctrl_position_y",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_ctrl_position_z",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_ctrl_orientation_roll",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_ctrl_orientation_pitch",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_ctrl_orientation_yaw",    pyro::data_type_t::FLOAT);
    // 夹爪
    global_databoard.create_topic("arm_ctrl_gripper",    pyro::data_type_t::FLOAT);

    // 执行层指令
    global_databoard.create_topic("arm_command_joint0",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_command_joint1",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_command_joint2",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_command_joint3",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_command_joint4",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_command_joint5",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_command_gripper",   pyro::data_type_t::FLOAT);

    // 反馈量
    global_databoard.create_topic("axis1_current_pos",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("axis2_current_pos",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("axis3_current_pos",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("axis4_current_pos",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("axis5_current_pos",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("axis6_current_pos",    pyro::data_type_t::FLOAT);
    global_databoard.create_topic("gripper_current_pos",  pyro::data_type_t::FLOAT);

    // ==================== 底盘通信（ARM → 底盘 控制指令） ====================
    // 使能（改为 UNSIGNED_INT，更语义化）
    global_databoard.create_topic("chassis_ctrl_enable",        pyro::data_type_t::UNSIGNED_INT);
    global_databoard.create_topic("chassis_ctrl_vx",            pyro::data_type_t::FLOAT);
    global_databoard.create_topic("chassis_ctrl_vy",            pyro::data_type_t::FLOAT);
    global_databoard.create_topic("chassis_ctrl_wz",            pyro::data_type_t::FLOAT);
    global_databoard.create_topic("chassis_ctrl_magazine_pos",  pyro::data_type_t::UNSIGNED_INT);

    // 摇臂控制（虽然暂时不用，但通信框架已预留，先创建以便扩展）
    global_databoard.create_topic("chassis_ctrl_lift_control_mod",   pyro::data_type_t::UNSIGNED_INT);
    global_databoard.create_topic("chassis_ctrl_lift_auto",          pyro::data_type_t::UNSIGNED_INT);
    global_databoard.create_topic("chassis_ctrl_lift_mannual",       pyro::data_type_t::UNSIGNED_INT);
    global_databoard.create_topic("chassis_ctrl_lift_calib_trigger", pyro::data_type_t::UNSIGNED_INT);

    // ==================== 底盘反馈（底盘 → ARM 反馈状态） ====================
    global_databoard.create_topic("chassis_feedback_online",         pyro::data_type_t::SIGNED_INT);
    global_databoard.create_topic("chassis_feedback_magazine_pos",   pyro::data_type_t::SIGNED_INT);
    global_databoard.create_topic("chassis_feedback_magazine_ready", pyro::data_type_t::SIGNED_INT);

    // ==================== 自控指令（规划层使用） ====================
    global_databoard.create_topic("axis1_self_command", pyro::data_type_t::FLOAT);
    global_databoard.create_topic("axis2_self_command", pyro::data_type_t::FLOAT);
    global_databoard.create_topic("axis3_self_command", pyro::data_type_t::FLOAT);
    global_databoard.create_topic("axis4_self_command", pyro::data_type_t::FLOAT);
    global_databoard.create_topic("axis5_self_command", pyro::data_type_t::FLOAT);
    global_databoard.create_topic("axis6_self_command", pyro::data_type_t::FLOAT);

    // ==================== 一键操作控制器 ====================
    // 输入
    global_databoard.create_topic("arm_ctrl_tg1_start",     pyro::data_type_t::UNSIGNED_INT);
    global_databoard.create_topic("arm_ctrl_tg1_choose",    pyro::data_type_t::UNSIGNED_INT);
    // 状态反馈
    global_databoard.create_topic("arm_tg1_status",         pyro::data_type_t::UNSIGNED_INT);
    global_databoard.create_topic("arm_tg1_current_action", pyro::data_type_t::UNSIGNED_INT);
    global_databoard.create_topic("arm_tg1_progress",       pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_tg1_error_code",     pyro::data_type_t::UNSIGNED_INT);
    global_databoard.create_topic("arm_tg1_checkpoint_waiting", pyro::data_type_t::UNSIGNED_INT);
    global_databoard.create_topic("arm_tg1_current_time",   pyro::data_type_t::FLOAT);
    global_databoard.create_topic("arm_tg1_total_time",     pyro::data_type_t::FLOAT);
}

extern "C"
{
    ins_drv_t *ins_drv;
    

    void pyro_init_thread(void *argument)
    {
        dwt_drv_t::init(480); // Initialize DWT at 480 MHz

        // ========== 新库：通过 bsp_can 初始化 CAN ==========
        bsp_can::get_can1().init();
        bsp_can::get_can2().init();
        bsp_can::get_can3().init();
        bsp_can::get_can1().start();
        bsp_can::get_can2().start();
        bsp_can::get_can3().start();

        // INS 暂时注释（需要配置参数）
        // ins_drv = ins_drv_t::get_instance();
        // ins_config_t ins_cfg;
        // ins_cfg.direct = ins_config_t::DIRECT_1;
        // ins_cfg.calibrate = false;
        // ins_cfg.gx_offset = 0.0f;
        // ins_cfg.gy_offset = 0.0f;
        // ins_cfg.gz_offset = 0.0f;
        // ins_cfg.g_norm = 9.80665f;
        // ins_drv->init(ins_cfg);
        pyro::bsp_uart::init_all();
#ifdef DR16_UART
        dr16_drv_t::instance().start();
        dr16_drv_t::instance().enable();
        DR16_UART.reset(100000, UART_WORDLENGTH_9B, UART_STOPBITS_2,
                        UART_PARITY_EVEN);
        DR16_UART.enable_rx_dma();
#endif

#ifdef VT03_UART
        vt03_drv_t::instance().start();
        vt03_drv_t::instance().enable();
        VT03_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                        UART_PARITY_NONE);
        VT03_UART.enable_rx_dma();
        rc_drv_t::init_virtual_rc();
#endif

#ifdef REFEREE_UART
        REFEREE_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                           UART_PARITY_NONE);
        REFEREE_UART.enable_rx_dma();
        referee_drv_t::get_instance()->init();
#endif

#ifdef SUPERCAP_UART
        SUPERCAP_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                            UART_PARITY_NONE);
        SUPERCAP_UART.enable_rx_dma();
        supercap_drv_t::get_instance()->start_rx();
#endif

#ifdef AUTOAIM_UART
        AUTOAIM_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                           UART_PARITY_NONE);
        AUTOAIM_UART.enable_rx_dma();
#endif

#ifdef IMAGE_UART
        IMAGE_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                         UART_PARITY_NONE);
        image_drv_t::get_instance().start();
        image_drv_t::get_instance().init();
#endif

#ifdef CUSTOM_UART
        CUSTOM_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                          UART_PARITY_NONE);
#endif

#ifdef SR04_UART
        SR04_UART.reset(9600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                        UART_PARITY_NONE);
        SR04_UART.enable_rx_dma();
        sr04_drv::get_instance().init();
        sr04_drv::get_instance().start();
#endif

#ifdef US100_UART
        US100_UART.reset(9600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                         UART_PARITY_NONE);
        us100_drv::get_instance().init();
        us100_drv::get_instance().start();
#endif

#ifdef SR05_UART
        SR05_UART.set_level_invert(false,true);
        SR05_UART.reset(9600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                                UART_PARITY_NONE);
        SR05_UART.enable_rx_dma();
        sr05_drv::get_instance().init();
        sr05_drv::get_instance().start();
#endif

        // 创建 DataBoard 话题（现在在同一个 namespace 内，可直接调用）
        globaldataboard_init();
        motion::initActionRegistry();
        //upper_com_init(&global_databoard);
        //vTaskDelete(nullptr);
    }
}

} // namespace pyro