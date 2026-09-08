# =========================================================================
# 1. 显式定义当前编译的板子类型（在这里切换 ARM_BOARD 或 CHASSIS_BOARD）
# =========================================================================
set(BOARD "ARM_BOARD")
#set(BOARD "CHASSIS_BOARD")


# =========================================================================
# 2. 声明 Engineer 目标文件库（OBJECT库，仅编译目标文件不链接）
# =========================================================================
add_library(Engineer OBJECT)

# 将 CMake 变量的值作为宏传递给 C/C++ 编译器代码
target_compile_definitions(Engineer PRIVATE
        BOARD=${BOARD}
)

# ========== 统一编译选项，解决 ARM/Thumb 指令集不匹配 ==========
target_compile_options(Engineer PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
)

# ===================== 两个板型共用的基础源文件（目前所有的源文件） =====================
set(BASE_SOURCES
        
)

# ===================== ARM_BOARD 扩展源文件 =====================
set(ARM_EXTRA_SOURCES
        Arm/pyro_mission_planer.cpp
        Arm/pyro_init_thread.cpp
        Arm/arm_control_execute.cpp
        Arm/pyro_debug_thread.cpp
        Arm/pyro_processing_thread.cpp
        Arm/pyro_receive_thread.cpp
        Arm/self_control_command.cpp
        Trajectory_generator/motion_planner.cpp
        Trajectory_generator/virtual_controller.cpp
        Trajectory_generator/action_register.cpp
        Arm/pyro_arm_interboard_thread.cpp
        DataBoard/pyro_databoard.cpp
)

# ===================== CHASSIS_BOARD 扩展源文件 =====================
set(CHASSIS_EXTRA_SOURCES
        pyro_chassis_new/Chassis/pyro_engineer_chassis.cpp
        pyro_chassis_new/Chassis/fsm/pyro_engineer_passive_state.cpp
        pyro_chassis_new/Chassis/fsm/pyro_engineer_active_state.cpp
        pyro_chassis_new/Application/Chassis_board/pyro_chassis_app.cpp
        pyro_chassis_new/Communication/Chassis_board/pyro_chassis_interboard_com.cpp
        pyro_chassis_new/pyro_init_thread.cpp
        pyro_chassis_new/pyro_mission_planner.cpp

)

# ===================== 组合最终源文件列表 =====================
if (BOARD STREQUAL "ARM_BOARD")
    target_sources(Engineer PRIVATE ${BASE_SOURCES} ${ARM_EXTRA_SOURCES})
    # 针对 ARM_BOARD 的特殊宏定义（作用于 PYRo 目标）
    target_compile_definitions(PYRo PUBLIC
            DR16_UART=PYRO_UART5
    )
    # 为 Engineer 自身添加宏定义
    target_compile_definitions(Engineer PRIVATE
            DR16_UART=PYRO_UART5
            SELF_CTRL_UART=PYRO_UART10
            DEBUG_UART=PYRO_UART1
            INTERBOARD_UART=PYRO_UART7
            RC
    )

elseif (BOARD STREQUAL "CHASSIS_BOARD")
    target_sources(Engineer PRIVATE ${BASE_SOURCES} ${CHASSIS_EXTRA_SOURCES})
    # 针对 CHASSIS_BOARD 的特殊宏定义（作用于 PYRo 目标）

    # 为 Engineer 自身添加宏定义

else()
    message(FATAL_ERROR "Unsupported BOARD type: ${BOARD}. Choose ARM_BOARD or CHASSIS_BOARD.")
endif()

# ===================== 头文件包含路径 =====================
target_include_directories(Engineer
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Arm
        ${CMAKE_CURRENT_SOURCE_DIR}/DataBoard
        ${CMAKE_CURRENT_SOURCE_DIR}/Trajectory_generator
        ${CMAKE_CURRENT_SOURCE_DIR}/pyro_chassis_new
        ${CMAKE_CURRENT_SOURCE_DIR}/pyro_chassis_new/Chassis      # 底盘模块头文件
        ${CMAKE_CURRENT_SOURCE_DIR}/pyro_chassis_new/Communication/Chassis_board  # 板间通信

)

# ===================== 链接依赖库 =====================
target_link_libraries(Engineer
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)