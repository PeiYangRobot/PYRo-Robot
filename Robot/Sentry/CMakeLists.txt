# =========================================================================
# 1. 显式定义当前编译的板子类型（在这里切换 GIMBAL_BOARD 或 CHASSIS_BOARD）
# =========================================================================
set(BOARD "GIMBAL_BOARD")
#set(BOARD "CHASSIS_BOARD")

# =========================================================================
# 2. 声明Hero目标文件库（OBJECT库，仅编译目标文件不链接，保留原工程逻辑）
# =========================================================================
add_library(Sentry OBJECT)

# 将 CMake 变量的值作为宏传递给 C/C++ 编译器代码
target_compile_definitions(Sentry PRIVATE
        BOARD=${BOARD}
        GIMBAL_BOARD=1
        CHASSIS_BOARD=2
)

# ===================== 所有源文件（严格匹配文件名/大小写/路径） =====================
if (BOARD STREQUAL "GIMBAL_BOARD")
    target_sources(Sentry
            PRIVATE
            # === Application ===
            Application/Gimbal_Board/pyro_booster_app.cpp
            Application/Gimbal_Board/pyro_gimbal_app.cpp

            # === Gimbal 模块 ===
            Gimbal/pyro_sentry_gimbal.cpp
            Gimbal/fsm/pyro_gimbal_active_state.cpp
            Gimbal/fsm/pyro_gimbal_passive_state.cpp


            # === Booster 模块 ===
            Booster/pyro_sentry_booster.cpp
            Booster/fsm/pyro_booster_active_state.cpp
            Booster/fsm/pyro_booster_passive_state.cpp

            Booster/fsm/pyro_booster_wait_state.cpp
            Booster/fsm/pyro_booster_ready_state.cpp
            Booster/fsm/pyro_booster_singgle_state.cpp
            Booster/fsm/pyro_booster_muti_state.cpp
            Booster/fsm/pyro_booster_stall_state.cpp
            Booster/fsm/pyro_booster_stay_state.cpp

            # === Communication ===
            #Communication/Gimbal_board/pyro_autoaim_drv.cpp
            #Communication/Gimbal_board/pyro_autoaim_com.cpp

            Communication/pyro_board_comm.cpp

            pyro_mission_planer.cpp
            pyro_init_thread.cpp
    )

    # 针对 Gimbal 的特殊头文件包含
    target_include_directories(Sentry
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/Booster
            ${CMAKE_CURRENT_SOURCE_DIR}/Gimbal
            ${CMAKE_CURRENT_SOURCE_DIR}/Communication/Gimbal_board
            ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_compile_definitions(PYRo PUBLIC
            DR16_UART=PYRO_UART5
            VT03_UART=PYRO_UART1
            AUTOAIM_UART=PYRO_UART7
    )

elseif (BOARD STREQUAL "CHASSIS_BOARD")
    target_sources(Sentry
            PRIVATE
            # === Application ===
            Application/Chassis_Board/pyro_chassis_app.cpp

            # === Chassis 模块 ===
            Chassis/pyro_rudder_chassis.cpp
            Chassis/fsm/pyro_rudder_active_state.cpp
            Chassis/fsm/pyro_rudder_passive_state.cpp

            # === Communication ===
            Communication/pyro_board_comm.cpp

            pyro_mission_planer.cpp
            pyro_init_thread.cpp
    )

    # 针对 Chassis 的特殊头文件包含
    target_include_directories(Sentry
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/Chassis
            ${CMAKE_CURRENT_SOURCE_DIR}
    )

    target_compile_definitions(PYRo PUBLIC
            REFEREE_UART=PYRO_UART1
            DR16_UART=PYRO_UART5
            SUPERCAP_UART=PYRO_UART7
            
    )
endif ()

# ===================== 头文件包含路径（覆盖所有模块根目录） =====================
target_include_directories(Sentry
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Booster
        ${CMAKE_CURRENT_SOURCE_DIR}/Chassis
        ${CMAKE_CURRENT_SOURCE_DIR}/Communication
        ${CMAKE_CURRENT_SOURCE_DIR}/Gimbal
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# ===================== 链接依赖库（保留原工程依赖，可按需增删） =====================
target_link_libraries(Sentry
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)

