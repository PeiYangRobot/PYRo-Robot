set(BOARD "CHASSIS_BOARD")
#set(BOARD "GIMBAL_BOARD")

add_library(Infantry1 OBJECT)

target_compile_definitions(Infantry1 PRIVATE
        BOARD=${BOARD}
        GIMBAL_BOARD=1
        CHASSIS_BOARD=2
)


if (BOARD STREQUAL "CHASSIS_BOARD")
        target_sources(Infantry1
                PRIVATE
                pyro_init_thread.cpp
                pyro_mission_planer.cpp

                Chassis/pyro_wl_chassis.cpp
                Chassis/pyro_wl_forward_kf.cpp
                Chassis/fsm/active/pyro_wl_active_state.cpp
                Chassis/fsm/active/pyro_wl_manual_state.cpp
                Chassis/fsm/pyro_wl_passive_state.cpp
                Chassis/fsm/active/normal/pyro_wl_normal_state.cpp
                Chassis/fsm/active/normal/pyro_wl_getover_state.cpp
                Chassis/fsm/active/normal/pyro_wl_air_state.cpp
                Chassis/fsm/active/normal/pyro_wl_align_state.cpp
                Chassis/fsm/active/normal/pyro_wl_step_state.cpp
                Chassis/fsm/active/normal/pyro_wl_balance_state.cpp
                Chassis/fsm/active/normal/pyro_wl_jump_state.cpp
                Chassis/fsm/active/normal/pyro_wl_jump_duo_state.cpp
                Chassis/fsm/active/normal/pyro_wl_spin_state.cpp
                Chassis/fsm/active/normal/pyro_wl_rescue_state.cpp


                Application/Chassis_board/pyro_wl_chassis_app.cpp

                Communication/General/pyro_board_drv.cpp
                Communication/Chassis/pyro_board_com.cpp
        )

        target_include_directories(Infantry1
                PUBLIC
                ${CMAKE_CURRENT_SOURCE_DIR}
                ${CMAKE_CURRENT_SOURCE_DIR}/Application/Chassis_board
                ${CMAKE_CURRENT_SOURCE_DIR}/Chassis
                ${CMAKE_CURRENT_SOURCE_DIR}/Communication/General
        )
        target_compile_definitions(PYRo PUBLIC
                DR16_UART=PYRO_UART5
                STP23L_DOWN_UART=PYRO_UART10
                STP23L_FRONT_UART=PYRO_UART7
        )
elseif (BOARD STREQUAL "GIMBAL_BOARD")
        target_sources(Infantry1
                PRIVATE
                pyro_init_thread.cpp
                pyro_mission_planer.cpp

                Gimbal/pyro_wl_gimbal.cpp
                Gimbal/fsm/pyro_gimbal_passive_state.cpp
                Gimbal/fsm/pyro_gimbal_active_state.cpp
                Gimbal/fsm/pyro_gimbal_manual_state.cpp
                Gimbal/fsm/pyro_gimbal_align_state.cpp

                Booster/pyro_wl_booster.cpp
                Booster/BulletSpeedControl/bullet_speed_control.cpp
                Booster/HeatControl/heat_control.cpp
                Booster/fsm/pyro_booster_passive_state.cpp
                Booster/fsm/pyro_booster_active_state.cpp
                Booster/fsm/pyro_booster_ready_state.cpp
                Booster/fsm/pyro_booster_spin_up_state.cpp
                Booster/fsm/pyro_booster_singlefire_state.cpp
                Booster/fsm/pyro_booster_burstfire_state.cpp
                Booster/fsm/pyro_booster_cali_forward_state.cpp
                Booster/fsm/pyro_booster_cali_reverse_state.cpp

                Application/Gimbal_board/pyro_wl_gimbal_app.cpp
                Application/Gimbal_board/pyro_wl_booster_app.cpp

                Communication/General/pyro_board_drv.cpp
                Communication/Gimbal/pyro_board_com.cpp
                Communication/Gimbal/pyro_autoaim_drv.cpp
                Communication/Gimbal/pyro_autoaim_com.cpp
        )
        # 针对 Gimbal 的特殊头文件包含
        target_include_directories(Infantry1
                PUBLIC
                ${CMAKE_CURRENT_SOURCE_DIR}
                ${CMAKE_CURRENT_SOURCE_DIR}/Application/Gimbal_board
                ${CMAKE_CURRENT_SOURCE_DIR}/Application/Booster_board
                ${CMAKE_CURRENT_SOURCE_DIR}/Gimbal
                ${CMAKE_CURRENT_SOURCE_DIR}/Booster
                ${CMAKE_CURRENT_SOURCE_DIR}/Booster/HeatControl
                ${CMAKE_CURRENT_SOURCE_DIR}/Booster/BulletSpeedControl
                ${CMAKE_CURRENT_SOURCE_DIR}/Communication/General
                ${CMAKE_CURRENT_SOURCE_DIR}/Communication/Gimbal
        )
        target_compile_definitions(PYRo PUBLIC
                DR16_UART=PYRO_UART5
                VT03_UART=PYRO_UART1
                AUTOAIM_UART=PYRO_UART7
        )

endif ()

target_link_libraries(Infantry1
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)
