#  PYRo Robot ID & Build Dashboard (Strict No-Default Edition)
# =============================================================================

# -----------------------------------------------------------------------------
# 1. 显式加载格式定义
# -----------------------------------------------------------------------------
include("${CMAKE_SOURCE_DIR}/CMake/config/pyro_format.cmake")

# -----------------------------------------------------------------------------
# 2. 核心配置参数 (CACHE VARIABLES)
# -----------------------------------------------------------------------------
# 【关键修改】绝对不给 ROBOT_ID 设置默认值！
# 这里只定义其他可选参数的默认值

set(DEMO_MODE 1 CACHE STRING "Enable Demo Mode (1=ON, 0=OFF)")
set(DEBUG_MODE 1 CACHE STRING "Enable Debug Mode (1=ON, 0=OFF)")
set(IMU_CALIBRATION_EN 0 CACHE STRING "Enable IMU Calibration (1=ON, 0=OFF)")

# -----------------------------------------------------------------------------
# 3. 校验逻辑 & ID 映射
# -----------------------------------------------------------------------------
set(CONFIG_VALID "TRUE")
set(FAIL_REASON "")
set(DISPLAY_ID "N/A")

# 检查 ROBOT_ID 是否由外部传入
if(NOT DEFINED ROBOT_ID)
    set(CONFIG_VALID "FALSE")
    set(FAIL_REASON "ROBOT_ID IS MISSING")
    set(ROBOT_NAME "UNKNOWN")
endif()
    set(DISPLAY_ID "${ROBOT_ID}")

    # 定义 ID 常量
    set(TEST_ROBOT_ID 0)
    set(HERO_ID 1)
    set(ENGINEER_ID 2)
    set(INFANTRY1_ID 3)
    set(INFANTRY2_ID 4)
    set(SENTRY_ID 5)
    set(UAV_ID 6)
    set(DARTS_ID 7)
    set(RADAR_ID 8)
    set(SUB_HERO_ID 10)
    set(SUB_ENGINEER_ID 20)
    set(SUB_INFANTRY_ID 30)
    set(SUB_SENTRY_ID 50)

    # 映射名称
    if(ROBOT_ID EQUAL 0)
        set(ROBOT_NAME "TEST_ROBOT")
    elseif(ROBOT_ID EQUAL 1)
        set(ROBOT_NAME "HERO")
    elseif(ROBOT_ID EQUAL 2)
        set(ROBOT_NAME "ENGINEER")
    elseif(ROBOT_ID EQUAL 3)
        set(ROBOT_NAME "INFANTRY1")
    elseif(ROBOT_ID EQUAL 4)
        set(ROBOT_NAME "INFANTRY2")
    elseif(ROBOT_ID EQUAL 5)
        set(ROBOT_NAME "SENTRY")
    elseif(ROBOT_ID EQUAL 6)
        set(ROBOT_NAME "UAV")
    elseif(ROBOT_ID EQUAL 7)
        set(ROBOT_NAME "DARTS")
    elseif(ROBOT_ID EQUAL 8)
        set(ROBOT_NAME "RADAR")
    elseif(ROBOT_ID EQUAL 10)
        set(ROBOT_NAME "SUB_HERO")
    elseif(ROBOT_ID EQUAL 20)
        set(ROBOT_NAME "SUB_ENGINEER")
    elseif(ROBOT_ID EQUAL 30)
        set(ROBOT_NAME "SUB_INFANTRY")
    elseif(ROBOT_ID EQUAL 50)
        set(ROBOT_NAME "SUB_SENTRY")
    else()
        set(ROBOT_NAME "UNKNOWN")
    endif()


# -----------------------------------------------------------------------------
# 4. 传递宏定义 (仅当配置有效时)
# -----------------------------------------------------------------------------
if(NOT DEFINED ROBOT_ID)
    add_compile_definitions(ROBOT_ID=1)
else()
    add_compile_definitions(ROBOT_ID=${ROBOT_ID})
endif()

add_compile_definitions(DEMO_MODE=${DEMO_MODE})
add_compile_definitions(DEBUG_MODE=${DEBUG_MODE})
add_compile_definitions(IMU_CALIBRATION_EN=${IMU_CALIBRATION_EN})

    # 传递常量 ID
add_compile_definitions(TEST_ROBOT_ID=${TEST_ROBOT_ID})
add_compile_definitions(HERO_ID=${HERO_ID})
add_compile_definitions(ENGINEER_ID=${ENGINEER_ID})
add_compile_definitions(INFANTRY1_ID=${INFANTRY1_ID})
add_compile_definitions(INFANTRY2_ID=${INFANTRY2_ID})
add_compile_definitions(SENTRY_ID=${SENTRY_ID})
add_compile_definitions(UAV_ID=${UAV_ID})
add_compile_definitions(DARTS_ID=${DARTS_ID})
add_compile_definitions(RADAR_ID=${RADAR_ID})
add_compile_definitions(SUB_HERO_ID=${SUB_HERO_ID})
add_compile_definitions(SUB_ENGINEER_ID=${SUB_ENGINEER_ID})
add_compile_definitions(SUB_INFANTRY_ID=${SUB_INFANTRY_ID})
add_compile_definitions(SUB_SENTRY_ID=${SUB_SENTRY_ID})

# -----------------------------------------------------------------------------
# 5. 视觉样式定义 (Fallback)
# -----------------------------------------------------------------------------
if(NOT DEFINED SUCCESS_FORMAT)
    string(ASCII 27 ESC)
    set(RESET_ALL   "${ESC}[0m")
    set(STYLE_BOLD  "1")
    set(SUCCESS_FORMAT "${ESC}[${STYLE_BOLD};38;2;255;255;255;48;2;16;185;129m")
    set(WARNING_FORMAT "${ESC}[${STYLE_BOLD};38;2;15;23;42;48;2;245;158;11m")
    set(ERROR_FORMAT   "${ESC}[${STYLE_BOLD};38;2;255;255;255;48;2;244;63;94m")
    set(INFO_FORMAT    "${ESC}[${STYLE_BOLD};38;2;255;255;255;48;2;14;165;233m")
    set(TITLE_FORMAT   "${ESC}[${STYLE_BOLD};38;2;255;255;255;48;2;139;92;246m")
    set(HIGHLIGHT_FORMAT "${ESC}[${STYLE_BOLD};38;2;34;211;238;48;2;30;41;59m")
    set(DECOR_COLOR "${ESC}[${STYLE_BOLD};38;2;129;140;248m")
endif()

# -----------------------------------------------------------------------------
# 6. 状态文本处理
# -----------------------------------------------------------------------------
if(CONFIG_VALID STREQUAL "FALSE")
    set(TXT_DEBUG "${DECOR_COLOR}N/A${RESET_ALL}")
    set(TXT_DEMO  "${DECOR_COLOR}N/A${RESET_ALL}")
else()

    if(DEBUG_MODE STREQUAL "1" OR DEBUG_MODE STREQUAL "ON")
        set(TXT_DEBUG "${HIGHLIGHT_FORMAT} ON ${RESET_ALL}")
    else()
        set(TXT_DEBUG "${DECOR_COLOR}OFF${RESET_ALL}")
    endif()

    if(DEMO_MODE STREQUAL "1" OR DEMO_MODE STREQUAL "ON")
        set(TXT_DEMO "${HIGHLIGHT_FORMAT} ON ${RESET_ALL}")
    else()
        set(TXT_DEMO "${DECOR_COLOR}OFF${RESET_ALL}")
    endif()
endif()

# -----------------------------------------------------------------------------
# 7. 布局排版计算
# -----------------------------------------------------------------------------
set(TERM_WIDTH 80)
macro(CALC_CENTER_PAD TEXT_CONTENT OUT_PAD_VAR)
    string(LENGTH "${TEXT_CONTENT}" CONTENT_LEN)
    math(EXPR PAD_LEN "(${TERM_WIDTH} - ${CONTENT_LEN}) / 2")
    if(PAD_LEN LESS 0)
        set(PAD_LEN 0)
    endif()
    string(REPEAT " " ${PAD_LEN} ${OUT_PAD_VAR})
endmacro()

# 1. 标题
set(RAW_TITLE "PYRO FIRMWARE BUILD REPORT")
CALC_CENTER_PAD("${RAW_TITLE}" PAD_TITLE)
set(LINE_TITLE "${PAD_TITLE}${TITLE_FORMAT} ${RAW_TITLE} ${RESET_ALL}")

# 2. 机器人信息
string(LENGTH "${ROBOT_NAME}" LEN_NAME)
string(LENGTH "${DISPLAY_ID}" LEN_ID)
math(EXPR INFO_LEN "20 + ${LEN_NAME} + ${LEN_ID}")
math(EXPR PAD_INFO_LEN "(${TERM_WIDTH} - ${INFO_LEN}) / 2")
if(PAD_INFO_LEN LESS 0)
    set(PAD_INFO_LEN 0)
endif()
string(REPEAT " " ${PAD_INFO_LEN} PAD_INFO)
set(LINE_ROBOT "${PAD_INFO}${DECOR_COLOR}Target: ${RESET_ALL}${HIGHLIGHT_FORMAT} ${ROBOT_NAME} ${RESET_ALL}   ${DECOR_COLOR}Unit ID: ${RESET_ALL}${HIGHLIGHT_FORMAT} ${DISPLAY_ID} ${RESET_ALL}")

# 3. 配置信息
set(RAW_CONFIG "Config: Debug:[OFF]  Demo:[OFF]")
CALC_CENTER_PAD("${RAW_CONFIG}" PAD_CONFIG)
set(LINE_CONFIG "${PAD_CONFIG}${DECOR_COLOR}Config: ${RESET_ALL}Debug:[${TXT_DEBUG}]  Demo:[${TXT_DEMO}]")

# 4. 状态行 (智能切换)
if(CONFIG_VALID STREQUAL "TRUE")
    set(RAW_STATUS "Status: BUILD SUCCESSFUL")
    CALC_CENTER_PAD("${RAW_STATUS}" PAD_STATUS)
    set(LINE_STATUS "${PAD_STATUS}${DECOR_COLOR}Status: ${RESET_ALL}${SUCCESS_FORMAT} BUILD SUCCESSFUL ${RESET_ALL}")
else()
    set(RAW_STATUS "Status: ${FAIL_REASON}")
    CALC_CENTER_PAD("${RAW_STATUS}" PAD_STATUS)
    set(LINE_STATUS "${PAD_STATUS}${DECOR_COLOR}Status: ${RESET_ALL}${ERROR_FORMAT} ${FAIL_REASON} ❌ ${RESET_ALL}")
endif()

# 5. 警告与资源路径
set(SUCCESS_DIR "${CMAKE_CURRENT_LIST_DIR}/success")
set(WARNING_DIR "${CMAKE_CURRENT_LIST_DIR}/warning")
set(ART_DIR "${SUCCESS_DIR}")
set(LINE_WARN_CONTENT "")
if(CONFIG_VALID STREQUAL "FALSE")
    #    message(STATUS "")
    #    message(STATUS "${LINE_DIV}")
    #    message(STATUS "")
    #    message(STATUS "${LINE_TITLE}")
    #    message(STATUS "")
    #    message(STATUS "${SUB_LINE_DIV}")
    #    message(STATUS "")
    #    message(STATUS "${LINE_ROBOT}")
    #    message(STATUS "")
    #    message(STATUS "${LINE_CONFIG}")
    #    message(STATUS "${LINE_DIV}")
    #    # 这里不显示图片，因为还没配置好
    #    message(STATUS "${LINE_DIV}")
    #    message(STATUS "")
    #    message(STATUS "${LINE_STATUS}") # 显示红色的 ERROR
    #    message(STATUS "")
    #    message(STATUS "${LINE_DIV}")
    #    message(STATUS "")
    set(ART_DIR "${WARNING_DIR}")
    set(RAW_WARN "[!] WARNING: 未定义ROBOT_ID!!!,重新编译")
    CALC_CENTER_PAD("${RAW_WARN}" PAD_WARN)
    set(LINE_WARN_CONTENT "${PAD_WARN}${ERROR_FORMAT} ${RAW_WARN} ${RESET_ALL}")
    set(ART_DIR "${WARNING_DIR}")
    # 终止配置
endif()


if(CONFIG_VALID STREQUAL "TRUE")
    if(IMU_CALIBRATION_EN STREQUAL "1" OR IMU_CALIBRATION_EN STREQUAL "ON")
        set(RAW_WARN "[!] WARNING: IMU 校准状态，不要上力!!!")
        CALC_CENTER_PAD("${RAW_WARN}" PAD_WARN)
        set(LINE_WARN_CONTENT "${PAD_WARN}${ERROR_FORMAT} ${RAW_WARN} ${RESET_ALL}")
        set(ART_DIR "${WARNING_DIR}")
    endif()
endif()

# 6. 分割线
string(REPEAT "━" ${TERM_WIDTH} LINE_STR)
string(REPEAT "░" 80 SUB_LINE_STR)
set(LINE_DIV "${DECOR_COLOR}${LINE_STR}${RESET_ALL}")
set(SUB_LINE_DIV "${DECOR_COLOR}${SUB_LINE_STR}${RESET_ALL}")

# -----------------------------------------------------------------------------
# 8. 错误拦截 (Configuration Phase Output)
# -----------------------------------------------------------------------------
# 如果配置无效，在 Configure 阶段就打印这个面板，然后报错
if(CONFIG_VALID STREQUAL "FALSE")
#    message(STATUS "")
#    message(STATUS "${LINE_DIV}")
#    message(STATUS "")
#    message(STATUS "${LINE_TITLE}")
#    message(STATUS "")
#    message(STATUS "${SUB_LINE_DIV}")
#    message(STATUS "")
#    message(STATUS "${LINE_ROBOT}")
#    message(STATUS "")
#    message(STATUS "${LINE_CONFIG}")
#    message(STATUS "${LINE_DIV}")
#    # 这里不显示图片，因为还没配置好
#    message(STATUS "${LINE_DIV}")
#    message(STATUS "")
#    message(STATUS "${LINE_STATUS}") # 显示红色的 ERROR
#    message(STATUS "")
#    message(STATUS "${LINE_DIV}")
#    message(STATUS "")
    set(ART_DIR "${WARNING_DIR}")
    # 终止配置

endif()

# -----------------------------------------------------------------------------
# 9. 定义 Custom Target (Build Phase Output)
# -----------------------------------------------------------------------------
set(SCRIPT_PATH "${CMAKE_CURRENT_LIST_DIR}/random_show.bat")
set(CORE_CONFIG_H "${CMAKE_SOURCE_DIR}/PYRo/Core/Config/pyro_core_config.h")

add_custom_target(echo_robot_id ALL
        COMMAND ${CMAKE_COMMAND} -E echo ""
        COMMAND ${CMAKE_COMMAND} -E echo "${LINE_DIV}"
        COMMAND ${CMAKE_COMMAND} -E echo ""
        COMMAND ${CMAKE_COMMAND} -E echo "${LINE_TITLE}"
        COMMAND ${CMAKE_COMMAND} -E echo ""
        COMMAND ${CMAKE_COMMAND} -E echo "${SUB_LINE_DIV}"
        COMMAND ${CMAKE_COMMAND} -E echo ""
        COMMAND ${CMAKE_COMMAND} -E echo "${LINE_ROBOT}"
        COMMAND ${CMAKE_COMMAND} -E echo ""
        COMMAND ${CMAKE_COMMAND} -E echo "${LINE_CONFIG}"
        # COMMAND ${CMAKE_COMMAND} -E echo "${LINE_DIV}"
        # COMMAND cmd /c "${SCRIPT_PATH}" "${ART_DIR}"
        COMMAND ${CMAKE_COMMAND} -E echo "${LINE_DIV}"
        COMMAND ${CMAKE_COMMAND} -E echo ""
        COMMAND ${CMAKE_COMMAND} -E echo "${LINE_WARN_CONTENT}"
        COMMAND ${CMAKE_COMMAND} -E echo ""
        COMMAND ${CMAKE_COMMAND} -E echo "${LINE_STATUS}"
        COMMAND ${CMAKE_COMMAND} -E echo ""
        COMMAND ${CMAKE_COMMAND} -E echo "${LINE_DIV}"
        COMMAND ${CMAKE_COMMAND} -E echo ""
        COMMENT "Displaying Robot Identity..."
        VERBATIM
        USES_TERMINAL
)

