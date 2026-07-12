cmake_minimum_required(VERSION 3.22)


# 设置机器人ID
# 包含机器人配置（定义ROBOT_NAME）
include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/config/pyro_robot_id_config.cmake)

# ==============================================================================
# 编译器与工具链配置
# 必须在第一次调用 project() 之前包含
# ==============================================================================
set(CMAKE_SYSTEM_NAME Generic)
include(${CMAKE_CURRENT_SOURCE_DIR}/CubeMX/cmake/gcc-arm-none-eabi.cmake)

# ==============================================================================
# 2. 项目定义
# ==============================================================================
set(CMAKE_PROJECT_NAME PYRo_Robot)
project(${CMAKE_PROJECT_NAME} C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()
# ==============================================================================
# 3. 额外性能优化 (保留工具链未包含的指令)
# ==============================================================================
add_compile_options(
        "-mthumb"
        "-Wdouble-promotion"
        "-funsafe-math-optimizations"
)
# ==============================================================================
# 4. CMSIS-DSP & dsppp 库引入
# ==============================================================================
set(DSP_REPO "${CMAKE_CURRENT_SOURCE_DIR}/third_party/CMSIS-DSP")

add_compile_definitions(
        ARM_MATH_CM7
        ARM_MATH_LOOPUNROLL
)

set(CMSISDSP_BUILD_EXAMPLES OFF CACHE BOOL "Disable examples" FORCE)
add_subdirectory(${DSP_REPO} build_cmsis_dsp)

target_include_directories(CMSISDSP PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/CubeMX/Drivers/CMSIS/Include"
)

add_library(dsppp_lib INTERFACE)
target_include_directories(dsppp_lib INTERFACE
        ${DSP_REPO}/dsppp/Include
        ${DSP_REPO}/Include
)

# 先创建可执行文件目标，再进入子目录（因为子目录中的配置需要操作这个目标）
add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/CubeMX/cmake/stm32cubemx)
add_subdirectory(PYRo)


if (ROBOT_ID EQUAL HERO_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/hero)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            Hero)
elseif (ROBOT_ID EQUAL SUB_HERO_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/sub_hero)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            Sub_Hero)
elseif (ROBOT_ID EQUAL INFANTRY2_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/infantry2)
elseif (ROBOT_ID EQUAL SENTRY_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/sentry)
endif ()


target_link_libraries(${CMAKE_PROJECT_NAME}
        PYRo
        stm32cubemx
        CMSISDSP
        dsppp_lib
)


add_dependencies(echo_robot_id ${CMAKE_PROJECT_NAME})

