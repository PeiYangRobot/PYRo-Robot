cmake_minimum_required(VERSION 3.22)
project(forward_kf_checks LANGUAGES C CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(DSP_ROOT "${REPO_ROOT}/third_party/CMSIS-DSP")

set(DSP_SOURCES
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_init_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_add_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_sub_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_mult_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_trans_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_inverse_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_cholesky_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.c
    ${DSP_ROOT}/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.c
)
set(KF_SOURCES
    ${REPO_ROOT}/Robot/infantry1/Chassis/pyro_wl_forward_kf.cpp
    ${DSP_SOURCES})
set(MATRIX_SOURCES ${REPO_ROOT}/PYRo/Algorithm/Matrix/pyro_algo_matrix.cpp)
add_executable(matrix_test matrix_test.cpp ${DSP_SOURCES})
add_executable(forward_kf_test forward_kf_test.cpp ${KF_SOURCES})

find_package(Python3 COMPONENTS Interpreter REQUIRED)
set(CONTROL_TEST_GENERATED "${CMAKE_CURRENT_BINARY_DIR}/control-generated")
add_custom_command(
    OUTPUT ${CONTROL_TEST_GENERATED}/chassis_types.inc ${CONTROL_TEST_GENERATED}/chassis_functions.inc
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/generate_chassis_control_test.py ${CONTROL_TEST_GENERATED}
    DEPENDS generate_chassis_control_test.py
        ${REPO_ROOT}/Robot/infantry1/Chassis/pyro_wl_chassis.h
        ${REPO_ROOT}/Robot/infantry1/Chassis/pyro_wl_chassis.cpp
        ${REPO_ROOT}/Robot/infantry1/Chassis/fsm/active/normal/pyro_wl_balance_state.cpp
        ${REPO_ROOT}/Robot/infantry1/Chassis/fsm/active/normal/pyro_wl_spin_state.cpp
    VERBATIM
)
add_executable(chassis_control_test chassis_control_test.cpp ${KF_SOURCES}
    ${REPO_ROOT}/PYRo/Algorithm/Common/pyro_algo_common.cpp
    ${CONTROL_TEST_GENERATED}/chassis_types.inc ${CONTROL_TEST_GENERATED}/chassis_functions.inc)
target_include_directories(chassis_control_test PRIVATE ${CONTROL_TEST_GENERATED})

add_executable(kalman_core_test kalman_core_test.cpp ${DSP_SOURCES})
add_executable(forward_kf_trace forward_kf_trace.cpp ${KF_SOURCES})
add_executable(forward_kf_reference_trace forward_kf_trace.cpp
    reference/forward_kf_reference.cpp reference/kalman_filter.cpp ${DSP_SOURCES})
target_compile_definitions(forward_kf_reference_trace PRIVATE FORWARD_KF_REFERENCE)

set(OPTIMIZED_TEST_TARGETS)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_executable(matrix_optimized_test matrix_test.cpp ${DSP_SOURCES})
    add_executable(kalman_core_optimized_test kalman_core_test.cpp ${DSP_SOURCES})
    add_executable(forward_kf_optimized_test forward_kf_test.cpp ${KF_SOURCES})
    set(OPTIMIZED_TEST_TARGETS matrix_optimized_test kalman_core_optimized_test forward_kf_optimized_test)
    foreach(TEST_TARGET ${OPTIMIZED_TEST_TARGETS})
        # Verify bit-based finite guards even with finite-math assumptions.
        target_compile_options(${TEST_TARGET} PRIVATE
            -O3 -funsafe-math-optimizations -ffinite-math-only)
    endforeach()
endif()

foreach(TEST_TARGET matrix_test forward_kf_test chassis_control_test kalman_core_test
        forward_kf_trace forward_kf_reference_trace
        ${OPTIMIZED_TEST_TARGETS})
    target_sources(${TEST_TARGET} PRIVATE ${MATRIX_SOURCES})
    target_include_directories(${TEST_TARGET} PRIVATE
        ${REPO_ROOT}/Robot/infantry1/Chassis
        ${REPO_ROOT}/PYRo/Algorithm/Common
        ${REPO_ROOT}/PYRo/Algorithm/Matrix
        ${REPO_ROOT}/PYRo/Algorithm/KF
        ${REPO_ROOT}/PYRo/Core/Def
        ${DSP_ROOT}/Include
        ${DSP_ROOT}/PrivateInclude
    )
    # CMSIS's portable host configuration; keep dimension checks enabled in tests.
    target_compile_definitions(${TEST_TARGET} PRIVATE __GNUC_PYTHON__ ARM_MATH_MATRIX_CHECK)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${TEST_TARGET} PRIVATE -ffp-contract=off -fno-strict-aliasing)
    endif()
endforeach()
foreach(TRACE_SCENARIO normal stress)
    add_test(NAME forward_trace_${TRACE_SCENARIO}
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/compare_forward_trace.py
            $<TARGET_FILE:forward_kf_reference_trace> $<TARGET_FILE:forward_kf_trace>
            ${CMAKE_CURRENT_BINARY_DIR}/forward-trace-${TRACE_SCENARIO} ${TRACE_SCENARIO})
endforeach()
add_test(NAME matrix_checks COMMAND matrix_test)
add_test(NAME forward_kf_checks COMMAND forward_kf_test)
add_test(NAME chassis_control_chain COMMAND chassis_control_test)
add_test(NAME kalman_core_checks COMMAND kalman_core_test)
foreach(TEST_TARGET ${OPTIMIZED_TEST_TARGETS})
    add_test(NAME ${TEST_TARGET} COMMAND ${TEST_TARGET})
endforeach()
