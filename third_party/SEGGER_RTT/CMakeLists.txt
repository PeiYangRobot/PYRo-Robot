cmake_minimum_required(VERSION 3.22)

add_library(SEGGER_RTT STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/SEGGER_RTT.c
)

target_include_directories(SEGGER_RTT PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
