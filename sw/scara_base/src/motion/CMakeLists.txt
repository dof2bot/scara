#
# @brief   scara-base motion planner library build setup
# @version v1.0.0
# @date    2026-08-29
# @author  Vladimir Roncevic <elektron.ronca@gmail.com>
#

add_library(motion INTERFACE)

target_sources(
    motion INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/motion_planner.c
    ${CMAKE_CURRENT_LIST_DIR}/trajectory_chunk.c
    ${CMAKE_CURRENT_LIST_DIR}/velocity_profile.c
)

target_include_directories(
    motion INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../config
)

target_link_libraries(
    motion INTERFACE
    kinematics
    stepper
    config
    pico_stdlib
    hardware_clocks
    m
)
