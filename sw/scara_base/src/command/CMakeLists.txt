#
# @brief   scara-base command parser & dispatcher library build setup
# @version v1.0.0
# @date    2026-08-29
# @author  Vladimir Roncevic <elektron.ronca@gmail.com>
#

add_library(command INTERFACE)

target_sources(
    command INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/trajectory_queue.c
    ${CMAKE_CURRENT_LIST_DIR}/parser.c
    ${CMAKE_CURRENT_LIST_DIR}/dispatcher.c
)

target_include_directories(
    command INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../config
)

target_link_libraries(
    command INTERFACE
    motion
    kinematics
    stepper
    pico_stdlib
    pico_sync
)
