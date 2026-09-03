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
    ${CMAKE_CURRENT_LIST_DIR}/handlers/cmd_motion.c
    ${CMAKE_CURRENT_LIST_DIR}/handlers/cmd_system.c
    ${CMAKE_CURRENT_LIST_DIR}/handlers/cmd_config.c
    ${CMAKE_CURRENT_LIST_DIR}/handlers/cmd_tool.c
)

target_include_directories(
    command INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/handlers
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../config
)

target_link_libraries(
    command INTERFACE
    motion
    kinematics
    stepper
    config
    io
    homing
    pico_stdlib
    pico_sync
)
