#
# @brief   scara-base homing controller library build setup
# @version v1.0.0
# @date    2026-08-29
# @author  Vladimir Roncevic <elektron.ronca@gmail.com>
#

add_library(homing INTERFACE)

target_sources(
    homing INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/homing_controller.c
)

target_include_directories(
    homing INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../config
    ${CMAKE_CURRENT_LIST_DIR}/../io
    ${CMAKE_CURRENT_LIST_DIR}/../stepper
    ${CMAKE_CURRENT_LIST_DIR}/../motion
)

target_link_libraries(
    homing INTERFACE
    io
    stepper
    motion
    pico_stdlib
    hardware_gpio
    m
)
