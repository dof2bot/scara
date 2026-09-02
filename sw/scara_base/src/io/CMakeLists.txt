#
# @brief   scara-base io library build setup
# @version v1.0.0
# @date    2026-08-29
# @author  Vladimir Roncevic <elektron.ronca@gmail.com>
#

add_library(io INTERFACE)

target_sources(
    io INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/io_gpio.c
)

target_include_directories(
    io INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../config
)

target_link_libraries(
    io INTERFACE
    pico_stdlib
    hardware_gpio
)
