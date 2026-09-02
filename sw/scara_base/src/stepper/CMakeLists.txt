#
# @brief   scara-base stepper driver library build setup
# @version v1.0.0
# @date    2026-08-29
# @author  Vladimir Roncevic <elektron.ronca@gmail.com>
#

add_library(stepper INTERFACE)

# Generate header from PIO program
pico_generate_pio_header(stepper ${CMAKE_CURRENT_LIST_DIR}/stepper_multi.pio)

target_sources(
    stepper INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/stepper_driver.c
)

target_include_directories(
    stepper INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../config
)

target_link_libraries(
    stepper INTERFACE
    pico_stdlib
    hardware_gpio
    hardware_pio
    hardware_dma
    hardware_clocks
)
