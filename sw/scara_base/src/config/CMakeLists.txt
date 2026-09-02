#
# @brief   config component compilation
# @version v1.0.0
# @date    2026-08-30
# @author  Vladimir Roncevic <elektron.ronca@gmail.com>
#

add_library(config
    config_storage.c
)

target_include_directories(config PUBLIC
    .
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(config
    pico_stdlib
    hardware_flash
    hardware_sync
)
