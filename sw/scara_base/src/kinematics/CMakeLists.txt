#
# @brief   scara-base kinematics library build setup
# @version v1.0.0
# @date    2026-08-29
# @author  Vladimir Roncevic <elektron.ronca@gmail.com>
#

add_library(kinematics INTERFACE)

target_sources(
    kinematics INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/scara_ik.c
)

target_include_directories(
    kinematics INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../config
)

target_link_libraries(
    kinematics INTERFACE
    pico_stdlib
    m
)
