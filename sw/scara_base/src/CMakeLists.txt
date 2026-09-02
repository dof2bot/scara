#
# @brief   scara-base setup output files and target linkage
# @version v1.0.0
# @date    2026-08-29
# @author  Vladimir Roncevic <elektron.ronca@gmail.com>
#

# Prepares PIO target for stepper
pico_simple_hardware_target(pio)

# Enable verbose logging globally
add_compile_definitions(VERBOSE)

# Adds sub-structure includes
include_directories(.)
include_directories(config)
include_directories(kinematics)
include_directories(stepper)
include_directories(motion)
include_directories(command)
include_directories(io)

# Adds sub-structure sources
add_subdirectory(config)
add_subdirectory(kinematics)
add_subdirectory(stepper)
add_subdirectory(motion)
add_subdirectory(command)
add_subdirectory(io)

# Sets executable
add_executable("${CMAKE_PROJECT_NAME}" main.c)

# Enables USB CDC serial output
pico_enable_stdio_usb("${CMAKE_PROJECT_NAME}" 1)

# Disables UART default output
pico_enable_stdio_uart("${CMAKE_PROJECT_NAME}" 0)

# Links all libraries
target_link_libraries(
    "${CMAKE_PROJECT_NAME}"
    config
    kinematics
    stepper
    motion
    command
    io
    pico_stdlib
    pico_multicore
    pico_sync
    hardware_gpio
    hardware_pio
    hardware_dma
    hardware_clocks
    hardware_flash
    hardware_sync
    m
)

# Creates extra output files map/bin/hex/uf2
pico_add_extra_outputs("${CMAKE_PROJECT_NAME}")

# Adds URL via pico_set_program_url
pro_auto_set_url("${CMAKE_PROJECT_NAME}")
