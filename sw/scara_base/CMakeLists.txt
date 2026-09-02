#
# @brief   scara-base main entry point for build process
# @version v1.0.0
# @date    2026-08-29
# @author  Vladimir Roncevic <elektron.ronca@gmail.com>
#

# Sets CMake version
cmake_minimum_required(VERSION 3.12)

# Pico SDK import setup
include(pico_sdk_import.cmake)

# Sets project info
project(scara-base C CXX ASM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_PROJECT_VERSION 1.0.0)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PRO_PATH ${PROJECT_SOURCE_DIR})
set(PICO_BOARD "pico")

# Initializes the SDK
pico_sdk_init()

# Adds linker flags 
add_link_options(-Wl,-gc-sections,--print-memory-usage)

# Sets project url
include(pro_auto_set_url.cmake)

# Adds source directory
add_subdirectory(src)
