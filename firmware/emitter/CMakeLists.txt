cmake_minimum_required(VERSION 3.13)

# Pico SDK v1.5.1 installer sets PICO_SDK_PATH in the environment
include(pico_sdk_import.cmake)

project(sbus_cc1101_emitter C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(sbus_cc1101_emitter main.c)

# cc1101.h lives one level up (cc1101_link/)
target_include_directories(sbus_cc1101_emitter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_link_libraries(sbus_cc1101_emitter
    pico_stdlib
    hardware_uart
    hardware_gpio
    hardware_timer
    hardware_spi)

pico_enable_stdio_usb(sbus_cc1101_emitter 1)
pico_enable_stdio_uart(sbus_cc1101_emitter 0)
pico_add_extra_outputs(sbus_cc1101_emitter)
