cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(sbus_cc1101_receiver C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(sbus_cc1101_receiver main.c)

target_include_directories(sbus_cc1101_receiver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_link_libraries(sbus_cc1101_receiver
    pico_stdlib
    hardware_gpio
    hardware_timer
    hardware_spi)

pico_enable_stdio_usb(sbus_cc1101_receiver 1)
pico_enable_stdio_uart(sbus_cc1101_receiver 0)
pico_add_extra_outputs(sbus_cc1101_receiver)
