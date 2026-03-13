cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(master_fw C CXX ASM)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(../../rc_link_test/RF24 ${CMAKE_CURRENT_BINARY_DIR}/RF24)

add_executable(master_fw main.cpp)

target_link_libraries(master_fw
    pico_stdlib
    hardware_spi
    hardware_uart
    hardware_gpio
    hardware_timer
    RF24
)

pico_enable_stdio_usb(master_fw 1)
pico_enable_stdio_uart(master_fw 0)
pico_add_extra_outputs(master_fw)
