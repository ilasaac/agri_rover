cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(controller_nrf24_link C CXX ASM)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# RF24 library shared from rc_link_test
add_subdirectory(../rc_link_test/RF24 ${CMAKE_CURRENT_BINARY_DIR}/RF24)

add_subdirectory(master)
add_subdirectory(slave)
