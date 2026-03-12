cmake_minimum_required(VERSION 3.13)

<<<<<<< HEAD
# Include the Pico SDK
include(pico_sdk_import.cmake)

project(nrf24_heartbeat_project C CXX ASM)

pico_sdk_init()

# Create your executable (assuming your C++ file is named main.cpp)
add_executable(heartbeat main.cpp)

# Include the RF24 library directory (assumes it's in a subfolder named RF24)
add_subdirectory(RF24)

# Link the required libraries: standard IO, hardware SPI, and the RF24 library
target_link_libraries(heartbeat 
    pico_stdlib 
    hardware_spi 
    RF24
)

# --- THE MAGIC SAUCE FOR SERIAL OVER USB ---
# Enable output to the USB port
pico_enable_stdio_usb(heartbeat 1)
# Disable output to the hardware UART pins (optional, but good practice here)
pico_enable_stdio_uart(heartbeat 0)

# Generate the .uf2 file to drag-and-drop onto the Pico
pico_add_extra_outputs(heartbeat)
=======
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
>>>>>>> 8fdb5309e6227a2e152d22eb19c8a0f35c249915
