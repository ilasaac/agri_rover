cmake_minimum_required(VERSION 3.13)

# Include the Pico SDK
include(pico_sdk_import.cmake)

project(nrf24_heartbeat_project C CXX ASM)

pico_sdk_init()

# Include the RF24 library directory
add_subdirectory(RF24)

# ==========================================
# TARGET 1: THE INITIATOR (Sender)
# ==========================================
add_executable(initiator_fw main.cpp)

# Inject the macro definition into the C++ code
target_compile_definitions(initiator_fw PRIVATE IS_INITIATOR=1)

target_link_libraries(initiator_fw pico_stdlib hardware_spi RF24)
pico_enable_stdio_usb(initiator_fw 1)
pico_enable_stdio_uart(initiator_fw 0)
pico_add_extra_outputs(initiator_fw)

# ==========================================
# TARGET 2: THE RESPONDER (Receiver)
# ==========================================
add_executable(responder_fw main.cpp)

# Inject the macro definition into the C++ code
target_compile_definitions(responder_fw PRIVATE IS_INITIATOR=0)

target_link_libraries(responder_fw pico_stdlib hardware_spi RF24)
pico_enable_stdio_usb(responder_fw 1)
pico_enable_stdio_uart(responder_fw 0)
pico_add_extra_outputs(responder_fw)