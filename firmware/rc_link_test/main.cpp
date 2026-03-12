#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "RF24.h"

// Allow CMake to define IS_INITIATOR. If it doesn't, default to 1.
#ifndef IS_INITIATOR
#define IS_INITIATOR 1
#endif

// SPI and Pin definitions
#define PIN_MISO 4
#define PIN_CSN  5
#define PIN_SCK  6
#define PIN_MOSI 7
#define PIN_CE   8

RF24 radio(PIN_CE, PIN_CSN);
uint8_t address[2][6] = {"1Node", "2Node"};

int main() {
    stdio_init_all();
    
    // CRITICAL FIX 1: Wait for the computer to actually connect to the serial port!
    // (Note: If you ever power this from a battery instead of USB, you must remove this loop)
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(1000); // Give the terminal one extra second to render the window

    printf("=================================\n");
    if (IS_INITIATOR) {
        printf("  STARTING AS INITIATOR (Sender)\n");
    } else {
        printf("  STARTING AS RESPONDER (Receiver)\n");
    }
    printf("=================================\n");

    // Initialize radio, but DO NOT freeze if it fails
    bool radio_ok = radio.begin();
    if (!radio_ok) {
        printf("ERROR: nRF24L01 not found! Check wiring.\n");
    } else {
        printf("SUCCESS: nRF24L01 found and initialized.\n");
        radio.setPALevel(RF24_PA_LOW);
        radio.setPayloadSize(sizeof(uint32_t));

        if (IS_INITIATOR) {
            radio.openWritingPipe(address[0]);
            radio.openReadingPipe(1, address[1]);
            radio.stopListening();
        } else {
            radio.openWritingPipe(address[1]);
            radio.openReadingPipe(1, address[0]);
            radio.startListening();
        }
    }

    uint32_t heartbeat_val = 1;
    uint32_t last_sys_heartbeat = time_us_32();
    uint32_t last_radio_ping = time_us_32();

    // Main Loop
    while (true) {
        uint32_t current_time = time_us_32();

        // CRITICAL FIX 2: System Heartbeat (runs every 2 seconds no matter what)
        if (current_time - last_sys_heartbeat > 2000000) {
            printf("[SYS] RP2040 is alive... (Radio OK: %s)\n", radio_ok ? "YES" : "NO");
            last_sys_heartbeat = current_time;
        }

        // Only attempt radio communication if the radio initialized properly
        if (radio_ok) {
            if (IS_INITIATOR) {
                // INITIATOR: Send a radio heartbeat every 1 second
                if (current_time - last_radio_ping > 1000000) {
                    printf("[RADIO] Sending payload: %d\n", heartbeat_val);
                    radio.stopListening();
                    radio.write(&heartbeat_val, sizeof(heartbeat_val));

                    radio.startListening();
                    uint32_t wait_start = time_us_32();
                    bool timeout = true;

                    // Wait up to 200ms for a response
                    while (time_us_32() - wait_start < 200000) {
                        if (radio.available()) {
                            timeout = false;
                            break;
                        }
                    }

                    if (timeout) {
                        printf("[RADIO] Timeout! No response received.\n");
                    } else {
                        uint32_t response;
                        radio.read(&response, sizeof(response));
                        printf("[RADIO] Received response: %d\n", response);
                        heartbeat_val = response + 1; // Increment for next round
                    }
                    
                    last_radio_ping = time_us_32(); // Reset radio timer
                }
            } else {
                // RESPONDER: Constantly check for incoming packets
                if (radio.available()) {
                    uint32_t received_val;
                    radio.read(&received_val, sizeof(received_val));
                    printf("[RADIO] Received incoming hp: %d\n", received_val);

                    uint32_t next_val = received_val + 1;
                    sleep_ms(10); // Give initiator time to switch to listening mode

                    radio.stopListening();
                    radio.write(&next_val, sizeof(next_val));
                    printf("[RADIO] Responding with hp: %d\n", next_val);
                    radio.startListening();
                }
            }
        }
        
        // Small delay to prevent the loop from hogging 100% of the CPU
        sleep_ms(1);
    }
    return 0;
}