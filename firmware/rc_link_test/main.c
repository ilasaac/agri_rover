#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "RF24.h" // Requires the standard RF24 library in your CMake project

// SPI and Pin definitions
#define PIN_MISO 4
#define PIN_CSN  5
#define PIN_SCK  6
#define PIN_MOSI 7
#define PIN_CE   8

// CRITICAL: Set to 1 for the Sender (RP2040 #1) and 0 for the Responder (RP2040 #2)
#define IS_INITIATOR 1

// Instantiate the radio
RF24 radio(PIN_CE, PIN_CSN);

// Define communication pipes (addresses)
uint8_t address[2][6] = {"1Node", "2Node"};

int main() {
    // Initialize standard I/O (Serial over USB)
    stdio_init_all();
    
    // 2-second delay to give you time to open the serial monitor via USB
    sleep_ms(2000); 

    if (!radio.begin()) {
        printf("ERROR: Radio hardware not responding! Check wiring.\n");
        while (1) {
            sleep_ms(1000);
        }
    }

    // Configuration for desktop testing
    radio.setPALevel(RF24_PA_LOW); 
    radio.setPayloadSize(sizeof(uint32_t));

    uint32_t heartbeat = 1;

    // Open pipes based on role
    if (IS_INITIATOR) {
        radio.openWritingPipe(address[0]);
        radio.openReadingPipe(1, address[1]);
        radio.stopListening();
        printf("--- INITIATOR NODE STARTED ---\n");
    } else {
        radio.openWritingPipe(address[1]);
        radio.openReadingPipe(1, address[0]);
        radio.startListening();
        printf("--- RESPONDER NODE STARTED ---\n");
    }

    while (true) {
        if (IS_INITIATOR) {
            // Role: SENDER
            printf("Sending heartbeat: %d\n", heartbeat);
            radio.stopListening();
            radio.write(&heartbeat, sizeof(heartbeat));

            // Switch to listening to await the n+1 response
            radio.startListening();
            uint32_t start_time = time_us_32();
            bool timeout = false;

            // Wait up to 200ms for a response
            while (!radio.available()) {
                if (time_us_32() - start_time > 200000) { 
                    timeout = true;
                    break;
                }
            }

            if (timeout) {
                printf("Timeout! No response received.\n");
            } else {
                uint32_t response;
                radio.read(&response, sizeof(response));
                printf("Received response: %d\n", response);
                
                // Set the next heartbeat based on the response
                heartbeat = response + 1; 
            }
            
            // Wait 1 second before sending the next cycle
            sleep_ms(1000); 

        } else {
            // Role: RESPONDER
            if (radio.available()) {
                uint32_t received_val;
                radio.read(&received_val, sizeof(received_val));
                printf("Received incoming hp: %d\n", received_val);

                // Calculate n+1
                uint32_t next_val = received_val + 1;
                
                // Tiny delay to ensure the initiator is ready to listen
                sleep_ms(10); 

                radio.stopListening();
                radio.write(&next_val, sizeof(next_val));
                printf("Responding with hp: %d\n", next_val);
                radio.startListening();
            }
        }
    }
    return 0;
}