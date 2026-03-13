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

// EXPLICIT ADDRESSES (No more arrays to prevent pointer swapping)
const uint8_t address_A[6] = "1Node";
const uint8_t address_B[6] = "2Node";

int main() {
    stdio_init_all();
    
    // Wait for the computer to actually connect to the serial port
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(1000); 

    printf("\n=================================\n");
    if (IS_INITIATOR) {
        printf("  STARTING AS INITIATOR (Sender)\n");
    } else {
        printf("  STARTING AS RESPONDER (Receiver)\n");
    }
    printf("=================================\n");

    // Explicitly route the SPI pins
    spi_init(spi0, 1000000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    bool radio_ok = radio.begin();

    if (!radio_ok) {
        printf("ERROR: nRF24L01 not found! Check wiring.\n");
    } else {
        printf("SUCCESS: nRF24L01 found and initialized.\n");
        
        // --- DESKTOP TESTING & CLONE TIMING FIXES ---
        radio.setPALevel(RF24_PA_LOW);       // Lower power to prevent desk overload
        radio.setDataRate(RF24_1MBPS);       
        radio.setChannel(76);                
        radio.setRetries(15, 15);            // MAGIC FIX: Max delay (4000us) and max retries (15) for clones
        radio.setPayloadSize(sizeof(uint32_t)); 

        // Explicitly route addresses based on role
        if (IS_INITIATOR) {
            radio.openWritingPipe(address_A);       // Initiator writes to A
            radio.openReadingPipe(1, address_B);    // Initiator listens on B
            radio.stopListening();
        } else {
            radio.openWritingPipe(address_B);       // Responder writes to B
            radio.openReadingPipe(1, address_A);    // Responder listens on A
            radio.startListening();
        }

        printf("\n--- RADIO DIAGNOSTICS ---\n");
        radio.printPrettyDetails();
        printf("-------------------------\n\n");
    }

    uint32_t heartbeat_val = 1;
    uint32_t last_sys_heartbeat = time_us_32();
    uint32_t last_radio_ping = time_us_32();

    while (true) {
        uint32_t current_time = time_us_32();

        // System Heartbeat (runs every 2 seconds no matter what)
        if (current_time - last_sys_heartbeat > 2000000) {
            printf("[SYS] RP2040 is alive... (Radio OK: %s)\n", radio_ok ? "YES" : "NO");
            last_sys_heartbeat = current_time;
        }

        // Only attempt radio communication if the radio initialized properly
        if (radio_ok) {
            if (IS_INITIATOR) {
                // INITIATOR: Send a radio heartbeat every 1 second
                if (current_time - last_radio_ping > 1000000) {
                    printf("\n[RADIO] Sending payload: %d\n", heartbeat_val);
                    radio.stopListening();
                    
                    // Check if the hardware actually delivered the packet
                    bool delivery_success = radio.write(&heartbeat_val, sizeof(heartbeat_val));

                    if (!delivery_success) {
                        printf("[RADIO] Delivery failed! (No hardware ACK from Responder)\n");
                    } else {
                        printf("[RADIO] Hardware ACK received! Waiting for software response...\n");
                        
                        radio.startListening();
                        uint32_t wait_start = time_us_32();
                        bool timeout = true;

                        // Wait up to 200ms for the software response
                        while (time_us_32() - wait_start < 200000) {
                            if (radio.available()) {
                                timeout = false;
                                break;
                            }
                        }

                        if (timeout) {
                            printf("[RADIO] Timeout! Responder hardware got it, but software didn't reply.\n");
                        } else {
                            uint32_t response;
                            radio.read(&response, sizeof(response));
                            printf("[RADIO] SUCCESS! Received response: %d\n", response);
                            heartbeat_val = response + 1; 
                        }
                    }
                    last_radio_ping = time_us_32(); 
                }
            } else {
                // RESPONDER: Constantly check for incoming packets
                if (radio.available()) {
                    uint32_t received_val;
                    radio.read(&received_val, sizeof(received_val));
                    printf("\n[RADIO] Received incoming hp: %d\n", received_val);

                    uint32_t next_val = received_val + 1;
                    
                    // Give the Initiator a tiny bit of time to switch its antenna to listening mode
                    sleep_ms(10); 

                    radio.stopListening();
                    radio.write(&next_val, sizeof(next_val));
                    printf("[RADIO] Responding with hp: %d\n", next_val);
                    radio.startListening();
                }
            }
        }
        sleep_ms(1);
    }
    return 0;
}