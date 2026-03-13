#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "RF24.h"

#ifndef IS_INITIATOR
#define IS_INITIATOR 1
#endif

#define PIN_MISO 4
#define PIN_CSN  5
#define PIN_SCK  6
#define PIN_MOSI 7
#define PIN_CE   8

RF24 radio(PIN_CE, PIN_CSN);
const uint8_t address_A[6] = "1Node";

// --- THE DATA PAYLOAD ---
// This struct holds your 9 PPM values. Total size: 18 bytes.
struct RCPayload {
    uint16_t channel[9];
};

RCPayload payload; // Create an instance of the struct

int main() {
    stdio_init_all();
    sleep_ms(2000); // 2-second boot delay for power stabilization

    printf("\n=================================\n");
    if (IS_INITIATOR) printf("  STARTING CONTROLLER (Transmitter)\n");
    else printf("  STARTING ROVER (Receiver)\n");
    printf("=================================\n");

    spi_init(spi0, 1000000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    if (!radio.begin()) {
        printf("ERROR: nRF24L01 not found!\n");
    }

    // --- BULLETPROOF RF SETTINGS ---
    radio.setPALevel(RF24_PA_MAX);      // Turn power UP for actual room/outdoor distance
    radio.setDataRate(RF24_250KBPS);    // Slow & steady for maximum range
    radio.setChannel(76);               // Wi-Fi safe channel
    radio.setPayloadSize(sizeof(RCPayload)); // Set payload size to exactly 18 bytes
    radio.disableCRC();                 // Keep CRC off to bypass clone chip bugs

    if (IS_INITIATOR) {
        // Controller only needs to write
        radio.openWritingPipe(address_A);
        radio.stopListening();
        
        // Initialize dummy PPM values (Center stick positions ~1500)
        for (int i = 0; i < 9; i++) {
            payload.channel[i] = 1500;
        }
    } else {
        // Rover only needs to read
        radio.openReadingPipe(1, address_A);
        radio.startListening();
    }

    uint32_t last_tx_time = time_us_32();

    while (true) {
        if (IS_INITIATOR) {
            // Send data 50 times per second (every 20,000 microseconds)
            if (time_us_32() - last_tx_time > 20000) {
                
                // Simulate stick movement on Channel 0 (e.g., Throttle)
                payload.channel[0]++;
                if (payload.channel[0] > 2000) payload.channel[0] = 1000;

                // Blast the entire 18-byte struct into the air
                radio.write(&payload, sizeof(payload));
                
                printf("[TX] Sent Ch0: %d | Ch1: %d\n", payload.channel[0], payload.channel[1]);
                last_tx_time = time_us_32(); 
            }
        } else {
            // --- ROVER (RECEIVER) ---
            if (radio.available()) {
                // Read the incoming bytes directly into our struct memory
                radio.read(&payload, sizeof(payload));
                
                // Print a few channels to prove it works
                printf("[RX] Ch0 (Throttle): %d | Ch1 (Steering): %d | Ch8 (Aux): %d\n", 
                       payload.channel[0], payload.channel[1], payload.channel[8]);
            }
        }
        sleep_ms(1); // Keep the loop from hogging the CPU
    }
    return 0;
}