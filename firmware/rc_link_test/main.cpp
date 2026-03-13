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

struct RCPayload {
    uint16_t channel[9];
};
RCPayload payload; 

int main() {
    stdio_init_all();
    sleep_ms(2000); // 2-second boot delay, NO USB LOCK!

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

    radio.setPALevel(RF24_PA_MAX);      
    radio.setDataRate(RF24_250KBPS);    
    radio.setChannel(76);               
    radio.setPayloadSize(sizeof(RCPayload)); 
    radio.disableCRC();                 

    if (IS_INITIATOR) {
        radio.openWritingPipe(address_A);
        radio.stopListening();
        for (int i = 0; i < 9; i++) {
            payload.channel[i] = 1500;
        }
    } else {
        radio.openReadingPipe(1, address_A);
        radio.startListening();
    }

    uint32_t last_tx_time = time_us_32();
    uint32_t last_heartbeat = time_us_32();

    while (true) {
        uint32_t current_time = time_us_32();

        // Heartbeat for BOTH boards so you know they are alive
        if (current_time - last_heartbeat > 1000000) {
            if (!IS_INITIATOR) printf("."); // Print a dot every second on the Rover
            last_heartbeat = current_time;
        }

        if (IS_INITIATOR) {
            // Blast data 50 times a second
            if (current_time - last_tx_time > 20000) {
                payload.channel[0]++;
                if (payload.channel[0] > 2000) payload.channel[0] = 1000;

                radio.write(&payload, sizeof(payload));
                last_tx_time = current_time; 
            }
        } else {
            // ROVER (RECEIVER)
            if (radio.available()) {
                radio.read(&payload, sizeof(payload));
                printf("\n[RX] Ch0 (Throttle): %d | Ch1 (Steering): %d\n", 
                       payload.channel[0], payload.channel[1]);
            }
        }
        sleep_ms(1); 
    }
    return 0;
}