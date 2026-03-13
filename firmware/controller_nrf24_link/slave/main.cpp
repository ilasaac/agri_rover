/*
 * controller_nrf24_link/slave/main.cpp
 * RP2040 — nRF24L01+ receiver + PPM generator
 *
 * Receives raw 9-channel PPM from master over nRF24L01+.
 * Applies mode gating locally and drives PPM output.
 * Link is one-way: slave receives only, never transmits back.
 *
 * Pinout:
 *   GP2   → PPM output  (to local ESC or FC)
 *   GP16  ← SPI0 MISO   (nRF24L01+)
 *   GP17  → SPI0 CSN    (nRF24L01+)
 *   GP18  → SPI0 SCK    (nRF24L01+)
 *   GP19  → SPI0 MOSI   (nRF24L01+)
 *   GP20  → nRF24L01+ CE
 *   USB-C ↔ Jetson Nano (heartbeat + telemetry + autonomous commands)
 *
 * USB protocol (CDC-ACM):
 *   Output "CH:thr,str,ch3,...,ch9 MODE:str\n"  on each received RF frame
 *   Output "[RF_LINK_OK]", "[LINK_LOST]"
 *   Input  <HB:N>           Jetson heartbeat — echoed as <HB:N+1>
 *   Input  <T,S>            2-channel throttle/steering override (legacy)
 *   Input  <J:c1,...,c8>    8-channel override (AUTONOMOUS mode only — RF must be active)
 *
 * nRF24L01+ config:
 *   Channel 76, 250 kbps, PA_MAX, no CRC, no auto-ack (one-way RX only)
 *   Payload: RCPayload — 9 × uint16_t = 18 bytes
 *   radio.startListening() — never calls write() or stopListening()
 *
 * Safety timeouts:
 *   RF_TIMEOUT_US        500 ms — PPM stops if no nRF24 packet received
 *   HEARTBEAT_TIMEOUT_US 300 ms — halts in AUTO if Jetson HB silent
 *   PI_CMD_TIMEOUT_US    500 ms — neutral if no autonomous command
 *
 * Mode logic (SWA = channel[2] = CH3, SWB = channel[3] = CH4):
 *   EMERGENCY    SWA < 1700  → thr/str gated to 1500
 *   AUTONOMOUS   SWB > 1700 + HB alive + cmd fresh → Jetson drives thr/str
 *   AUTO-NO-HB   SWB > 1700 but no Jetson HB → neutral
 *   AUTO-TIMEOUT SWB > 1700 + HB alive but cmd timed out → neutral
 *   MANUAL       This rover selected (CH9 in (1250,1750]) → raw sticks
 *   STANDBY      This rover not selected → neutral
 *
 * RF lost behaviour:
 *   No RC signal → PPM stops immediately. The rover does not move.
 *   <J:...> commands are accepted only while RF is active (AUTONOMOUS mode).
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/spi.h"
#include "RF24.h"

// ── PPM ───────────────────────────────────────────────────────────────────
#define PPM_OUTPUT_PIN  2
#define PPM_FRAME_US    20000
#define PPM_PULSE_US    300

// ── Mode switches (0-indexed positions in the 9-channel payload) ──────────
#define SWA_CH               2    // Emergency stop  — channel[2] = CH3
#define SWB_CH               3    // Autonomous mode — channel[3] = CH4
#define EMERGENCY_THRESHOLD  1700
#define AUTONOMOUS_THRESHOLD 1700

// ── Rover selection (CH9, index 8) ────────────────────────────────────────
#define ROVER_SELECT_LOW   1250
#define ROVER_SELECT_HIGH  1750

// ── Timeouts ──────────────────────────────────────────────────────────────
#define RF_TIMEOUT_US        500000   // 500 ms — PPM stops if RF silent
#define HEARTBEAT_TIMEOUT_US 300000   // 300 ms — halt in AUTO if Jetson HB silent
#define PI_CMD_TIMEOUT_US    500000   // 500 ms — hold neutral if no autonomous cmd

// ── nRF24L01+ pins (SPI0 on GP16-19, CE on GP20) ─────────────────────────
#define PIN_MISO  16
#define PIN_CSN   17
#define PIN_SCK   18
#define PIN_MOSI  19
#define PIN_CE    20

static const uint8_t NRF_ADDR[6] = "RV_TX";

struct RCPayload {
    uint16_t channel[9];   // raw PPM values, 1000-2000
};

static RF24 radio(PIN_CE, PIN_CSN);

// ── Globals ───────────────────────────────────────────────────────────────
static uint16_t ppm_channels[8];       // gated values used for PPM output
static uint16_t rover_select_ppm = 1500;

static uint16_t pi_throttle    = 1500;
static uint16_t pi_steering    = 1500;
static uint32_t last_pi_cmd_us = 0;
static char     pi_rx_buf[64];
static uint8_t  pi_rx_idx      = 0;
static bool     pi_in_packet   = false;
static uint32_t last_hb_us     = 0;
static bool     hb_active      = false;

static uint32_t last_rf_rx_us  = 0;
static bool     rf_was_ok      = false;

// ── PPM ISR ───────────────────────────────────────────────────────────────
static volatile bool ppm_active  = false;
static uint16_t      ppm_snap[8];
static int           ppm_ch       = 0;
static bool          ppm_in_pulse = true;

static int64_t ppm_callback(alarm_id_t id, void *user_data) {
    if (!ppm_active) { gpio_put(PPM_OUTPUT_PIN, 1); return 0; }
    if (ppm_in_pulse) {
        gpio_put(PPM_OUTPUT_PIN, 1);
        ppm_in_pulse = false;
        if (ppm_ch < 8) return -(int64_t)(ppm_snap[ppm_ch] - PPM_PULSE_US);
        uint32_t used = 0;
        for (int i = 0; i < 8; i++) used += ppm_snap[i];
        int64_t sync = (int64_t)PPM_FRAME_US - (int64_t)used - PPM_PULSE_US;
        return -(sync < 2500 ? 2500 : sync);
    }
    ppm_ch++;
    if (ppm_ch == 9) { ppm_ch = 0; memcpy(ppm_snap, ppm_channels, sizeof(ppm_snap)); }
    gpio_put(PPM_OUTPUT_PIN, 0);
    ppm_in_pulse = true;
    return -PPM_PULSE_US;
}

static void ppm_stop(void) {
    if (!ppm_active) return;
    ppm_active = false;
    gpio_put(PPM_OUTPUT_PIN, 1);
}

static void ppm_start(void) {
    if (ppm_active) return;
    ppm_ch = 0; ppm_in_pulse = true;
    memcpy(ppm_snap, ppm_channels, sizeof(ppm_snap));
    ppm_active = true;
    add_alarm_in_us(PPM_PULSE_US, ppm_callback, NULL, true);
}

static void ppm_init(void) {
    gpio_init(PPM_OUTPUT_PIN);
    gpio_set_dir(PPM_OUTPUT_PIN, GPIO_OUT);
    for (int i = 0; i < 8; i++) ppm_snap[i] = 1500;
    gpio_put(PPM_OUTPUT_PIN, 1);
}

// ── Process received RF payload ───────────────────────────────────────────
static void process_payload(const RCPayload &pl) {
    uint32_t now = time_us_32();
    last_rf_rx_us = now;

    // Copy switch and auxiliary channels; thr/str are set after mode gating
    for (int i = 2; i < 8; i++) ppm_channels[i] = pl.channel[i];
    rover_select_ppm = pl.channel[8];

    uint16_t raw_throttle = pl.channel[0];
    uint16_t raw_steering = pl.channel[1];

    bool hb_alive    = hb_active && (now - last_hb_us < HEARTBEAT_TIMEOUT_US);
    bool is_selected = (rover_select_ppm > ROVER_SELECT_LOW &&
                        rover_select_ppm <= ROVER_SELECT_HIGH);
    const char *mode_str;

    if (ppm_channels[SWA_CH] < EMERGENCY_THRESHOLD) {
        ppm_channels[0] = 1500; ppm_channels[1] = 1500;
        mode_str = "EMERGENCY";
    } else if (ppm_channels[SWB_CH] > AUTONOMOUS_THRESHOLD) {
        if (!hb_alive) {
            ppm_channels[0] = 1500; ppm_channels[1] = 1500;
            mode_str = "AUTO-NO-HB";
        } else if (now - last_pi_cmd_us < PI_CMD_TIMEOUT_US) {
            ppm_channels[0] = pi_throttle;
            ppm_channels[1] = pi_steering;
            mode_str = "AUTONOMOUS";
        } else {
            ppm_channels[0] = 1500; ppm_channels[1] = 1500;
            mode_str = "AUTO-TIMEOUT";
        }
    } else if (is_selected) {
        // Master is in RELAY mode — this rover drives with raw sticks
        ppm_channels[0] = raw_throttle;
        ppm_channels[1] = raw_steering;
        mode_str = "MANUAL";
    } else {
        // Master driving its own rover — this rover stands by
        ppm_channels[0] = 1500; ppm_channels[1] = 1500;
        mode_str = "STANDBY";
    }

    // CH1/CH2 in telemetry always carry raw values (same convention as master)
    printf("CH:%d,%d,%d,%d,%d,%d,%d,%d,%d MODE:%s\n",
        raw_throttle, raw_steering,
        ppm_channels[2], ppm_channels[3], ppm_channels[4], ppm_channels[5],
        ppm_channels[6], ppm_channels[7],
        rover_select_ppm, mode_str);
    stdio_flush();
}

// ── Main ──────────────────────────────────────────────────────────────────
int main(void) {
    stdio_init_all();

    // Wait up to 5 s for Jetson to open USB serial
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while (!stdio_usb_connected()) {
        if (to_ms_since_boot(get_absolute_time()) - t0 > 5000) break;
        sleep_ms(100);
    }
    printf("SLAVE ready  PPM:GP%d  nRF24:SPI0(GP%d-%d) CE:GP%d\n",
           PPM_OUTPUT_PIN, PIN_MISO, PIN_MOSI, PIN_CE);
    stdio_flush();

    ppm_init();

    // ── nRF24L01+ init ────────────────────────────────────────────────────
    spi_init(spi0, 1000000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    if (!radio.begin()) {
        printf("[ERROR] nRF24L01+ not found — check wiring\n");
        stdio_flush();
    }
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    radio.setPayloadSize(sizeof(RCPayload));
    radio.setAutoAck(false);
    radio.disableCRC();
    radio.openReadingPipe(1, NRF_ADDR);
    radio.startListening();   // RX mode — slave never transmits

    printf("nRF24 RX ready  ch=76  250kbps  payload=%d B\n", (int)sizeof(RCPayload));
    stdio_flush();

    while (true) {

        // ── nRF24 receive ─────────────────────────────────────────────────
        if (radio.available()) {
            RCPayload pl;
            radio.read(&pl, sizeof(pl));
            process_payload(pl);

            if (!rf_was_ok) {
                rf_was_ok = true;
                ppm_start();
                printf("[RF_LINK_OK]\n"); stdio_flush();
            }
        }

        // ── Jetson USB commands / heartbeat ───────────────────────────────
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            uint8_t b = (uint8_t)c;
            if (b == '<') {
                pi_rx_idx = 0; pi_in_packet = true;
            } else if (b == '>' && pi_in_packet) {
                pi_in_packet = false;
                pi_rx_buf[pi_rx_idx] = '\0';

                if (strncmp(pi_rx_buf, "HB:", 3) == 0) {
                    unsigned int n;
                    if (sscanf(pi_rx_buf + 3, "%u", &n) == 1) {
                        last_hb_us = time_us_32();
                        hb_active  = true;
                        printf("<HB:%u>\n", (n + 1) % 10000);
                        stdio_flush();
                    }
                } else if (strncmp(pi_rx_buf, "J:", 2) == 0) {
                    uint16_t ch[8];
                    int n = sscanf(pi_rx_buf + 2,
                        "%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu",
                        &ch[0], &ch[1], &ch[2], &ch[3],
                        &ch[4], &ch[5], &ch[6], &ch[7]);
                    if (n == 8) {
                        pi_throttle    = ch[0];
                        pi_steering    = ch[1];
                        last_pi_cmd_us = time_us_32();
                    }
                } else {
                    // Legacy 2-channel throttle/steering
                    int t, s;
                    if (sscanf(pi_rx_buf, "%d,%d", &t, &s) == 2 &&
                        t >= 800 && t <= 2200 && s >= 800 && s <= 2200) {
                        pi_throttle    = (uint16_t)t;
                        pi_steering    = (uint16_t)s;
                        last_pi_cmd_us = time_us_32();
                    }
                }
            } else if (pi_in_packet && pi_rx_idx < (uint8_t)(sizeof(pi_rx_buf) - 1)) {
                pi_rx_buf[pi_rx_idx++] = b;
            }
        }

        // ── RF link watchdog ──────────────────────────────────────────────
        uint32_t now = time_us_32();
        bool rf_ok = (last_rf_rx_us != 0) && (now - last_rf_rx_us < RF_TIMEOUT_US);

        if (rf_was_ok && !rf_ok) {
            rf_was_ok = false;
            ppm_stop();
            printf("[LINK_LOST]\n"); stdio_flush();
        }

    }
}
