/*
 * controller_nrf24_link/master/main.cpp
 * RP2040 — SBUS decoder + PPM generator + nRF24L01+ transmitter
 *
 * Receives SBUS from the HM30 air unit on GP5, generates PPM for the local
 * ESC/FC on GP2, and broadcasts raw 9-channel PPM to the slave over nRF24L01+.
 * Link is one-way: master transmits only, slave does not respond.
 *
 * Pinout:
 *   GP5   ← SBUS input  (HM30 air unit FC/UART port, inverted 100 kbaud 8E2)
 *   GP2   → PPM output  (to local ESC or FC)
 *   GP16  ← SPI0 MISO   (nRF24L01+)
 *   GP17  → SPI0 CSN    (nRF24L01+)
 *   GP18  → SPI0 SCK    (nRF24L01+)
 *   GP19  → SPI0 MOSI   (nRF24L01+)
 *   GP20  → nRF24L01+ CE
 *   USB-C ↔ Jetson Nano (heartbeat + telemetry + autonomous commands)
 *
 * USB protocol (115200 baud CDC-ACM):
 *   Output "CH:thr,str,ch3,...,ch9 MODE:str\n"  every SBUS frame
 *   Output "[SBUS_OK]", "[SBUS_LOST]", "[FAILSAFE]", "[FAILSAFE_CLEARED]"
 *   Input  <HB:N>           Jetson heartbeat — echoed as <HB:N+1>
 *   Input  <T,S>            2-channel throttle/steering override (legacy)
 *   Input  <J:c1,...,c8>    8-channel override (autonomous mode)
 *
 * nRF24L01+ config:
 *   Channel 76, 250 kbps, PA_MAX, 1-byte CRC, no auto-ack (one-way)
 *   Payload: RCPayload — 9 × uint16_t = 18 bytes
 *   TX rate: 50 Hz (every 20 ms), rate-limited inside SBUS parse
 *
 * Safety timeouts:
 *   SBUS_TIMEOUT_US      200 ms — PPM stops if SBUS silent
 *   HEARTBEAT_TIMEOUT_US 300 ms — halts in AUTO if Jetson HB silent
 *   PI_CMD_TIMEOUT_US    500 ms — neutral if no autonomous command
 *
 * Mode logic (SWA = ppm_channels[2] = CH3, SWB = ppm_channels[3] = CH4):
 *   EMERGENCY   SWA < 1700  → thr/str gated to 1500 locally and on slave
 *   AUTONOMOUS  SWB > 1700 + HB alive + cmd → Jetson drives thr/str
 *   AUTO-NO-HB  SWB > 1700 but no Jetson HB → neutral
 *   AUTO-TIMEOUT SWB > 1700 + HB alive but cmd timed out → neutral
 *   RELAY       CH9 in [1250,1750] → local motors neutral, raw relayed
 *   MANUAL      default
 *
 * UART telemetry design:
 *   CH1/CH2 in the USB output are always RAW SBUS stick values (before any
 *   mode gating) so the Jetson and slave can each apply independent gating.
 *   The nRF24 payload also carries raw values — the slave gates its own motors.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/spi.h"
#include "RF24.h"

// ── SBUS ──────────────────────────────────────────────────────────────────
#define UART_SBUS    uart1
#define SBUS_RX_PIN  5
#define SBUS_BAUD    100000

// ── PPM ───────────────────────────────────────────────────────────────────
#define PPM_OUTPUT_PIN  2
#define PPM_FRAME_US    20000
#define PPM_PULSE_US    300

// ── Mode switches (0-indexed positions in the 9-channel PPM array) ─────────
#define SWA_CH               2    // Emergency stop  — ppm_channels[2] = CH3
#define SWB_CH               3    // Autonomous mode — ppm_channels[3] = CH4
#define EMERGENCY_THRESHOLD  1700
#define AUTONOMOUS_THRESHOLD 1700

// ── Rover selection (CH9, index 8) ────────────────────────────────────────
#define ROVER_SELECT_LOW   1250
#define ROVER_SELECT_HIGH  1750

// ── Timeouts ──────────────────────────────────────────────────────────────
#define SBUS_TIMEOUT_US      200000   // 200 ms — PPM stops if SBUS silent
#define HEARTBEAT_TIMEOUT_US 300000   // 300 ms — halt in AUTO if Jetson HB silent
#define PI_CMD_TIMEOUT_US    500000   // 500 ms — hold neutral if no autonomous cmd
#define NRF_TX_INTERVAL_US    20000   //  20 ms →  50 Hz TX rate

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
static uint8_t  sbus_buffer[25];
static uint8_t  buf_idx           = 0;
static uint32_t last_rx_us        = 0;

static uint16_t ppm_channels[8];
static uint16_t rover_select_ppm  = 1500;

static uint16_t pi_throttle       = 1500;
static uint16_t pi_steering       = 1500;
static uint32_t last_pi_cmd_us    = 0;
static char     pi_rx_buf[64];
static uint8_t  pi_rx_idx         = 0;
static bool     pi_in_packet      = false;
static uint32_t last_hb_us        = 0;
static bool     hb_active         = false;
static uint32_t last_nrf_tx_us    = 0;
static bool     failsafe_active   = false;

// ── PPM ISR ───────────────────────────────────────────────────────────────
static volatile bool ppm_active  = false;
static bool          sbus_was_ok = false;
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

static void ppm_stop(void)  { ppm_active = false; gpio_put(PPM_OUTPUT_PIN, 1); }
static void ppm_start(void) {
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

// ── SBUS decode ───────────────────────────────────────────────────────────
static long map_val(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void parse_and_transmit_sbus(void) {
    uint16_t ch[16];

    ch[0]  = ((sbus_buffer[1]        | sbus_buffer[2]  << 8)                           & 0x07FF);
    ch[1]  = ((sbus_buffer[2]  >>  3 | sbus_buffer[3]  << 5)                           & 0x07FF);
    ch[2]  = ((sbus_buffer[3]  >>  6 | sbus_buffer[4]  << 2  | sbus_buffer[5]  << 10)  & 0x07FF);
    ch[3]  = ((sbus_buffer[5]  >>  1 | sbus_buffer[6]  << 7)                           & 0x07FF);
    ch[4]  = ((sbus_buffer[6]  >>  4 | sbus_buffer[7]  << 4)                           & 0x07FF);
    ch[5]  = ((sbus_buffer[7]  >>  7 | sbus_buffer[8]  << 1  | sbus_buffer[9]  << 9)   & 0x07FF);
    ch[6]  = ((sbus_buffer[9]  >>  2 | sbus_buffer[10] << 6)                           & 0x07FF);
    ch[7]  = ((sbus_buffer[10] >>  5 | sbus_buffer[11] << 3)                           & 0x07FF);
    ch[8]  = ((sbus_buffer[12]       | sbus_buffer[13] << 8)                           & 0x07FF);
    ch[9]  = ((sbus_buffer[13] >>  3 | sbus_buffer[14] << 5)                           & 0x07FF);
    ch[10] = ((sbus_buffer[14] >>  6 | sbus_buffer[15] << 2  | sbus_buffer[16] << 10)  & 0x07FF);
    ch[11] = ((sbus_buffer[16] >>  1 | sbus_buffer[17] << 7)                           & 0x07FF);
    ch[12] = ((sbus_buffer[17] >>  4 | sbus_buffer[18] << 4)                           & 0x07FF);
    ch[13] = ((sbus_buffer[18] >>  7 | sbus_buffer[19] << 1  | sbus_buffer[20] << 9)   & 0x07FF);
    ch[14] = ((sbus_buffer[20] >>  2 | sbus_buffer[21] << 6)                           & 0x07FF);
    ch[15] = ((sbus_buffer[21] >>  5 | sbus_buffer[22] << 3)                           & 0x07FF);

    // Map SBUS raw (172-1811) to PPM (1000-2000) using same channel order as cc1101 firmware
    const uint8_t src[8] = {2, 0, 4, 5, 10, 11, 6, 7};
    for (int i = 0; i < 8; i++) {
        ppm_channels[i] = (uint16_t)map_val(ch[src[i]], 172, 1811, 1000, 2000);
        if (ppm_channels[i] < 800)  ppm_channels[i] = 800;
        if (ppm_channels[i] > 2200) ppm_channels[i] = 2200;
    }
    // Invert switch channels (active-low on this transmitter)
    ppm_channels[1] = 3000 - ppm_channels[1];
    ppm_channels[2] = 3000 - ppm_channels[2];
    ppm_channels[3] = 3000 - ppm_channels[3];
    ppm_channels[6] = 3000 - ppm_channels[6];
    ppm_channels[7] = 3000 - ppm_channels[7];

    // CH9: rover-selection switch
    rover_select_ppm = (uint16_t)map_val(ch[8], 172, 1811, 1000, 2000);
    if (rover_select_ppm < 1000) rover_select_ppm = 1000;
    if (rover_select_ppm > 2000) rover_select_ppm = 2000;

    // Snapshot raw stick values BEFORE mode gating.
    // Local PPM output (motor control) uses gated values for safety.
    // nRF24 payload carries raw values so the slave applies its own gating.
    uint16_t raw_throttle = ppm_channels[0];
    uint16_t raw_steering = ppm_channels[1];

    // ── Mode logic — gates ppm_channels[0/1] for local PPM only ─────────
    bool hb_alive = hb_active && (time_us_32() - last_hb_us < HEARTBEAT_TIMEOUT_US);
    const char *mode_str;

    if (ppm_channels[SWA_CH] < EMERGENCY_THRESHOLD) {
        ppm_channels[0] = 1500; ppm_channels[1] = 1500;
        mode_str = "EMERGENCY";
    } else if (ppm_channels[SWB_CH] > AUTONOMOUS_THRESHOLD) {
        if (!hb_alive) {
            ppm_channels[0] = 1500; ppm_channels[1] = 1500;
            mode_str = "AUTO-NO-HB";
        } else if (time_us_32() - last_pi_cmd_us < PI_CMD_TIMEOUT_US) {
            ppm_channels[0] = pi_throttle;
            ppm_channels[1] = pi_steering;
            mode_str = "AUTONOMOUS";
        } else {
            ppm_channels[0] = 1500; ppm_channels[1] = 1500;
            mode_str = "AUTO-TIMEOUT";
        }
    } else if (rover_select_ppm > ROVER_SELECT_LOW && rover_select_ppm <= ROVER_SELECT_HIGH) {
        // Rover 2 selected: neutral this rover's motors; raw sticks go to slave.
        ppm_channels[0] = 1500;
        ppm_channels[1] = 1500;
        mode_str = "RELAY";
    } else {
        mode_str = "MANUAL";
    }

    // ── nRF24 TX — send raw 9-channel PPM at 50 Hz ───────────────────────
    uint32_t now = time_us_32();
    if (now - last_nrf_tx_us >= NRF_TX_INTERVAL_US) {
        last_nrf_tx_us = now;
        RCPayload payload;
        payload.channel[0] = raw_throttle;
        payload.channel[1] = raw_steering;
        for (int i = 2; i < 8; i++) payload.channel[i] = ppm_channels[i];
        payload.channel[8] = rover_select_ppm;
        radio.write(&payload, sizeof(payload));
    }

    // ── USB telemetry to Jetson — CH1/CH2 always raw (not gated) ─────────
    printf("CH:%d,%d,%d,%d,%d,%d,%d,%d,%d MODE:%s\n",
        raw_throttle, raw_steering,
        ppm_channels[2], ppm_channels[3], ppm_channels[4], ppm_channels[5],
        ppm_channels[6], ppm_channels[7],
        rover_select_ppm, mode_str);
    stdio_flush();

    // ── SBUS failsafe / frame-lost flags ─────────────────────────────────
    bool frame_lost = sbus_buffer[23] & (1 << 2);
    bool failsafe   = sbus_buffer[23] & (1 << 3);

    if (frame_lost) { printf("[FRAME_LOST]\n"); stdio_flush(); }

    if (failsafe && !failsafe_active) {
        failsafe_active = true;
        ppm_stop();
        printf("[FAILSAFE]\n"); stdio_flush();
    } else if (!failsafe && failsafe_active) {
        failsafe_active = false;
        ppm_start();
        printf("[FAILSAFE_CLEARED]\n"); stdio_flush();
    }
}

// ── Main ──────────────────────────────────────────────────────────────────
int main(void) {
    stdio_init_all();

    // Wait up to 5 s for Jetson to open USB serial before proceeding
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while (!stdio_usb_connected()) {
        if (to_ms_since_boot(get_absolute_time()) - t0 > 5000) break;
        sleep_ms(100);
    }
    printf("MASTER ready  SBUS:GP%d  PPM:GP%d  nRF24:SPI0(GP%d-%d) CE:GP%d\n",
           SBUS_RX_PIN, PPM_OUTPUT_PIN, PIN_MISO, PIN_MOSI, PIN_CE);

    // ── SBUS UART — inverted, 8E2, 100 kbaud ─────────────────────────────
    uart_init(UART_SBUS, SBUS_BAUD);
    gpio_set_function(SBUS_RX_PIN, GPIO_FUNC_UART);
    gpio_set_inover(SBUS_RX_PIN, GPIO_OVERRIDE_INVERT);
    uart_set_format(UART_SBUS, 8, 2, UART_PARITY_EVEN);

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
    radio.enableCRC();     // filters noise — both sides must match
    radio.openWritingPipe(NRF_ADDR);
    radio.stopListening();   // TX mode

    printf("nRF24 TX ready  ch=76  250kbps  payload=%d B\n", (int)sizeof(RCPayload));
    stdio_flush();

    while (true) {

        // ── SBUS receive ──────────────────────────────────────────────────
        if (uart_is_readable(UART_SBUS)) {
            uint32_t now = time_us_32();
            if (now - last_rx_us > 3000) buf_idx = 0;   // gap resets frame sync
            last_rx_us = now;

            uint8_t b = uart_getc(UART_SBUS);
            if (buf_idx == 0 && b != 0x0F) continue;    // wait for start byte
            sbus_buffer[buf_idx++] = b;

            if (buf_idx == 25) {
                buf_idx = 0;
                uint8_t end = sbus_buffer[24];
                if (end == 0x00 || end == 0x04 || end == 0x14 ||
                    end == 0x24 || end == 0x34) {
                    parse_and_transmit_sbus();
                }
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
                    // 8-channel autonomous override
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

        // ── SBUS watchdog — PPM follows SBUS signal presence ──────────────
        uint32_t now = time_us_32();
        bool sbus_ok = (last_rx_us != 0) && (now - last_rx_us < SBUS_TIMEOUT_US);
        if (sbus_ok && !sbus_was_ok) {
            sbus_was_ok = true;
            if (!failsafe_active) ppm_start();
            printf("[SBUS_OK]\n"); stdio_flush();
        } else if (!sbus_ok && sbus_was_ok) {
            sbus_was_ok = false;
            ppm_stop();
            printf("[SBUS_LOST]\n"); stdio_flush();
        }
    }
}
