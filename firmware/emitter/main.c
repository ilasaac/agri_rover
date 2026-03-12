/*
 * emitter/main.c
 * RP2040 — SBUS decoder + PPM generator + CC1101 transmitter
 *
 * Receives SBUS from HM30 air unit, generates PPM for local ESC/FC,
 * and re-broadcasts the raw SBUS frame over CC1101 to the receiver rover.
 *
 * Pinout:
 *   GP5   ← SBUS input  (HM30 air unit FC/UART port, inverted 100kbaud 8E2)
 *   GP2   → PPM output  (to local ESC or FC)
 *   GP16  ← SPI0 MISO   (CC1101)
 *   GP17  → SPI0 CS     (CC1101)
 *   GP18  → SPI0 SCK    (CC1101)
 *   GP19  → SPI0 MOSI   (CC1101)
 *   GP20  ← CC1101 GDO0
 *   USB-C ↔ Jetson Nano (heartbeat + telemetry + autonomous commands)
 *
 * Failsafe: PPM stops if SBUS is lost for >100 ms.
 * Autonomous mode: Jetson overrides CH1/CH2 when SWB active + HB alive.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "../cc1101.h"

// ── SBUS ──────────────────────────────────────────────────────────────────
#define UART_SBUS       uart1
#define SBUS_RX_PIN     5
#define SBUS_BAUD       100000

// ── PPM ───────────────────────────────────────────────────────────────────
#define PPM_OUTPUT_PIN  2
#define PPM_FRAME_US    20000
#define PPM_PULSE_US    300

// ── Mode switches (0-indexed PPM channels) ────────────────────────────────
#define SWA_CH              2   // Emergency stop
#define SWB_CH              3   // Autonomous mode
#define EMERGENCY_THRESHOLD 1700
#define AUTONOMOUS_THRESHOLD 1700

// ── Timeouts ──────────────────────────────────────────────────────────────
#define SBUS_TIMEOUT_US      200000   //  200 ms — stop PPM if no SBUS
#define HEARTBEAT_TIMEOUT_US 300000   //  300 ms — halt in AUTO if Jetson silent
#define PI_CMD_TIMEOUT_US    500000   //  500 ms — hold still if no new cmd
#define CC1101_TX_MIN_US     70000    //   70 ms — one packet per window (4.8 kbps, 27 bytes = ~58 ms on-air)

typedef enum { MODE_MANUAL, MODE_AUTONOMOUS, MODE_EMERGENCY } rover_mode_t;

// ── Globals ───────────────────────────────────────────────────────────────
static uint8_t  sbus_buffer[25];
static uint8_t  buf_idx     = 0;
static uint32_t last_rx_us  = 0;

static uint16_t ppm_channels[8];
static uint16_t rover_select_ppm  = 1500;  // CH9: 1000=rover1 1500=rover2 2000=auto
#define ROVER_SELECT_LOW   1250
#define ROVER_SELECT_HIGH  1750

static uint16_t pi_throttle       = 1500;
static uint16_t pi_steering       = 1500;
static uint32_t last_pi_cmd_us    = 0;
static char     pi_rx_buf[32];
static uint8_t  pi_rx_idx         = 0;
static bool     pi_in_packet      = false;
static uint32_t last_hb_us        = 0;
static bool     hb_active         = false;
static uint32_t last_cc1101_tx_us    = 0;
static uint32_t last_cc1101_debug_us = 0;
static bool     failsafe_active      = false;

// ── PPM ISR ───────────────────────────────────────────────────────────────
static volatile bool ppm_active = false;
static bool          sbus_was_ok = false;
static uint16_t ppm_snap[8];
static int      ppm_ch       = 0;
static bool     ppm_in_pulse = true;

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

// ── SBUS helpers ─────────────────────────────────────────────────────────
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

    const uint8_t src[8] = {2, 0, 4, 5, 10, 11, 6, 7};
    for (int i = 0; i < 8; i++) {
        ppm_channels[i] = (uint16_t)map_val(ch[src[i]], 172, 1811, 1000, 2000);
        if (ppm_channels[i] < 800)  ppm_channels[i] = 800;
        if (ppm_channels[i] > 2200) ppm_channels[i] = 2200;
    }
    // Invert switch channels
    ppm_channels[1] = 3000 - ppm_channels[1];
    ppm_channels[2] = 3000 - ppm_channels[2];
    ppm_channels[3] = 3000 - ppm_channels[3];
    ppm_channels[6] = 3000 - ppm_channels[6];
    ppm_channels[7] = 3000 - ppm_channels[7];

    // CH9: rover selection switch (SBUS index 8)
    // 1000 = Rover 1 (master), 1500 = Rover 2 (slave), 2000 = both autonomous
    // Adjust ROVER_SELECT_SBUS_CH if your transmitter maps this switch differently.
    rover_select_ppm = (uint16_t)map_val(ch[8], 172, 1811, 1000, 2000);
    if (rover_select_ppm < 1000) rover_select_ppm = 1000;
    if (rover_select_ppm > 2000) rover_select_ppm = 2000;

    // Mode logic
    const char *mode_str;
    if (ppm_channels[SWA_CH] < EMERGENCY_THRESHOLD) {
        ppm_channels[0] = 1500; ppm_channels[1] = 1500;
        mode_str = "EMERGENCY";
    } else if (ppm_channels[SWB_CH] > AUTONOMOUS_THRESHOLD) {
        bool hb_alive = hb_active && (time_us_32() - last_hb_us < HEARTBEAT_TIMEOUT_US);
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
        // Rover 2 selected: this rover (master) outputs neutral PPM.
        // The master Jetson relays the actual stick channels to the slave.
        ppm_channels[0] = 1500;
        ppm_channels[1] = 1500;
        mode_str = "RELAY";
    } else {
        mode_str = "MANUAL";
    }

    // Telemetry to Jetson
    printf("CH:%d,%d,%d,%d,%d,%d,%d,%d,%d MODE:%s\n",
        ppm_channels[0], ppm_channels[1], ppm_channels[2], ppm_channels[3],
        ppm_channels[4], ppm_channels[5], ppm_channels[6], ppm_channels[7],
        rover_select_ppm, mode_str);
    stdio_flush();

    bool frame_lost = sbus_buffer[23] & (1 << 2);
    bool failsafe   = sbus_buffer[23] & (1 << 3);

    if (frame_lost) {
        printf("[FRAME_LOST]\n");
        stdio_flush();
    }

    if (failsafe && !failsafe_active) {
        failsafe_active = true;
        ppm_stop();
        printf("[FAILSAFE]\n");
        stdio_flush();
    } else if (!failsafe && failsafe_active) {
        failsafe_active = false;
        ppm_start();
        printf("[FAILSAFE_CLEARED]\n");
        stdio_flush();
    }

    // Transmit raw SBUS over CC1101 (rate-limited to SBUS frame rate)
    uint32_t now = time_us_32();
    if (now - last_cc1101_tx_us >= CC1101_TX_MIN_US) {
        last_cc1101_tx_us = now;
        static bool first_tx = true;
        if (first_tx) {
            first_tx = false;
            printf("[CC1101_TX_ATTEMPT]\n");
            stdio_flush();
        }
        if (!cc1101_transmit(sbus_buffer, CC1101_PKT_SBUS)) {
            printf("[CC1101_TX_FAIL]\n");
            stdio_flush();
        } else {
            static bool first_tx_ok = true;
            if (first_tx_ok) {
                first_tx_ok = false;
                uint8_t marc = cc1101_read_status(CC1101_MARCSTATE);
                printf("[CC1101_TX_OK MARC_AFTER:%02X]\n", marc);
                stdio_flush();
            }
            // Open a short RX window for the slave heartbeat reply.
            // 5 ms is enough to catch an immediate reply; the full 58 ms
            // propagation window is not used here to keep the main loop
            // responsive.  Increase back to 70000 when the CC1101 link is
            // operational and slave reply timing is confirmed.
            cc1101_strobe(CC1101_SRX);
            uint32_t rx_t0 = time_us_32();
            while (time_us_32() - rx_t0 < 5000) {
                uint8_t rtype, rpl[25];
                if (cc1101_receive(&rtype, rpl)) {
                    if (rtype == CC1101_PKT_HB_SLAVE) {
                        printf("[RF_HB_RX]\n");
                        stdio_flush();
                    }
                    break;
                }
            }
            cc1101_strobe(CC1101_SIDLE);
        }
    }
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
    printf("EMITTER ready  SBUS:GP%d  PPM:GP%d  CC1101:SPI0\n",
           SBUS_RX_PIN, PPM_OUTPUT_PIN);

    // SBUS UART — inverted, 8E2, 100 kbaud
    uart_init(UART_SBUS, SBUS_BAUD);
    gpio_set_function(SBUS_RX_PIN, GPIO_FUNC_UART);
    gpio_set_inover(SBUS_RX_PIN, GPIO_OVERRIDE_INVERT);
    uart_set_format(UART_SBUS, 8, 2, UART_PARITY_EVEN);

    ppm_init();
    cc1101_init(false);  // TX mode: after TX→IDLE

    // SPI sanity check — PARTNUM=0x00, VERSION=0x14, MARCSTATE=0x01 (IDLE)
    {
        uint8_t pn   = cc1101_read_status(0x30);
        uint8_t ver  = cc1101_read_status(0x31);
        uint8_t marc = cc1101_read_status(CC1101_MARCSTATE);
        printf("CC1101 PN:%02X VER:%02X MARC:%02X  (expect PN:00 VER:14 MARC:01)\n",
               pn, ver, marc);
        stdio_flush();
    }

    while (true) {

        // ── SBUS receive ──────────────────────────────────────────────────
        if (uart_is_readable(UART_SBUS)) {
            uint32_t now = time_us_32();
            if (now - last_rx_us > 3000) buf_idx = 0;
            last_rx_us = now;

            uint8_t b = uart_getc(UART_SBUS);
            if (buf_idx == 0 && b != 0x0F) continue;
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
            if (b == '<') { pi_rx_idx = 0; pi_in_packet = true; }
            else if (b == '>' && pi_in_packet) {
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
                } else {
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

        // ── CC1101 periodic diagnostic (every 10 s) ───────────────────────
        {
            uint32_t now_d = time_us_32();
            if (now_d - last_cc1101_debug_us >= 10000000) {
                last_cc1101_debug_us = now_d;
                uint8_t pn   = cc1101_read_status(0x30);
                uint8_t ver  = cc1101_read_status(0x31);
                uint8_t marc = cc1101_read_status(CC1101_MARCSTATE);
                printf("CC1101 PN:%02X VER:%02X MARC:%02X\n", pn, ver, marc);
                stdio_flush();
            }
        }

        // ── SBUS watchdog / PPM control ───────────────────────────────────
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
