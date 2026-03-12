/*
 * receiver/main.c
 * RP2040 — CC1101 receiver + PPM generator (Rover 2)
 *
 * Receives SBUS frames over CC1101 from the emitter rover,
 * decodes them to PPM for the local ESC/FC.
 *
 * Pinout:
 *   GP2   → PPM output  (to local ESC or FC)
 *   GP16  ← SPI0 MISO   (CC1101)
 *   GP17  → SPI0 CS     (CC1101)
 *   GP18  → SPI0 SCK    (CC1101)
 *   GP19  → SPI0 MOSI   (CC1101)
 *   GP20  ← CC1101 GDO0
 *   USB-C ↔ Jetson Nano (heartbeat + telemetry + autonomous commands)
 *
 * Failsafe: PPM stops if no CC1101 packet received for >500 ms OR
 *           Jetson heartbeat lost in AUTONOMOUS mode.
 *
 * UART telemetry design (CH: lines sent to Jetson 2):
 *   9 channels are output: CH1/CH2 are always the RAW decoded stick values
 *   (not gated to 1500 by mode logic), CH9 is the rover-selection switch.
 *   The Jetson applies its own independent gating; the RP2040 gates only
 *   the PPM motor output.  This mirrors the emitter firmware convention.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "../cc1101.h"

// ── PPM ───────────────────────────────────────────────────────────────────
#define PPM_OUTPUT_PIN  2
#define PPM_FRAME_US    20000
#define PPM_PULSE_US    300

// ── Mode switches ─────────────────────────────────────────────────────────
#define SWA_CH               2
#define SWB_CH               3
#define EMERGENCY_THRESHOLD  1700
#define AUTONOMOUS_THRESHOLD 1700

// ── Timeouts ──────────────────────────────────────────────────────────────
#define CC1101_LINK_TIMEOUT_US  500000   // 500 ms — stop PPM if RF link dies
#define HEARTBEAT_TIMEOUT_US    300000   // 300 ms — halt in AUTO if Jetson silent
#define PI_CMD_TIMEOUT_US       500000   // 500 ms — hold still if no new cmd
#define SLAVE_HB_TX_INTERVAL_US 500000   // 500 ms — heartbeat back to emitter

typedef enum { MODE_MANUAL, MODE_AUTONOMOUS, MODE_EMERGENCY } rover_mode_t;

// ── Globals ───────────────────────────────────────────────────────────────
static uint8_t  sbus_buffer[25];
static uint16_t ppm_channels[8];

static uint16_t rover_select_ppm = 1500;  // CH9: 1000=RV1 1500=RV2 2000=both
#define ROVER_SELECT_LOW   1250
#define ROVER_SELECT_HIGH  1750

static uint16_t pi_throttle    = 1500;
static uint16_t pi_steering    = 1500;
static uint32_t last_pi_cmd_us = 0;
static char     pi_rx_buf[64];   // expanded: was 32, now 64 for <J:ch1,...,ch8>
static uint8_t  pi_rx_idx      = 0;
static bool     pi_in_packet   = false;

/*
 * Jetson full 8-channel fallback (CC1101 primary, Jetson secondary).
 *
 * When the Jetson sends <J:ch1,...,ch8>, these values are stored here.
 * They are applied to PPM output ONLY when the CC1101 RF link is silent
 * for > CC1101_LINK_TIMEOUT_US (500 ms).  CC1101 packets always take priority.
 *
 * ── CC1101 reintegration guide ────────────────────────────────────────────
 * When the CC1101 RF link is stable again:
 *   1. Ensure the emitter RP2040 firmware is flashed and CC1101 wired (SPI0).
 *   2. The rf_ok flag (line ~270) will become true on the first received packet.
 *   3. rf_ok=true causes ppm_channels to be sourced from decode_sbus() instead
 *      of from jetson_channels — automatic, no firmware change required.
 *   4. The Jetson can keep sending <J:...> as a warm standby; it is ignored.
 * ──────────────────────────────────────────────────────────────────────────
 */
static uint16_t jetson_channels[8]   = {1500,1500,1500,1500,1500,1500,1500,1500};
static uint32_t last_jetson_cmd_us   = 0;
static uint32_t last_hb_us     = 0;
static bool     hb_active      = false;

static uint32_t last_rf_pkt_us = 0;   // last CC1101 packet timestamp
static bool     rf_was_ok      = false;
static uint32_t last_hb_tx_us  = 0;   // last slave→emitter HB transmit
static uint32_t last_marc_us   = 0;   // last MARCSTATE debug print

// ── PPM ISR ───────────────────────────────────────────────────────────────
static volatile bool ppm_active = false;
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

// ── SBUS decode + mode logic ──────────────────────────────────────────────
static long map_val(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void decode_sbus(void) {
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
    ppm_channels[1] = 3000 - ppm_channels[1];
    ppm_channels[2] = 3000 - ppm_channels[2];
    ppm_channels[3] = 3000 - ppm_channels[3];
    ppm_channels[6] = 3000 - ppm_channels[6];
    ppm_channels[7] = 3000 - ppm_channels[7];

    // CH9: rover-selection switch (same SBUS source as emitter)
    rover_select_ppm = (uint16_t)map_val(ch[8], 172, 1811, 1000, 2000);
    if (rover_select_ppm < 1000) rover_select_ppm = 1000;
    if (rover_select_ppm > 2000) rover_select_ppm = 2000;

    // Snapshot raw stick values BEFORE mode gating (see emitter for rationale).
    // Jetson 2 receives these raw values and applies its own independent gating.
    uint16_t raw_throttle = ppm_channels[0];
    uint16_t raw_steering = ppm_channels[1];

    // Mode logic — gates PPM motor output only, not UART telemetry
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
    } else {
        mode_str = "MANUAL";
    }

    // Telemetry to Jetson 2: 9 channels — CH1/CH2 raw (ungated) stick values,
    // CH3-CH8 switch/aux channels, CH9 rover-selection.
    printf("CH:%d,%d,%d,%d,%d,%d,%d,%d,%d MODE:%s\n",
        raw_throttle, raw_steering, ppm_channels[2], ppm_channels[3],
        ppm_channels[4], ppm_channels[5], ppm_channels[6], ppm_channels[7],
        rover_select_ppm, mode_str);
    stdio_flush();
}

// ── Main ──────────────────────────────────────────────────────────────────
int main(void) {
    stdio_init_all();

    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while (!stdio_usb_connected()) {
        if (to_ms_since_boot(get_absolute_time()) - t0 > 5000) break;
        sleep_ms(100);
    }
    printf("RECEIVER ready  PPM:GP%d  CC1101:SPI0\n", PPM_OUTPUT_PIN);

    ppm_init();
    cc1101_init(true);           // RX mode: after RX stay in RX
    cc1101_strobe(CC1101_SRX);   // start listening

    // Startup SPI sanity check
    // PARTNUM should always be 0x00, VERSION 0x14 for a genuine CC1101
    // MARCSTATE 0x0D = RX  (0x01 = IDLE means SRX strobe failed)
    {
        uint8_t pn   = cc1101_read_status(0x30);   // PARTNUM
        uint8_t ver  = cc1101_read_status(0x31);   // VERSION
        uint8_t marc = cc1101_read_status(CC1101_MARCSTATE);
        printf("CC1101 PN:%02X VER:%02X MARC:%02X  (expect PN:00 VER:14 MARC:0D)\n",
               pn, ver, marc);
        stdio_flush();
    }

    while (true) {

        // ── CC1101 receive ────────────────────────────────────────────────
        uint8_t pkt_type;
        uint8_t payload[25];
        if (cc1101_receive(&pkt_type, payload)) {
            last_rf_pkt_us = time_us_32();
            if (pkt_type == CC1101_PKT_SBUS) {
                memcpy(sbus_buffer, payload, 25);
                decode_sbus();

                // Send heartbeat back to emitter (rate-limited to 500 ms)
                uint32_t now_us = time_us_32();
                if (now_us - last_hb_tx_us >= SLAVE_HB_TX_INTERVAL_US) {
                    last_hb_tx_us = now_us;
                    uint8_t dummy[25] = {0};
                    if (cc1101_transmit(dummy, CC1101_PKT_HB_SLAVE)) {
                        printf("[RF_HB_TX]\n");
                    } else {
                        printf("[RF_HB_TX_FAIL]\n");
                    }
                    stdio_flush();
                    cc1101_strobe(CC1101_SRX);   // back to RX after TX
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
                } else if (strncmp(pi_rx_buf, "J:", 2) == 0) {
                    /* Full 8-channel Jetson command — CC1101 fallback.
                     * Format: <J:ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8>
                     * Applied to PPM only when CC1101 RF link is silent. */
                    uint16_t ch[8];
                    int n = sscanf(pi_rx_buf + 2,
                        "%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu",
                        &ch[0], &ch[1], &ch[2], &ch[3],
                        &ch[4], &ch[5], &ch[6], &ch[7]);
                    if (n == 8) {
                        for (int i = 0; i < 8; i++) {
                            if (ch[i] >= 800 && ch[i] <= 2200)
                                jetson_channels[i] = ch[i];
                        }
                        last_jetson_cmd_us = time_us_32();
                    }
                } else {
                    /* Legacy 2-channel throttle/steering (kept for compatibility) */
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

        // ── CC1101 state watchdog (every 5 s) ────────────────────────────
        uint32_t now = time_us_32();
        if (now - last_marc_us >= 5000000) {
            last_marc_us = now;
            uint8_t pn   = cc1101_read_status(0x30);
            uint8_t ver  = cc1101_read_status(0x31);
            uint8_t marc = cc1101_read_status(CC1101_MARCSTATE);
            uint8_t rxb  = cc1101_read_status(CC1101_RXBYTES);
            printf("[CC1101 PN:%02X VER:%02X MARC:%02X RXB:%02X]\n", pn, ver, marc, rxb);
            stdio_flush();
            // If chip fell out of RX, kick it back
            if ((marc & 0x1F) != 0x0D) {
                cc1101_strobe(CC1101_SIDLE);
                cc1101_strobe(CC1101_SFRX);
                cc1101_strobe(CC1101_SRX);
                printf("[CC1101 RECOVERED]\n");
                stdio_flush();
            }
        }

        // ── CC1101 / Jetson link watchdog + PPM source select ─────────────
        //
        // Priority 1: CC1101 RF link (ppm_channels filled by decode_sbus())
        // Priority 2: Jetson serial fallback (jetson_channels via <J:...>)
        //
        // When CC1101 is active (rf_ok), ppm_channels is already kept current
        // by decode_sbus() on each received SBUS packet.  When CC1101 is dead,
        // jetson_channels is copied to ppm_channels every loop so the PPM ISR
        // always sees fresh values.
        //
        // CC1101 reintegration: restore the RF link and rf_ok becomes true
        // automatically — no firmware change required (see jetson_channels decl).

        bool rf_ok = (last_rf_pkt_us != 0) &&
                     (now - last_rf_pkt_us < CC1101_LINK_TIMEOUT_US);
        bool hb_alive  = hb_active && (now - last_hb_us < HEARTBEAT_TIMEOUT_US);
        bool jetson_ok = hb_alive && (last_jetson_cmd_us != 0) &&
                         (now - last_jetson_cmd_us < PI_CMD_TIMEOUT_US);

        if (rf_ok) {
            // CC1101 primary: ppm_channels already updated by decode_sbus()
            if (!rf_was_ok) {
                rf_was_ok = true;
                ppm_start();
                printf("[RF_LINK_OK]\n"); stdio_flush();
            }
        } else if (jetson_ok) {
            // Jetson fallback: copy latest 8-channel command to PPM output
            memcpy(ppm_channels, jetson_channels, sizeof(ppm_channels));
            // Safety: apply emergency override exactly as decode_sbus() does
            // (mirrors the CC1101 path so safety behaviour is identical)
            if (ppm_channels[SWA_CH] < EMERGENCY_THRESHOLD) {
                ppm_channels[0] = 1500;   // throttle neutral
                ppm_channels[1] = 1500;   // steering neutral
            }
            if (!rf_was_ok) {
                rf_was_ok = true;
                ppm_start();
                printf("[JETSON_CTRL]\n"); stdio_flush();
            }
        } else {
            if (rf_was_ok) {
                rf_was_ok = false;
                ppm_stop();
                printf("[LINK_LOST]\n"); stdio_flush();
            }
        }
    }
}
