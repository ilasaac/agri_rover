#pragma once
/*
 * cc1101.h — CC1101 sub-GHz radio driver for RP2040
 *
 * SPI0 wiring (both emitter and receiver):
 *   GP16  MISO
 *   GP17  CS   (active-low)
 *   GP18  SCK
 *   GP19  MOSI
 *   GP20  GDO0 (packet-ready / TX-done indicator)
 *
 * RF config: 433.0 MHz · 4.8 kbps · GFSK · fixed 27-byte packets · CRC
 *
 * Packet layout (27 bytes, CC1101 appends 2 status bytes in RX FIFO):
 *   [0]     type       CC1101_PKT_SBUS (0x01) or CC1101_PKT_HB (0x02)
 *   [1]     source_id  0x01 = emitter
 *   [2-26]  payload    25 bytes (raw SBUS frame, XOR-encrypted)
 *
 * Basic security: payload XOR-encrypted with APP_KEY.
 * Change APP_KEY to the same value on both ends.
 */

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// ── Pins ──────────────────────────────────────────────────────────────────
#define CC1101_SPI      spi0
#define CC1101_MISO     16
#define CC1101_CS       17
#define CC1101_SCK      18
#define CC1101_MOSI     19
#define CC1101_GDO0     20

// ── Register addresses ────────────────────────────────────────────────────
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
#define CC1101_PKTLEN   0x06
#define CC1101_PKTCTRL1 0x07
#define CC1101_PKTCTRL0 0x08
#define CC1101_CHANNR   0x0A
#define CC1101_FSCTRL1  0x0B
#define CC1101_FSCTRL0  0x0C
#define CC1101_FREQ2    0x0D
#define CC1101_FREQ1    0x0E
#define CC1101_FREQ0    0x0F
#define CC1101_MDMCFG4  0x10
#define CC1101_MDMCFG3  0x11
#define CC1101_MDMCFG2  0x12
#define CC1101_MDMCFG1  0x13
#define CC1101_MDMCFG0  0x14
#define CC1101_DEVIATN  0x15
#define CC1101_MCSM1    0x17
#define CC1101_MCSM0    0x18
#define CC1101_FOCCFG   0x19
#define CC1101_BSCFG    0x1A
#define CC1101_AGCCTRL2 0x1B
#define CC1101_AGCCTRL1 0x1C
#define CC1101_AGCCTRL0 0x1D
#define CC1101_FREND1   0x21
#define CC1101_FREND0   0x22
#define CC1101_FSCAL3   0x23
#define CC1101_FSCAL2   0x24
#define CC1101_FSCAL1   0x25
#define CC1101_FSCAL0   0x26

// ── Status registers (burst-read prefix 0xC0) ─────────────────────────────
#define CC1101_MARCSTATE 0x35
#define CC1101_RXBYTES   0x3B
#define CC1101_TXBYTES   0x3A

// ── Strobe commands ───────────────────────────────────────────────────────
#define CC1101_SRES  0x30
#define CC1101_SRX   0x34
#define CC1101_STX   0x35
#define CC1101_SIDLE 0x36
#define CC1101_SFRX  0x3A
#define CC1101_SFTX  0x3B

// ── Packet constants ──────────────────────────────────────────────────────
#define CC1101_PACKET_LEN  27   // 2 header + 25 payload
#define CC1101_PKT_SBUS      0x01
#define CC1101_PKT_HB        0x02
#define CC1101_PKT_HB_SLAVE  0x03   // receiver→emitter heartbeat

// ── Change this key on both ends ──────────────────────────────────────────
static const uint8_t CC1101_APP_KEY[16] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
};

// Saved rx_persist flag for recovery re-init
static bool _cc1101_rx_persist = false;

// ── SPI primitives ────────────────────────────────────────────────────────
static inline void _cs_lo(void) { gpio_put(CC1101_CS, 0); }
static inline void _cs_hi(void) { gpio_put(CC1101_CS, 1); }
static inline void _wait_miso(void) {
    uint32_t t = time_us_32();
    while (gpio_get(CC1101_MISO)) {
        if (time_us_32() - t > 200000) break;  // 200 ms — covers oscillator startup
        tight_loop_contents();
    }
}

static inline uint8_t cc1101_strobe(uint8_t cmd) {
    uint8_t s;
    _cs_lo(); _wait_miso();
    spi_write_read_blocking(CC1101_SPI, &cmd, &s, 1);
    _cs_hi();
    return s;
}

static inline void cc1101_write_reg(uint8_t addr, uint8_t val) {
    uint8_t b[2] = { addr, val };
    _cs_lo(); _wait_miso();
    spi_write_blocking(CC1101_SPI, b, 2);
    _cs_hi();
}

static inline uint8_t cc1101_read_reg(uint8_t addr) {
    uint8_t cmd = addr | 0x80, val;
    _cs_lo(); _wait_miso();
    spi_write_blocking(CC1101_SPI, &cmd, 1);
    spi_read_blocking(CC1101_SPI, 0xFF, &val, 1);
    _cs_hi();
    return val;
}

static inline uint8_t cc1101_read_status(uint8_t addr) {
    uint8_t cmd = addr | 0xC0, val;
    _cs_lo(); _wait_miso();
    spi_write_blocking(CC1101_SPI, &cmd, 1);
    spi_read_blocking(CC1101_SPI, 0xFF, &val, 1);
    _cs_hi();
    return val;
}

static inline void cc1101_write_fifo(const uint8_t *data, uint8_t len) {
    uint8_t cmd = 0x3F | 0x40;  // burst write TXFIFO
    _cs_lo(); _wait_miso();
    spi_write_blocking(CC1101_SPI, &cmd, 1);
    spi_write_blocking(CC1101_SPI, data, len);
    _cs_hi();
}

static inline void cc1101_read_fifo(uint8_t *data, uint8_t len) {
    uint8_t cmd = 0x3F | 0xC0;  // burst read RXFIFO
    _cs_lo(); _wait_miso();
    spi_write_blocking(CC1101_SPI, &cmd, 1);
    spi_read_blocking(CC1101_SPI, 0xFF, data, len);
    _cs_hi();
}

// ── XOR encrypt / decrypt payload (in-place) ─────────────────────────────
static inline void cc1101_xor(uint8_t *buf, uint8_t len) {
    for (uint8_t i = 0; i < len; i++)
        buf[i] ^= CC1101_APP_KEY[i % 16];
}

// ── Reset ─────────────────────────────────────────────────────────────────
static inline void cc1101_reset(void) {
    _cs_hi(); sleep_us(5);
    _cs_lo(); sleep_us(10);
    _cs_hi(); sleep_us(41);
    _cs_lo(); _wait_miso();
    uint8_t sres = CC1101_SRES;
    spi_write_blocking(CC1101_SPI, &sres, 1);
    _cs_hi(); sleep_ms(5);
}

// Apply all RF configuration registers (no SPI/GPIO init, no reset).
// Called from cc1101_init() and from the TX-fail recovery path.
static inline void _cc1101_apply_regs(bool rx_persist) {
    cc1101_write_reg(CC1101_IOCFG0,   0x06); // GDO0: asserts on sync, de-asserts end of pkt
    cc1101_write_reg(CC1101_FIFOTHR,  0x47);
    cc1101_write_reg(CC1101_PKTLEN,   CC1101_PACKET_LEN);
    cc1101_write_reg(CC1101_PKTCTRL1, 0x04); // append RSSI+LQI status bytes
    cc1101_write_reg(CC1101_PKTCTRL0, 0x04); // fixed length, CRC on, no whitening
    cc1101_write_reg(CC1101_CHANNR,   0x00);
    cc1101_write_reg(CC1101_FSCTRL1,  0x06);
    cc1101_write_reg(CC1101_FSCTRL0,  0x00);
    cc1101_write_reg(CC1101_FREQ2,    0x10); // 433.0 MHz
    cc1101_write_reg(CC1101_FREQ1,    0xA7);
    cc1101_write_reg(CC1101_FREQ0,    0x62);
    cc1101_write_reg(CC1101_MDMCFG4,  0xC7); // BW = 102 kHz, DRATE_E=7
    cc1101_write_reg(CC1101_MDMCFG3,  0x83); // 4.8 kbps, DRATE_M=131
    cc1101_write_reg(CC1101_MDMCFG2,  0x13); // GFSK, 16/16 sync word
    cc1101_write_reg(CC1101_MDMCFG1,  0x22); // 2 preamble bytes
    cc1101_write_reg(CC1101_MDMCFG0,  0xF8);
    cc1101_write_reg(CC1101_DEVIATN,  0x47); // ±38 kHz deviation
    cc1101_write_reg(CC1101_MCSM1,    rx_persist ? 0x0C : 0x00);
    cc1101_write_reg(CC1101_MCSM0,    0x18); // autocal IDLE→RX/TX
    cc1101_write_reg(CC1101_FOCCFG,   0x16);
    cc1101_write_reg(CC1101_BSCFG,    0x6C);
    cc1101_write_reg(CC1101_AGCCTRL2, 0x43);
    cc1101_write_reg(CC1101_AGCCTRL1, 0x40);
    cc1101_write_reg(CC1101_AGCCTRL0, 0x91);
    cc1101_write_reg(CC1101_FREND1,   0x56);
    cc1101_write_reg(CC1101_FREND0,   0x10);
    cc1101_write_reg(CC1101_FSCAL3,   0xE9);
    cc1101_write_reg(CC1101_FSCAL2,   0x2A);
    cc1101_write_reg(CC1101_FSCAL1,   0x00);
    cc1101_write_reg(CC1101_FSCAL0,   0x1F);

    // PATABLE[0] = 0xC0 → +10 dBm at 433 MHz (TI datasheet Table 39)
    // FREND0 bits[2:0]=0 → STX uses PATABLE[0]
    {
        uint8_t cmd = 0x3E | 0x40;   // burst write PATABLE address
        uint8_t pa  = 0xC0;
        _cs_lo(); _wait_miso();
        spi_write_blocking(CC1101_SPI, &cmd, 1);
        spi_write_blocking(CC1101_SPI, &pa,  1);
        _cs_hi();
    }
}

/*
 * cc1101_init()
 *   rx_persist = false  (emitter):  after TX→IDLE, after RX→IDLE
 *   rx_persist = true   (receiver): after RX stay in RX, after TX→IDLE
 */
static inline void cc1101_init(bool rx_persist) {
    _cc1101_rx_persist = rx_persist;

    spi_init(CC1101_SPI, 8000000);
    spi_set_format(CC1101_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(CC1101_MISO, GPIO_FUNC_SPI);
    gpio_set_function(CC1101_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(CC1101_MOSI, GPIO_FUNC_SPI);
    gpio_init(CC1101_CS);   gpio_set_dir(CC1101_CS,   GPIO_OUT); _cs_hi();
    gpio_init(CC1101_GDO0); gpio_set_dir(CC1101_GDO0, GPIO_IN);

    cc1101_reset();
    // 433.0 MHz · 4.8 kbps · GFSK · fixed 27-byte packets · CRC on
    _cc1101_apply_regs(rx_persist);
}

/*
 * Transmit a 25-byte payload (raw SBUS frame).
 * Blocks until TX complete or 100 ms timeout.
 * Returns false on timeout (TX stuck).
 *
 * After 5 consecutive failures the CC1101 is hard-reset and
 * re-initialised so a bad power-on state cannot cause permanent failure.
 */
static inline bool cc1101_transmit(const uint8_t *sbus_buf, uint8_t pkt_type) {
    static uint8_t _tx_fail_count = 0;

    uint8_t pkt[CC1101_PACKET_LEN];
    pkt[0] = pkt_type;
    pkt[1] = 0x01;                          // source_id = emitter
    memcpy(&pkt[2], sbus_buf, 25);
    cc1101_xor(&pkt[2], 25);               // encrypt payload

    cc1101_strobe(CC1101_SIDLE);
    cc1101_strobe(CC1101_SFTX);
    cc1101_write_fifo(pkt, CC1101_PACKET_LEN);
    cc1101_strobe(CC1101_STX);

    uint32_t t0 = time_us_32();
    while (cc1101_read_status(CC1101_TXBYTES) != 0) {
        if (time_us_32() - t0 > 70000) {
            cc1101_strobe(CC1101_SIDLE);
            _tx_fail_count++;
            if (_tx_fail_count >= 5) {
                // Hard-reset the chip and re-apply all RF registers
                _tx_fail_count = 0;
                cc1101_reset();
                _cc1101_apply_regs(_cc1101_rx_persist);
            }
            return false;
        }
    }
    _tx_fail_count = 0;   // clear on success
    return true;
}

/*
 * Poll for a received packet.
 * Returns true if a valid CRC packet is available.
 * On success: *pkt_type and payload[25] are filled.
 */
static inline bool cc1101_receive(uint8_t *pkt_type, uint8_t *payload) {
    uint8_t rxb = cc1101_read_status(CC1101_RXBYTES);

    // Handle RX FIFO overflow (bit 7 set)
    if (rxb & 0x80) {
        cc1101_strobe(CC1101_SIDLE);
        cc1101_strobe(CC1101_SFRX);
        cc1101_strobe(CC1101_SRX);
        return false;
    }

    // Need full packet + 2 appended status bytes
    if (rxb < CC1101_PACKET_LEN + 2) return false;

    uint8_t buf[CC1101_PACKET_LEN + 2];
    cc1101_read_fifo(buf, CC1101_PACKET_LEN + 2);

    // Check CRC_OK bit in second status byte
    // IMPORTANT: SFRX must only be issued from IDLE (datasheet §10.4).
    // With rx_persist (RXOFF_MODE=11) the chip stays in RX after reception,
    // so we must go IDLE first, then flush, then restart RX.
    if (!(buf[CC1101_PACKET_LEN + 1] & 0x80)) {
        cc1101_strobe(CC1101_SIDLE);
        cc1101_strobe(CC1101_SFRX);
        cc1101_strobe(CC1101_SRX);
        return false;
    }

    *pkt_type = buf[0];
    cc1101_xor(&buf[2], 25);   // decrypt payload
    memcpy(payload, &buf[2], 25);
    return true;
}
