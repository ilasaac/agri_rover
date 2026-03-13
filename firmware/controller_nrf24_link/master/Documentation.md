# controller_nrf24_link — Master Firmware

## Overview

Runs on the **controller-side RP2040** (Raspberry Pi Pico). Receives SBUS from the HM30 air unit, decodes it into 9 PPM channels, applies mode gating to its own PPM output, and broadcasts the raw channel values to the slave rover over an nRF24L01+ RF link at 50 Hz.

The RF link is **one-way**: the master transmits only. The slave never replies.

---

## Hardware Pinout

| GPIO | Direction | Function |
|------|-----------|----------|
| GP5  | IN        | SBUS from HM30 air unit (inverted, 100 kbaud, 8E2) |
| GP2  | OUT       | PPM output to local ESC or flight controller |
| GP16 | IN        | SPI0 MISO — nRF24L01+ |
| GP17 | OUT       | SPI0 CSN  — nRF24L01+ |
| GP18 | OUT       | SPI0 SCK  — nRF24L01+ |
| GP19 | OUT       | SPI0 MOSI — nRF24L01+ |
| GP20 | OUT       | nRF24L01+ CE |
| USB-C | BIDIR   | CDC-ACM serial to Jetson Nano |

---

## Building

Open the `master/` folder in VS Code with the Raspberry Pi Pico extension. The folder is self-contained: it has its own `pico_sdk_import.cmake`.

```
cd master
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
ninja
```

Flash `master_fw.uf2` to the Pico via BOOTSEL mode.

---

## SBUS Decoding

- **Format**: 100 kbaud, 8E2, inverted logic (hardware invert on GP5)
- **Frame**: 25 bytes — start byte `0x0F`, 22 data bytes, flag byte, end byte
- **Valid end bytes**: `0x00`, `0x04`, `0x14`, `0x24`, `0x34`
- **Channel mapping** (SBUS raw index → PPM output index):

| PPM index | SBUS src | Function |
|-----------|----------|----------|
| 0 | ch[2] | Throttle |
| 1 | ch[0] | Steering (inverted) |
| 2 | ch[4] | SWA — Emergency stop (inverted) |
| 3 | ch[5] | SWB — Autonomous mode (inverted) |
| 4 | ch[10] | Aux CH5 |
| 5 | ch[11] | Aux CH6 (inverted) |
| 6 | ch[6]  | Aux CH7 (inverted) |
| 7 | ch[7]  | Aux CH8 |
| 8 | ch[8]  | Rover select (CH9) — not inverted |

- **Scaling**: SBUS raw (172–1811) → PPM (1000–2000), clamped to 800–2200
- **Frame sync**: gap >3 ms between bytes resets the buffer index

---

## Mode Logic

Evaluated on every valid SBUS frame. Controls the **local PPM output only**. The nRF24 payload always carries raw, ungated values so the slave applies its own independent gating.

| Mode | Condition | Local thr/str output |
|------|-----------|----------------------|
| `EMERGENCY`    | SWA (CH3) < 1700 | 1500 / 1500 |
| `AUTO-NO-HB`   | SWB (CH4) > 1700, Jetson HB missing | 1500 / 1500 |
| `AUTO-TIMEOUT` | SWB (CH4) > 1700, HB alive, no `<J:>` cmd within 500 ms | 1500 / 1500 |
| `AUTONOMOUS`   | SWB (CH4) > 1700, HB alive, fresh `<J:>` cmd | Jetson thr/str |
| `RELAY`        | CH9 in (1250, 1750] | 1500 / 1500 (slave drives) |
| `MANUAL`       | Default | Raw SBUS stick values |

**RELAY mode** means the rover-select switch points to the slave rover. The master neutralises its own motors and passes raw sticks to the slave via nRF24.

---

## nRF24L01+ Configuration

| Parameter | Value |
|-----------|-------|
| Channel | 76 (2.476 GHz) |
| Data rate | 250 kbps |
| PA level | MAX |
| CRC | Disabled |
| Auto-ACK | Disabled |
| Payload size | 18 bytes (static) |
| TX rate | 50 Hz (every 20 ms) |
| Address | `"RV_TX"` |
| Mode | TX only (`stopListening`) |

### RF Payload — RCPayload struct

```
Offset  Field              Notes
 0–1    channel[0]  thr    Raw throttle (before mode gating), 1000–2000
 2–3    channel[1]  str    Raw steering (before mode gating), 1000–2000
 4–5    channel[2]  SWA    Emergency stop switch, 1000–2000
 6–7    channel[3]  SWB    Autonomous mode switch, 1000–2000
 8–9    channel[4]  CH5    Auxiliary, 1000–2000
10–11   channel[5]  CH6    Auxiliary, 1000–2000
12–13   channel[6]  CH7    Auxiliary, 1000–2000
14–15   channel[7]  CH8    Auxiliary, 1000–2000
16–17   channel[8]  SEL    Rover-select (CH9), 1000–2000
```

Total: 18 bytes. All values are raw — the slave gates its own motors independently.

---

## USB / Jetson Protocol

### Output (master → Jetson)

| Message | Trigger | Description |
|---------|---------|-------------|
| `CH:thr,str,ch3,ch4,ch5,ch6,ch7,ch8,ch9 MODE:str\n` | Every valid SBUS frame | Telemetry. CH1 and CH2 are always raw (ungated). |
| `[SBUS_OK]` | SBUS signal appears after absence | Link recovery |
| `[SBUS_LOST]` | No SBUS byte for 200 ms | Link loss |
| `[FAILSAFE]` | SBUS failsafe flag set | Transmitter lost, PPM stopped |
| `[FAILSAFE_CLEARED]` | SBUS failsafe flag cleared | PPM resumed |
| `[FRAME_LOST]` | SBUS frame-lost flag set | Per-frame warning |
| `<HB:N+1>` | On receipt of `<HB:N>` | Heartbeat echo |

### Input (Jetson → master)

| Command | Format | Description |
|---------|--------|-------------|
| Heartbeat | `<HB:N>` | N is 0–9999, echoed as N+1. Must arrive within 300 ms to keep AUTONOMOUS mode alive. |
| Autonomous cmd | `<J:c1,c2,c3,c4,c5,c6,c7,c8>` | 8-channel override. c1=throttle, c2=steering. Used in AUTONOMOUS mode. Must arrive within 500 ms. |
| Legacy override | `<T,S>` | 2-channel throttle/steering, 800–2200. |

---

## Safety Timeouts

| Timeout | Value | Effect |
|---------|-------|--------|
| SBUS watchdog | 200 ms | PPM output stops; `[SBUS_LOST]` printed |
| Heartbeat | 300 ms | AUTONOMOUS mode drops to AUTO-NO-HB (neutral) |
| Jetson cmd | 500 ms | AUTONOMOUS mode drops to AUTO-TIMEOUT (neutral) |
| SBUS failsafe | HW flag | PPM output stops; `[FAILSAFE]` printed |

All timeouts are independent and stack. For example, SBUS loss also kills the nRF24 TX (no new packets are sent).

---

## Boot Sequence

1. `stdio_init_all()` — USB CDC-ACM init
2. Wait up to 5 s for Jetson to open the serial port
3. Print `MASTER ready  SBUS:GP5  PPM:GP2  nRF24:SPI0(GP16-19) CE:GP20`
4. Init SBUS UART (uart1, GP5, 100 kbaud, 8E2, inverted)
5. Init PPM (GP2, idle high)
6. Init SPI0 at 1 MHz
7. Init nRF24 → print `nRF24 TX ready  ch=76  250kbps  payload=18 B`
8. Enter main loop

If the nRF24 is not detected, `[ERROR] nRF24L01+ not found — check wiring` is printed but execution continues (local PPM still works).

---

## Troubleshooting

| Symptom | Likely cause |
|---------|-------------|
| No `[SBUS_OK]` after power-on | SBUS wiring or inversion; confirm HM30 is on and TX is bound |
| MODE always MANUAL despite switch | SWA/SWB switch channels not mapped correctly on transmitter |
| `[FAILSAFE]` at startup | Transmitter off or not bound to HM30 |
| nRF24 error at boot | SPI wiring; check GP16–20 and 3.3 V supply to module |
| Slave not responding | Confirm both modules on channel 76, same payload size, no CRC |
