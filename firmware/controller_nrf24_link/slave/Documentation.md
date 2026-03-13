# controller_nrf24_link — Slave Firmware

## Overview

Runs on the **rover-side RP2040** (Raspberry Pi Pico). Receives raw 9-channel PPM values from the master over nRF24L01+, applies mode gating independently, and drives a PPM output to the rover's ESC or flight controller.

The RF link is **one-way**: the slave only receives. It never transmits back to the master.

**Safety rule**: if the RF link is silent for more than 500 ms, the PPM output stops immediately and the rover does not move. There is no Jetson-direct fallback — the RC link must be present for the rover to operate.

---

## Hardware Pinout

| GPIO | Direction | Function |
|------|-----------|----------|
| GP2  | OUT       | PPM output to local ESC or flight controller |
| GP16 | IN        | SPI0 MISO — nRF24L01+ |
| GP17 | OUT       | SPI0 CSN  — nRF24L01+ |
| GP18 | OUT       | SPI0 SCK  — nRF24L01+ |
| GP19 | OUT       | SPI0 MOSI — nRF24L01+ |
| GP20 | OUT       | nRF24L01+ CE |
| USB-C | BIDIR   | CDC-ACM serial to Jetson Nano |

No SBUS UART is needed on the slave — channel values arrive via the RF payload.

---

## Building

Open the `slave/` folder in VS Code with the Raspberry Pi Pico extension. The folder is self-contained: it has its own `pico_sdk_import.cmake`.

```
cd slave
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
ninja
```

Flash `slave_fw.uf2` to the Pico via BOOTSEL mode.

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
| Address | `"RV_TX"` (reading pipe 1) |
| Mode | RX only (`startListening` — never transmits) |

### RF Payload — RCPayload struct

```
Offset  Field              Notes
 0–1    channel[0]  thr    Raw throttle from master, 1000–2000
 2–3    channel[1]  str    Raw steering from master, 1000–2000
 4–5    channel[2]  SWA    Emergency stop switch, 1000–2000
 6–7    channel[3]  SWB    Autonomous mode switch, 1000–2000
 8–9    channel[4]  CH5    Auxiliary, 1000–2000
10–11   channel[5]  CH6    Auxiliary, 1000–2000
12–13   channel[6]  CH7    Auxiliary, 1000–2000
14–15   channel[7]  CH8    Auxiliary, 1000–2000
16–17   channel[8]  SEL    Rover-select (CH9), 1000–2000
```

Total: 18 bytes. Values are raw (ungated). The slave applies its own mode gating.

---

## Mode Logic

Evaluated on every received RF packet. Controls the local PPM output.

| Mode | Condition | Local thr/str output |
|------|-----------|----------------------|
| `EMERGENCY`    | SWA (CH3) < 1700 | 1500 / 1500 |
| `AUTO-NO-HB`   | SWB (CH4) > 1700, Jetson HB missing | 1500 / 1500 |
| `AUTO-TIMEOUT` | SWB (CH4) > 1700, HB alive, no `<J:>` cmd within 500 ms | 1500 / 1500 |
| `AUTONOMOUS`   | SWB (CH4) > 1700, HB alive, fresh `<J:>` cmd | Jetson thr/str |
| `STANDBY`      | CH9 outside (1250, 1750] (master rover selected) | 1500 / 1500 |
| `MANUAL`       | CH9 in (1250, 1750] (this rover selected) | Raw thr/str from master |

**Rover selection**: CH9 in (1250, 1750] means the transmitter's relay switch is pointing at this rover. The master puts its own motors to neutral (RELAY mode) and passes raw sticks. When CH9 is outside this range, the master is driving its own rover and this rover stands by.

**EMERGENCY always wins** — it gates the rover to neutral regardless of rover selection or autonomous commands.

---

## RF Link Watchdog

| State | Condition | Action |
|-------|-----------|--------|
| `[RF_LINK_OK]` | First packet received after silence | PPM started |
| `[LINK_LOST]`  | No packet for 500 ms | PPM stopped immediately |

Once `[LINK_LOST]` is declared, the rover is fully stopped. PPM does not restart until a new RF packet arrives. There is no override path — the RC link is mandatory for movement.

---

## USB / Jetson Protocol

### Output (slave → Jetson)

| Message | Trigger | Description |
|---------|---------|-------------|
| `CH:thr,str,ch3,ch4,ch5,ch6,ch7,ch8,ch9 MODE:str\n` | Every received RF packet | Telemetry. CH1 and CH2 are always raw (ungated). |
| `[RF_LINK_OK]` | RF link recovered | PPM started |
| `[LINK_LOST]` | RF silent >500 ms | PPM stopped |
| `<HB:N+1>` | On receipt of `<HB:N>` | Heartbeat echo |

### Input (Jetson → slave)

| Command | Format | Description |
|---------|--------|-------------|
| Heartbeat | `<HB:N>` | N is 0–9999, echoed as N+1. Must arrive within 300 ms to keep AUTONOMOUS mode alive. |
| Autonomous cmd | `<J:c1,c2,c3,c4,c5,c6,c7,c8>` | 8-channel override. c1=throttle, c2=steering. Only applied in AUTONOMOUS mode (while RF is active and SWB > 1700). |
| Legacy override | `<T,S>` | 2-channel throttle/steering, 800–2200. Same conditions as `<J:>`. |

> **Note**: `<J:>` and `<T,S>` commands are stored and applied only when the RF link is active and the mode logic selects AUTONOMOUS. They have no effect when the RF link is lost.

---

## Safety Timeouts

| Timeout | Value | Effect |
|---------|-------|--------|
| RF link watchdog | 500 ms | PPM output stops; `[LINK_LOST]` printed |
| Heartbeat | 300 ms | AUTONOMOUS mode drops to AUTO-NO-HB (neutral) |
| Jetson cmd | 500 ms | AUTONOMOUS mode drops to AUTO-TIMEOUT (neutral) |

---

## PPM Output

- **Frame period**: 20 ms (50 Hz)
- **Pulse width**: 300 µs sync pulse per channel
- **Range**: 1000–2000 µs channel values (clamped to 800–2200 by master before TX)
- **Idle state**: GP2 held high (no PPM when RF is lost or at boot)
- **Channels**: 8 channels per frame

The PPM generator runs on a hardware alarm (timer ISR). It snapshots `ppm_channels[]` at the start of each 20 ms frame, so mode gating changes take effect on the next frame boundary.

---

## Boot Sequence

1. `stdio_init_all()` — USB CDC-ACM init
2. Wait up to 5 s for Jetson to open the serial port
3. Print `SLAVE ready  PPM:GP2  nRF24:SPI0(GP16-19) CE:GP20`
4. Init PPM (GP2, idle high)
5. Init SPI0 at 1 MHz
6. Init nRF24 → `startListening()` → print `nRF24 RX ready  ch=76  250kbps  payload=18 B`
7. Enter main loop (PPM stays off until first RF packet)

If the nRF24 is not detected, `[ERROR] nRF24L01+ not found — check wiring` is printed but execution continues.

---

## Troubleshooting

| Symptom | Likely cause |
|---------|-------------|
| No `[RF_LINK_OK]` after power-on | Master not powered; nRF24 wiring; channel/address mismatch |
| `[LINK_LOST]` immediately after `[RF_LINK_OK]` | RF interference or range; try reducing distance |
| MODE always `STANDBY` | CH9 rover-select switch not in (1250, 1750] range |
| MODE always `EMERGENCY` | SWA switch on transmitter is in emergency position (< 1700) |
| AUTONOMOUS mode not activating | SWB > 1700 but Jetson HB or `<J:>` commands not arriving in time |
| PPM not driving ESC | RF link lost; check `[RF_LINK_OK]` / `[LINK_LOST]` messages |
| nRF24 error at boot | SPI wiring; check GP16–20 and 3.3 V supply to module |
