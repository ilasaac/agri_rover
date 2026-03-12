# Rover Run Options

Each rover runs `rover/main.py` with a combination of real or simulated GPS and RP2040.
The table below lists all supported configurations.

---

## Component Modes

| Component | Real | Simulated |
|-----------|------|-----------|
| **GPS** | Two serial NMEA receivers on `/dev/ttyUSB0` + `/dev/ttyUSB1` | `simulator/sim.py` generates physics-based NMEA over pty |
| **RP2040** | USB-UART on `/dev/ttyACM0` (real emitter/receiver firmware) | `rp2040_emulator.py` (RV2) or `sim.py --mode emulate` generates `CH:` lines over pty |

---

## All Configurations

### Option A — Fully Real

Both rovers running on physical hardware. No simulation involved.

**Rover 1:**
```bash
ROVER_ID=1 python rover/main.py
```

**Rover 2:**
```bash
ROVER_ID=2 python rover/main.py
```

Uses `UART_PORT=/dev/ttyACM0`, `GPS_PRIMARY_PORT=/dev/ttyUSB0`, `GPS_SECONDARY_PORT=/dev/ttyUSB1` by default.

---

### Option B — Fully Simulated (both rovers)

No physical hardware needed. Both rovers use physics simulation for GPS and RP2040.

**Terminal 1 — RV1 simulator:**
```bash
python simulator/sim.py --rover 1 --mode emulate
```

**Terminal 2 — RV1 rover** (use pty paths printed by sim):
```bash
ROVER_ID=1 UART_PORT=<pty> GPS_PRIMARY_PORT=<pty> GPS_SECONDARY_PORT=<pty> python rover/main.py
```

**Terminal 3 — RV2 simulator:**
```bash
python simulator/sim.py --rover 2 --mode emulate
```

**Terminal 4 — RV2 rover** (use pty paths printed by sim):
```bash
ROVER_ID=2 UART_PORT=<pty> GPS_PRIMARY_PORT=<pty> GPS_SECONDARY_PORT=<pty> python rover/main.py
```

---

### Option C — Simulated GPS, Real RP2040 (both rovers)

Real RP2040 hardware on both rovers, but GPS replaced by simulation.
`sim.py` runs in **proxy mode** — it bridges the real RP2040 UART while generating virtual GPS pty ports.

**Terminal 1 — RV1 simulator (proxy):**
```bash
python simulator/sim.py --rover 1 --mode proxy --real-port /dev/ttyACM0
```

**Terminal 2 — RV1 rover:**
```bash
ROVER_ID=1 python rover/main.py --sim-gps --real-port /dev/ttyACM0
```

**Terminal 3 — RV2 simulator (proxy):**
```bash
python simulator/sim.py --rover 2 --mode proxy --real-port /dev/ttyACM0
```

**Terminal 4 — RV2 rover:**
```bash
ROVER_ID=2 python rover/main.py --sim-gps --real-port /dev/ttyACM0
```

---

### ★ Option D — Simulated GPS, Real RP2040 (RV1) + Simulated RP2040 via UDP (RV2) ★

> **Current test bench configuration.**

RV1 has a real RP2040 emitter connected. RV2 has no physical RF receiver — instead
`rp2040_emulator.py` listens for the `RC_CHANNELS` MAVLink packets that RV1 broadcasts
over UDP and re-emits them as `CH:` UART lines, exactly as the receiver firmware would.
Both rovers use simulated GPS.

```
RV1 (real RP2040 emitter)
  └─ sim.py proxy mode  →  virtual GPS pty
  └─ rover/main.py      →  reads real UART + sim GPS
       │
       │  UDP RC_CHANNELS (port 14550 → 14560 relay or broadcast)
       ▼
RV2 (no hardware)
  └─ sim.py emulate mode → virtual GPS pty + embedded UDP RC listener
  └─ rover/main.py       →  reads emulator UART + sim GPS
```

### Single command per rover (recommended)

`rover/main.py` launches all required subprocesses automatically.

**Rover 1:**
```bash
ROVER_ID=1 python rover/main.py --sim-gps --real-port /dev/ttyACM0
```
Spawns `sim.py --rover 1 --mode proxy` internally, which bridges the real RP2040 UART
and outputs virtual GPS pty paths.

**Rover 2:**
```bash
ROVER_ID=2 python rover/main.py --sim-gps
```
Spawns `sim.py --rover 2 --mode emulate` internally. The emulate mode includes an
embedded MAVLink UDP listener (port 14550) that receives `RC_CHANNELS` from RV1 and
drives the virtual RP2040 UART — no separate emulator process needed.

---

### Manual multi-terminal breakdown

Use this if you need to inspect or restart individual components separately.

**Terminal 1 — RV1 simulator (proxy, bridges real RP2040):**
```bash
python simulator/sim.py --rover 1 --mode proxy --real-port /dev/ttyACM0
```

**Terminal 2 — RV1 rover** (use pty paths printed by sim):
```bash
ROVER_ID=1 UART_PORT=<pty> GPS_PRIMARY_PORT=<pty> GPS_SECONDARY_PORT=<pty> python rover/main.py
```

**Terminal 3 — RV2 GPS simulator + RC emulator:**
```bash
python simulator/sim.py --rover 2 --mode emulate
```

**Terminal 4 — RV2 rover** (use pty paths printed by sim):
```bash
ROVER_ID=2 UART_PORT=<pty> GPS_PRIMARY_PORT=<pty> GPS_SECONDARY_PORT=<pty> python rover/main.py
```

> **Note:** If you need `rp2040_emulator.py` as a standalone process instead of the
> embedded listener in `sim.py` (e.g. real GPS on RV2), use Option F instead.

---

### Option E — Real GPS, Simulated RP2040 (both rovers)

Useful when GPS hardware is available but RF link (CC1101) is not.

**RV1:**
```bash
python tools/rp2040_emulator.py --listen-port 14550 --master-sysid 1
# Then:
ROVER_ID=1 UART_PORT=<emulator_pty> python rover/main.py
```

**RV2:**
```bash
python tools/rp2040_emulator.py --listen-port 14550 --master-sysid 1
# Then:
ROVER_ID=2 UART_PORT=<emulator_pty> python rover/main.py
```

---

### Option F — Real GPS, Real RP2040 (RV1) + Simulated RP2040 via UDP (RV2)

Like the test bench (Option D) but using real GPS receivers on both rovers.

**RV1:**
```bash
ROVER_ID=1 python rover/main.py
```

**Terminal — RV2 RP2040 emulator:**
```bash
python tools/rp2040_emulator.py --listen-port 14550 --master-sysid 1
```

**RV2:**
```bash
ROVER_ID=2 UART_PORT=<emulator_pty> python rover/main.py
```

---

## Quick Reference

| Option | RV1 GPS | RV1 RP2040 | RV2 GPS | RV2 RP2040 |
|--------|---------|------------|---------|------------|
| A — Fully Real | Real | Real | Real | Real |
| B — Fully Simulated | Sim | Sim | Sim | Sim |
| C — Sim GPS, Real RP2040 | Sim | Real | Sim | Real |
| **D — Test Bench ★** | **Sim** | **Real** | **Sim** | **UDP Emulator** |
| E — Real GPS, Sim RP2040 | Real | Sim | Real | Sim |
| F — Real GPS, RV2 UDP Emulator | Real | Real | Real | UDP Emulator |

---

## Monitoring

Regardless of configuration, the MAVLink monitor can be run alongside any option:

```bash
python monitor/monitor.py
```

Listens on UDP 14550 (RV1) and 14551 (RV2). To debug raw UDP packets:

```bash
python tools/catch_hb.py
```
