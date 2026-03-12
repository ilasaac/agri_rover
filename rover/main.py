"""
agri_rover/rover/main.py
Single rover controller — runs on Jetson/RPi for each rover.

Identity:
  ROVER_ID env var (1 or 2).  Defaults to 1.
  Rover 1 = master (emitter RP2040), Rover 2 = slave (receiver RP2040).

What it does:
  - Reads RP2040 UART: CH: lines → PPM channels, SBUS/mode status
  - Reads dual GPS (NMEA pty or real serial) → lat/lon/heading via RTK baseline
  - Sends MAVLink UDP to GCS (MK32 tablet at SIYI default 192.168.144.20:14550):
      HEARTBEAT, SYS_STATUS, RC_CHANNELS, GLOBAL_POSITION_INT,
      NAMED_VALUE_FLOAT (TANK, TEMP, HUMID, PRESSURE)
  - Receives MAVLink from GCS:
      RC_CHANNELS_OVERRIDE → forwarded to RP2040 as PPM
      COMMAND_LONG (arm/disarm/set_mode)

Usage:
  ROVER_ID=1 python rover/main.py
  ROVER_ID=2 python rover/main.py

Environment variables (override defaults):
  ROVER_ID          1 or 2
  UART_PORT         e.g. /dev/ttyACM0
  GPS_PRIMARY_PORT  e.g. /dev/ttyUSB0
  GPS_SECONDARY_PORT e.g. /dev/ttyUSB1
  GCS_HOST          e.g. 192.168.144.20
  GCS_PORT          e.g. 14550
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import serial

os.environ['MAVLINK20'] = '1'   # force MAVLink v2 frames

try:
    from pymavlink import mavutil
except ImportError:
    raise ImportError("pip install pymavlink")

# ─── Configuration ────────────────────────────────────────────────────────────

ROVER_ID: int = int(os.environ.get("ROVER_ID", "1"))

# Serial ports
UART_PORT:            str = os.environ.get("UART_PORT",            "/dev/ttyACM0")
UART_BAUD:            int = 115200
GPS_PRIMARY_PORT:     str = os.environ.get("GPS_PRIMARY_PORT",     "/dev/ttyUSB0")
GPS_SECONDARY_PORT:   str = os.environ.get("GPS_SECONDARY_PORT",   "/dev/ttyUSB1")
GPS_BAUD:             int = 115200

# GCS (MK32 tablet via SIYI HM30 or direct UDP)
GCS_HOST:             str = os.environ.get("GCS_HOST",             "127.0.0.1")
GCS_PORT:             int = int(os.environ.get("GCS_PORT",         "14550"))
MAVLINK_BIND_PORT:    int = 14550 + (ROVER_ID - 1)   # rover1=14550, rover2=14551

# Direct RC relay to RV2 machine (RV1 only) — avoids relying on WiFi broadcast
RELAY_HOST:           str = os.environ.get("RELAY_HOST", "")   # RV2 machine IP
RELAY_PORT:           int = 14560                               # rp2040_emulator listen port (separate from GCS port 14550)

# PPM / channel constants
PPM_MIN    = 1000
PPM_CENTER = 1500
PPM_MAX    = 2000

CH_THROTTLE   = 0
CH_STEERING   = 1
CH_EMERGENCY  = 4   # SWA (low = emergency)
CH_MODE       = 5   # SWB (high = autonomous)
CH_ROVER_SEL  = 8   # CH9 — 3-state rover selection

ROVER_SELECT_LOW  = 1250
ROVER_SELECT_HIGH = 1750

EMERGENCY_THRESHOLD  = 1700
AUTONOMOUS_THRESHOLD = 1700

# MAVLink identity
MAV_SYSTEM_ID    = ROVER_ID          # sysid 1 or 2
MAV_COMPONENT_ID = 1                 # MAV_COMP_ID_AUTOPILOT1
MAV_GCS_SYSID    = 255
MAV_TYPE         = 10                # MAV_TYPE_GROUND_ROVER
MAV_AUTOPILOT    = 3                 # MAV_AUTOPILOT_ARDUPILOTMEGA

# Telemetry rates
HZ_HEARTBEAT  = 1
HZ_POSITION   = 5
HZ_RC         = 10
HZ_SENSORS    = 1

CONTROL_HZ    = 20
HB_TIMEOUT_S  = 3.0

# ─── GPS fix constants ────────────────────────────────────────────────────────

FIX_INVALID = 0
FIX_GPS     = 1
FIX_DGPS    = 2
FIX_RTK_FIX = 4
FIX_RTK_FLT = 5

_ACCURACY_FALLBACK = {
    FIX_INVALID: 99.9,
    FIX_GPS:      1.50,
    FIX_DGPS:     0.50,
    FIX_RTK_FLT:  0.35,
    FIX_RTK_FIX:  0.02,
}

# ─── Shared state ─────────────────────────────────────────────────────────────

@dataclass
class RoverState:
    # GPS
    lat:          float = 0.0
    lon:          float = 0.0
    alt_m:        float = 0.0
    heading_deg:  float = 0.0
    fix_quality:  int   = FIX_INVALID
    h_accuracy_m: float = 99.9
    num_sats:     int   = 0
    baseline_m:   float = 0.0

    # RP2040 channels (what rover is currently outputting as PPM)
    ppm_channels: list[int] = field(default_factory=lambda: [PPM_CENTER] * 8)
    rc_channels:  list[int] = field(default_factory=lambda: [PPM_CENTER] * 9)
    sbus_ok:      Optional[bool] = None
    is_armed:     bool = False
    is_autonomous: bool = False
    is_emergency:  bool = False
    rover_select:  int  = PPM_MIN   # CH9 value

    # Sensors (stub values — replace with real sensors)
    tank_pct:     float = 0.0
    temp_c:       float = 25.0
    humid_pct:    float = 50.0
    pressure_hpa: float = 1013.25
    battery_mv:   int   = 0
    battery_pct:  int   = -1

    # Heartbeat from RP2040
    uart_hb_ok:   bool  = False
    uart_hb_age:  Optional[float] = None

    # Timestamp of last GCS RC_CHANNELS_OVERRIDE — used to prioritise GCS
    # joystick over RP2040 UART output when relaying channels to Rover 2.
    last_rc_override_t: float = 0.0

state = RoverState()
state_lock = threading.Lock()

# ─── UART bridge ──────────────────────────────────────────────────────────────

class UARTBridge:
    def __init__(self):
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._running = False
        self._hb_counter   = 0
        self._hb_expected  = -1
        self._hb_last_rx   = 0.0
        self._hb_ever_rx   = False
        self._hb_last_sent = 0.0

    def connect(self) -> None:
        self._ser = serial.Serial(UART_PORT, UART_BAUD, timeout=0.1)
        self._ser.reset_input_buffer()
        self._running = True
        threading.Thread(target=self._recv_loop, daemon=True, name="uart-recv").start()

    def close(self) -> None:
        self._running = False
        if self._ser and self._ser.is_open:
            self._ser.close()

    def send_ppm(self, channels: list[int]) -> None:
        # Rover 1 (emitter RP2040): legacy <T,S> — firmware only reads first 2 channels
        # Rover 2 (receiver RP2040): <J:ch1,...,ch8> — CC1101 fallback path in firmware
        if ROVER_ID == 2:
            packet = "<J:" + ",".join(str(v) for v in channels[:8]) + ">"
        else:
            packet = "<" + ",".join(str(v) for v in channels[:2]) + ">"
        self._write(packet.encode())

    def send_heartbeat(self) -> None:
        now = time.time()
        if now - self._hb_last_sent < 0.1:
            return
        self._hb_counter  = (self._hb_counter + 1) % 10000
        self._hb_expected = (self._hb_counter + 1) % 10000
        self._write(f"<HB:{self._hb_counter}>".encode())
        self._hb_last_sent = now

    @property
    def heartbeat_ok(self) -> bool:
        if not self._hb_ever_rx:
            return False
        return (time.time() - self._hb_last_rx) < HB_TIMEOUT_S

    @property
    def heartbeat_age(self) -> Optional[float]:
        if not self._hb_ever_rx:
            return None
        return time.time() - self._hb_last_rx

    def _write(self, data: bytes) -> None:
        try:
            if self._ser and self._ser.is_open:
                self._ser.write(data)
        except serial.SerialException:
            pass

    def _recv_loop(self) -> None:
        _log_path = f"/tmp/rv{ROVER_ID}_uart.log"
        while self._running:
            try:
                raw = self._ser.readline()
                if raw:
                    line = raw.decode("utf-8", errors="ignore").strip()
                    if line:
                        try:
                            # Write while file is under 8 KB; recreates after deletion
                            if not os.path.exists(_log_path) or \
                                    os.path.getsize(_log_path) < 8192:
                                with open(_log_path, "a") as _f:
                                    _f.write(line + "\n")
                        except Exception:
                            pass
                        self._parse_line(line)
            except Exception:
                time.sleep(0.01)

    def _parse_line(self, line: str) -> None:
        if line.startswith("CH:"):
            self._parse_channels(line[3:])
        elif line in ("[SBUS_LOST]", "[FAILSAFE]", "[RF_LINK_LOST]"):
            with state_lock:
                state.sbus_ok = False
        elif line in ("[SBUS_OK]", "[FAILSAFE_CLEARED]", "[RF_LINK_OK]"):
            with state_lock:
                state.sbus_ok = True
        elif line.startswith("<HB:") and line.endswith(">"):
            # Any valid echo from the RP2040 confirms it is alive.
            # Strict expected-number matching races with proxy latency.
            try:
                int(line[4:-1])   # validate it is a number
                self._hb_last_rx = time.time()
                self._hb_ever_rx = True
            except ValueError:
                pass

    def _parse_channels(self, data: str) -> None:
        try:
            parts   = data.split(" MODE:", 1)
            ch_part = parts[0]
            fw_mode = parts[1].strip() if len(parts) > 1 else ""

            vals = [int(v.strip()) for v in ch_part.split(",")]
            if len(vals) < 8:
                return

            rover_sel = vals[CH_ROVER_SEL] if len(vals) > CH_ROVER_SEL else PPM_MIN

            # Trust the RP2040 firmware's own mode string — it knows its state
            # better than we can infer from raw channel values (thresholds in the
            # firmware may differ from what we assume).
            # "AUTO-TIMEOUT" = rover was in autonomous mode but timed out waiting
            # for commands; firmware locks CH1/CH2 to 1500 for safety.
            is_emergency  = (fw_mode == "EMERGENCY")
            is_autonomous = ("AUTO" in fw_mode)   # AUTO, AUTO-TIMEOUT, AUTONOMOUS …

            # Gate throttle/steering by rover selection (CH9)
            # CH9 ≤ 1250 → RV1 only;  1250 < CH9 ≤ 1750 → RV2 only;  CH9 > 1750 → both
            # AUTO / EMERGENCY: joystick blocked
            if is_autonomous or is_emergency:
                active = False
            elif ROVER_ID == 1:
                active = (rover_sel <= ROVER_SELECT_LOW) or (rover_sel > ROVER_SELECT_HIGH)
            else:
                active = rover_sel > ROVER_SELECT_LOW

            effective = list(vals)
            if not active:
                effective[CH_THROTTLE] = PPM_CENTER
                effective[CH_STEERING] = PPM_CENTER

            now = time.monotonic()
            with state_lock:
                if now - state.last_rc_override_t > 0.5:
                    state.rc_channels = vals
                state.ppm_channels  = effective[:8]
                state.sbus_ok       = True
                state.is_emergency  = is_emergency
                state.is_autonomous = is_autonomous
                state.rover_select  = rover_sel
        except ValueError:
            pass


uart = UARTBridge()

# ─── GPS reader ───────────────────────────────────────────────────────────────

class GpsReader:
    _PTY_EMPTY_READ = "device reports readiness to read but returned no data"

    def __init__(self):
        self._running = False
        self._lock    = threading.Lock()
        self._p1: dict = {"lat": None, "lon": None, "alt": 0.0,
                          "fix": FIX_INVALID, "sats": 0, "acc": 99.9}
        self._p2: dict = {"lat": None, "lon": None}

    def start(self) -> None:
        self._running = True
        threading.Thread(target=self._read_loop, args=(GPS_PRIMARY_PORT,   True),
                         daemon=True, name="gps-primary").start()
        threading.Thread(target=self._read_loop, args=(GPS_SECONDARY_PORT, False),
                         daemon=True, name="gps-secondary").start()

    def stop(self) -> None:
        self._running = False

    def _read_loop(self, port: str, is_primary: bool) -> None:
        label = "primary" if is_primary else "secondary"
        ser: Optional[serial.Serial] = None
        while self._running:
            try:
                ser = serial.Serial(port, GPS_BAUD, timeout=0.2)
                while self._running:
                    raw = ser.readline()
                    if not raw:
                        continue
                    try:
                        line = raw.decode("ascii", errors="ignore").strip()
                    except Exception:
                        continue
                    if line.startswith("$"):
                        self._parse(line, is_primary)
            except serial.SerialException as exc:
                if self._PTY_EMPTY_READ in str(exc):
                    pass
                else:
                    print(f"[GPS] {label} {port}: {exc} — retry in 2 s")
                    time.sleep(2.0)
            finally:
                try:
                    if ser and ser.is_open:
                        ser.close()
                except Exception:
                    pass

    def _parse(self, line: str, is_primary: bool) -> None:
        if "*" in line:
            data, chk = line[1:].rsplit("*", 1)
            exp = 0
            for c in data:
                exp ^= ord(c)
            if f"{exp:02X}" != chk[:2].upper():
                return
            sentence = data
        else:
            sentence = line[1:]
        parts = sentence.split(",")
        tag   = parts[0].upper()
        if tag in ("GNGGA", "GPGGA", "GLGGA", "GAGGA"):
            self._handle_gga(parts, is_primary)
        elif tag in ("GNGST", "GPGST") and is_primary:
            self._handle_gst(parts)

    def _handle_gga(self, p: list, is_primary: bool) -> None:
        if len(p) < 10:
            return
        try:
            lat  = _nmea_to_deg(p[2], p[3])
            lon  = _nmea_to_deg(p[4], p[5])
            fix  = int(p[6])  if p[6]  else FIX_INVALID
            sats = int(p[7])  if p[7]  else 0
            alt  = float(p[9]) if p[9] else 0.0
        except (ValueError, IndexError):
            return
        with self._lock:
            if is_primary:
                old_fix = self._p1["fix"]
                self._p1.update(lat=lat, lon=lon, alt=alt, fix=fix, sats=sats)
                if fix != old_fix or self._p1["acc"] == 99.9:
                    self._p1["acc"] = _ACCURACY_FALLBACK.get(fix, 99.9)
            else:
                self._p2["lat"] = lat
                self._p2["lon"] = lon
        if is_primary:
            self._emit()

    def _handle_gst(self, p: list) -> None:
        if len(p) < 8:
            return
        try:
            h_acc = math.sqrt(float(p[6]) ** 2 + float(p[7]) ** 2)
        except (ValueError, IndexError):
            return
        with self._lock:
            self._p1["acc"] = h_acc
        self._emit()

    def _emit(self) -> None:
        with self._lock:
            p1 = dict(self._p1)
            p2 = dict(self._p2)
        if p1["lat"] is None:
            return
        hdg = 0.0
        base = 0.0
        if p2["lat"] is not None:
            base, raw_hdg = _haversine_bearing(p1["lat"], p1["lon"],
                                               p2["lat"], p2["lon"])
            hdg = (raw_hdg + 180.0) % 360.0
        with state_lock:
            state.lat          = p1["lat"]
            state.lon          = p1["lon"]
            state.alt_m        = p1["alt"]
            state.fix_quality  = p1["fix"]
            state.h_accuracy_m = p1["acc"]
            state.num_sats     = p1["sats"]
            state.heading_deg  = hdg
            state.baseline_m   = base


gps = GpsReader()

# ─── MAVLink handler ─────────────────────────────────────────────────────────

class MAVLink:
    def __init__(self):
        self._mav       = None
        self._relay_mav = None   # second connection: RC relay direct to RV2 machine
        self._running   = False
        self._lock      = threading.Lock()
        self._gcs_addr  = None   # (ip, port) — discovered from incoming GCS heartbeat

    def connect(self) -> None:
        import socket as _socket

        # Both rovers bind to 14550 — they always run on separate machines so
        # there is no conflict, and the GCS broadcasts to this port on all hosts.
        self._mav = mavutil.mavlink_connection(
            "udpin:0.0.0.0:14550",
            source_system    = MAV_SYSTEM_ID,
            source_component = MAV_COMPONENT_ID,
        )
        if hasattr(self._mav, "port") and hasattr(self._mav.port, "settimeout"):
            try:
                self._mav.port.settimeout(0.2)
            except Exception:
                pass

        # RV1 only: open a dedicated relay socket to RV2 machine so RC_CHANNELS
        # reach the rp2040_emulator even when WiFi blocks directed broadcast.
        if ROVER_ID == 1 and RELAY_HOST:
            self._relay_mav = mavutil.mavlink_connection(
                f"udpout:{RELAY_HOST}:{RELAY_PORT}",
                source_system    = MAV_SYSTEM_ID,
                source_component = MAV_COMPONENT_ID,
            )
            if RELAY_HOST.endswith(".255"):
                try:
                    self._relay_mav.port.setsockopt(
                        _socket.SOL_SOCKET, _socket.SO_BROADCAST, 1)
                except Exception:
                    pass
            print(f"[RV1] RC relay → {RELAY_HOST}:{RELAY_PORT}")

        self._running = True
        threading.Thread(target=self._recv_loop, daemon=True, name="mav-recv").start()

    def close(self) -> None:
        self._running = False
        if self._mav:
            try:
                self._mav.close()
            except Exception:
                pass
        if self._relay_mav:
            try:
                self._relay_mav.close()
            except Exception:
                pass

    # ── send ──────────────────────────────────────────────────────────────────

    def _send(self, fn) -> None:
        """Send MAVLink packet to GCS. No-op until a GCS heartbeat has been received."""
        addr = self._gcs_addr
        if addr is None or self._mav is None:
            return
        try:
            self._mav.last_address = addr
            fn()
        except Exception:
            pass

    def send_heartbeat(self) -> None:
        with state_lock:
            armed = state.is_armed
            auto  = state.is_autonomous
        base_mode   = 0x01 | (0x80 if armed else 0)
        custom_mode = 10 if auto else 0
        self._send(lambda: self._mav.mav.heartbeat_send(
            MAV_TYPE, MAV_AUTOPILOT, base_mode, custom_mode, 4
        ))

    def send_sys_status(self) -> None:
        with state_lock:
            mv  = state.battery_mv
            pct = state.battery_pct
        self._send(lambda: self._mav.mav.sys_status_send(
            0, 0, 0, 0, mv, -1, pct, 0, 0, 0, 0, 0, 0
        ))

    def send_global_position(self) -> None:
        with state_lock:
            lat = state.lat
            lon = state.lon
            alt = state.alt_m
            hdg = state.heading_deg
        if lat == 0.0 and lon == 0.0:
            return
        ts    = int(time.monotonic() * 1000) & 0xFFFFFFFF
        lat_i = int(lat * 1e7)
        lon_i = int(lon * 1e7)
        alt_i = int(alt * 1000)
        hdg_i = int(hdg * 100) % 36000
        self._send(lambda: self._mav.mav.global_position_int_send(
            ts, lat_i, lon_i, alt_i, alt_i, 0, 0, 0, hdg_i
        ))

    def send_rc_channels(self) -> None:
        with state_lock:
            raw = list(state.rc_channels)
        n      = len(raw)
        all_ch = raw + [65535] * (18 - n)
        ts     = int(time.monotonic() * 1000) & 0xFFFFFFFF
        _args  = (
            ts, n,
            all_ch[0],  all_ch[1],  all_ch[2],  all_ch[3],
            all_ch[4],  all_ch[5],  all_ch[6],  all_ch[7],
            all_ch[8],  all_ch[9],  all_ch[10], all_ch[11],
            all_ch[12], all_ch[13], all_ch[14], all_ch[15],
            all_ch[16], all_ch[17],
            255,
        )
        self._send(lambda: self._mav.mav.rc_channels_send(*_args))
        if self._relay_mav:
            try:
                self._relay_mav.mav.rc_channels_send(*_args)
            except Exception:
                pass

    def send_gps_raw(self) -> None:
        with state_lock:
            fix  = state.fix_quality
            sats = state.num_sats
            lat  = state.lat
            lon  = state.lon
            alt  = state.alt_m
            acc  = state.h_accuracy_m
        eph   = int(acc * 100) if acc < 655 else 65535
        ts    = int(time.monotonic() * 1000) & 0xFFFFFFFF
        lat_i = int(lat * 1e7)
        lon_i = int(lon * 1e7)
        alt_i = int(alt * 1000)
        self._send(lambda: self._mav.mav.gps_raw_int_send(
            ts, fix, lat_i, lon_i, alt_i, eph, 65535, 65535, 65535, sats
        ))

    def send_named_float(self, name: str, value: float) -> None:
        enc = name.encode("ascii", errors="replace")[:10].ljust(10, b"\x00")
        ts  = int(time.monotonic() * 1000) & 0xFFFFFFFF
        self._send(lambda: self._mav.mav.named_value_float_send(ts, enc, float(value)))

    def send_scaled_pressure(self, pressure_hpa: float, temp_c: float) -> None:
        ts    = int(time.monotonic() * 1000) & 0xFFFFFFFF
        pres  = float(pressure_hpa)
        temp  = int(temp_c * 100)
        self._send(lambda: self._mav.mav.scaled_pressure_send(ts, pres, 0.0, temp))

    # ── receive ───────────────────────────────────────────────────────────────

    def _recv_loop(self) -> None:
        import socket as _socket
        # Access the underlying UDP socket directly — same as catch_hb.py which
        # we confirmed works. recv_msg() wraps recv() in its own try/except and
        # returns None on socket.timeout, making it impossible to distinguish
        # "no data" from "parse failed", so we bypass it and use recvfrom() directly.
        sock = self._mav.port
        while self._running:
            try:
                data, addr = sock.recvfrom(2048)
            except _socket.timeout:
                continue          # no data — normal, just retry
            except Exception as e:
                print(f"[RV{ROVER_ID}] recv error: {e}")
                time.sleep(0.01)
                continue

            ip, port = addr

            # GCS discovery: packet from port 14550 with sysid=255 is the tablet
            if port == 14550 and _quick_sysid(data) == MAV_GCS_SYSID:
                if addr != self._gcs_addr:
                    self._gcs_addr = addr
                    print(f"\n[RV{ROVER_ID}] GCS discovered at {ip}:{port}")

            # Feed raw bytes to pymavlink parser (identical to recv_msg() internals)
            self._mav.last_address = addr
            for byte in data:
                try:
                    msg = self._mav.mav.parse_char(bytes([byte]))
                    if msg:
                        self._dispatch(msg)
                except Exception:
                    pass

    def _dispatch(self, msg) -> None:
        t = msg.get_type()
        if t == "RC_CHANNELS_OVERRIDE":
            self._handle_rc_override(msg)
        elif t == "COMMAND_LONG":
            self._handle_command(msg)

    def _handle_rc_override(self, msg) -> None:
        raw = [
            msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
            msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
        ]
        # Per MAVLink spec: 0 = "do not change this channel" (pass-through).
        # Many GCS apps send continuous RC_CHANNELS_OVERRIDE with 0/65535 on
        # all channels just to stay connected — that must NOT block the RP2040
        # UART path.  Only claim an active GCS override when at least one
        # motion channel (CH1 throttle, CH2 steering) has a real value.
        has_motion = any(v not in (0, 65535) for v in (raw[CH_THROTTLE], raw[CH_STEERING]))

        # Replace pass-through markers with center for PPM output
        ch = [PPM_CENTER if v in (0, 65535) else v for v in raw]

        swa = ch[CH_EMERGENCY]
        swb = ch[CH_MODE]
        is_emergency  = swa < EMERGENCY_THRESHOLD
        is_autonomous = (not is_emergency) and (swb > AUTONOMOUS_THRESHOLD)

        with state_lock:
            rover_sel = state.rover_select

        # AUTO mode: joystick blocked — autonomous script sends commands directly
        if is_autonomous:
            active = False
        elif is_emergency:
            active = False
        elif ROVER_ID == 1:
            active = (rover_sel <= ROVER_SELECT_LOW) or (rover_sel > ROVER_SELECT_HIGH)
        else:
            active = rover_sel > ROVER_SELECT_LOW

        effective = list(ch)
        if not active:
            effective[CH_THROTTLE] = PPM_CENTER
            effective[CH_STEERING] = PPM_CENTER

        uart.send_ppm(effective)
        with state_lock:
            if has_motion:
                # Real GCS joystick input — take priority over UART for 0.5 s
                state.last_rc_override_t = time.monotonic()
                state.rc_channels = list(ch) + [rover_sel]
            state.ppm_channels  = effective
            state.is_emergency  = is_emergency
            state.is_autonomous = is_autonomous

    def _handle_command(self, msg) -> None:
        cmd = msg.command
        if cmd == 400:   # MAV_CMD_COMPONENT_ARM_DISARM
            with state_lock:
                state.is_armed = (msg.param1 == 1)
            self._send(lambda: self._mav.mav.command_ack_send(cmd, 0))
        elif cmd == 176:  # MAV_CMD_DO_SET_MODE
            with state_lock:
                state.is_autonomous = (msg.param2 == 3)
            self._send(lambda: self._mav.mav.command_ack_send(cmd, 0))


mav = MAVLink()

# ─── Geometry helpers ─────────────────────────────────────────────────────────

def _nmea_to_deg(value: str, hemisphere: str) -> float:
    if not value:
        raise ValueError
    dot = value.index(".")
    deg = float(value[:dot - 2])
    mins = float(value[dot - 2:])
    result = deg + mins / 60.0
    if hemisphere in ("S", "W"):
        result = -result
    return result


def _quick_sysid(data: bytes) -> int:
    """Extract MAVLink sysid from raw bytes without full parsing. Returns -1 on failure."""
    if len(data) >= 6  and data[0] == 0xFE: return data[3]   # MAVLink v1
    if len(data) >= 10 and data[0] == 0xFD: return data[5]   # MAVLink v2
    return -1


def _haversine_bearing(lat1, lon1, lat2, lon2) -> tuple[float, float]:
    R    = 6_371_000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    dphi = phi2 - phi1
    a    = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    dist = R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    y    = math.sin(dlam) * math.cos(phi2)
    x    = math.cos(phi1) * math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlam)
    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
    return dist, bearing

# ─── Telemetry loop ───────────────────────────────────────────────────────────

def _telemetry_loop() -> None:
    t_hb      = 0.0
    t_pos     = 0.0
    t_rc      = 0.0
    t_sensors = 0.0

    while True:
        now = time.monotonic()

        if now - t_hb >= 1.0 / HZ_HEARTBEAT:
            mav.send_heartbeat()
            mav.send_sys_status()
            t_hb = now

        if now - t_pos >= 1.0 / HZ_POSITION:
            mav.send_global_position()
            mav.send_gps_raw()
            t_pos = now

        if now - t_rc >= 1.0 / HZ_RC:
            mav.send_rc_channels()
            t_rc = now

        if now - t_sensors >= 1.0 / HZ_SENSORS:
            with state_lock:
                tank  = state.tank_pct
                temp  = state.temp_c
                humid = state.humid_pct
                pres  = state.pressure_hpa
            mav.send_named_float("TANK",  tank)
            mav.send_named_float("HUMID", humid)
            mav.send_scaled_pressure(pres, temp)
            t_sensors = now

        time.sleep(0.01)

# ─── UART heartbeat loop ──────────────────────────────────────────────────────

def _uart_hb_loop() -> None:
    while True:
        uart.send_heartbeat()
        with state_lock:
            state.uart_hb_ok  = uart.heartbeat_ok
            state.uart_hb_age = uart.heartbeat_age
        time.sleep(0.05)

# ─── ANSI colours ─────────────────────────────────────────────────────────────

_R  = "\033[0m"       # reset
_BD = "\033[1m"       # bold
_DM = "\033[2m"       # dim
_RD = "\033[91m"      # red
_GR = "\033[92m"      # green
_YL = "\033[93m"      # yellow
_CY = "\033[96m"      # cyan
_WH = "\033[97m"      # white


def _c(text: str, col: str, bold: bool = False) -> str:
    return f"{_BD if bold else ''}{col}{text}{_R}"


# ─── Status display ───────────────────────────────────────────────────────────

def _status_loop() -> None:
    FIX_NAMES = {0: "NO_FIX", 1: "GPS", 2: "DGPS", 4: "RTK_FIX", 5: "RTK_FLT"}
    FIX_COL   = {0: _RD, 1: _YL, 2: _YL, 4: _GR, 5: _CY}

    import sys
    sys.stdout.write("\033[2J")   # clear screen once on start

    while True:
        with state_lock:
            s = RoverState(**state.__dict__)

        # ── Mode / arm ────────────────────────────────────────────────────────
        if s.is_emergency:
            mode_s = _c("EMERGENCY", _RD, bold=True)
        elif s.is_autonomous:
            mode_s = _c("AUTO", _YL, bold=True)
        else:
            mode_s = _c("MANUAL", _GR, bold=True)

        armed_s = _c("ARMED", _GR, bold=True) if s.is_armed else _c("DISARMED", _DM)

        # ── Rover selection (CH9) ─────────────────────────────────────────────
        ch9 = s.rover_select
        if ch9 > ROVER_SELECT_HIGH:
            sel_s = _c(f"AUTO ({ch9})", _YL)
        elif ch9 > ROVER_SELECT_LOW:
            sel_s = _c(f"RV2  ({ch9})", _CY)
        else:
            sel_s = _c(f"RV1  ({ch9})", _GR)

        # ── SBUS / HB ─────────────────────────────────────────────────────────
        if s.sbus_ok is True:
            sbus_s = _c("OK",   _GR)
        elif s.sbus_ok is False:
            sbus_s = _c("LOST", _RD, bold=True)
        else:
            sbus_s = _c("--",   _DM)

        if s.uart_hb_age is None:
            hb_s = _c("--", _DM)
        elif s.uart_hb_age < 1.0:
            hb_s = _c(f"{s.uart_hb_age:.1f}s", _GR)
        elif s.uart_hb_age < 2.5:
            hb_s = _c(f"{s.uart_hb_age:.1f}s", _YL)
        else:
            hb_s = _c(f"{s.uart_hb_age:.1f}s", _RD)

        # ── GPS ───────────────────────────────────────────────────────────────
        fix_col = FIX_COL.get(s.fix_quality, _DM)
        fix_s   = _c(FIX_NAMES.get(s.fix_quality, str(s.fix_quality)), fix_col, bold=(s.fix_quality == 4))
        acc_col = _GR if s.h_accuracy_m < 0.05 else (_YL if s.h_accuracy_m < 0.5 else _RD)
        acc_s   = _c(f"{s.h_accuracy_m:.3f}m", acc_col)
        sats_s  = _c(str(s.num_sats), _GR if s.num_sats >= 8 else _YL)

        # ── CH1–CH9 raw values from RP2040 UART ──────────────────────────────
        _rc = (list(s.rc_channels) + [PPM_CENTER] * 9)[:9]
        def _cv(i: int) -> str:
            v   = _rc[i]
            col = _GR if v > 1700 else (_RD if v < 1300 else _DM)
            return f"{col}CH{i+1}:{v}{_R}"
        _ch_row1 = "  ".join(_cv(i) for i in range(5))
        _ch_row2 = "  ".join(_cv(i) for i in range(5, 9))

        # ── Sensors ───────────────────────────────────────────────────────────
        tank_col = _GR if s.tank_pct > 30 else (_YL if s.tank_pct > 15 else _RD)
        tank_s   = _c(f"{s.tank_pct:.0f}%", tank_col)
        temp_s   = _c(f"{s.temp_c:.1f}°C", _YL if s.temp_c > 40 else _WH)
        humid_s  = _c(f"{s.humid_pct:.0f}%", _WH)

        # ── Assemble frame ────────────────────────────────────────────────────
        w   = 62
        sep = f"  {_DM}{'─'*w}{_R}\r\n"

        frame = (
            "\033[H"   # cursor home
            f"\r\n"
            f"  {_BD}{_WH}ROVER {ROVER_ID}{_R}  "
            f"{armed_s}  {mode_s}  SEL: {sel_s}\r\n"
            f"{sep}"
            f"  SBUS  {sbus_s}        HB  {hb_s}\r\n"
            f"  {_ch_row1}\r\n"
            f"  {_ch_row2}\r\n"
            f"{sep}"
            f"  GPS   {fix_s}   sats={sats_s}   acc={acc_s}\r\n"
            f"  pos   {_WH}{s.lat:.7f}{_R}, {_WH}{s.lon:.7f}{_R}\r\n"
            f"  hdg   {_CY}{s.heading_deg:.1f}°{_R}   "
            f"alt={_WH}{s.alt_m:.1f}m{_R}   "
            f"baseline={_DM}{s.baseline_m:.2f}m{_R}\r\n"
            f"{sep}"
            f"  TANK  {tank_s}   TEMP {temp_s}   HUMID {humid_s}   "
            f"PRES {_WH}{s.pressure_hpa:.1f}hPa{_R}\r\n"
            f"  {_DM}{time.strftime('%H:%M:%S')}  "
            f"GCS {mav._gcs_addr[0] if mav._gcs_addr else 'discovering…'}  "
            f"sysid={MAV_SYSTEM_ID}{_R}\r\n"
            "\033[J"   # clear to end of screen
        )

        sys.stdout.write(frame)
        sys.stdout.flush()
        time.sleep(0.2)

# ─── Simulator launcher helpers ───────────────────────────────────────────────

def _drain_proc(proc: subprocess.Popen) -> None:
    try:
        for _ in proc.stdout:
            pass
    except Exception:
        pass


def _launch_sim_gps(rover_id: int, real_port: str, gcs_host: str) -> None:
    """Launch simulator/sim.py and update GPS (and UART for RV1 proxy) port globals."""
    global UART_PORT, GPS_PRIMARY_PORT, GPS_SECONDARY_PORT

    mode   = "proxy" if rover_id == 1 else "emulate"
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "..", "simulator", "sim.py")
    cmd    = [sys.executable, script, "--rover", str(rover_id), "--mode", mode,
              "--gcs-host", gcs_host]
    if mode == "proxy":
        cmd += ["--real-port", real_port]

    print(f"[SIM] Starting GPS simulator (rover={rover_id}, mode={mode})…")
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)

    # sim.py always prints UART_PORT (UartProxy or UartEmulator pty)
    needed: set[str] = {"UART_PORT", "GPS_PRIMARY_PORT", "GPS_SECONDARY_PORT"}
    found:  dict[str, str] = {}
    for line in proc.stdout:
        stripped = line.strip().rstrip(" \\")
        for key in needed:
            if stripped.startswith(key + "="):
                found[key] = stripped.split("=", 1)[1]
                print(f"[SIM]   {key}={found[key]}")
        if needed <= found.keys():
            break

    UART_PORT          = found.get("UART_PORT",          UART_PORT)
    GPS_PRIMARY_PORT   = found.get("GPS_PRIMARY_PORT",   GPS_PRIMARY_PORT)
    GPS_SECONDARY_PORT = found.get("GPS_SECONDARY_PORT", GPS_SECONDARY_PORT)

    threading.Thread(target=_drain_proc, args=(proc,), daemon=True, name="sim-drain").start()


def _launch_sim_rc(listen_port: int) -> None:
    """Launch tools/rp2040_emulator.py and update UART_PORT global."""
    global UART_PORT

    script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "..", "tools", "rp2040_emulator.py")
    cmd = [sys.executable, script,
           "--listen-port", str(listen_port), "--master-sysid", "1"]

    print(f"[EMU] Starting RP2040 emulator (listen_port={listen_port})…")
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)

    for line in proc.stdout:
        if "Virtual pty" in line and ":" in line:
            UART_PORT = line.rsplit(":", 1)[-1].strip()
            print(f"[EMU]   UART_PORT={UART_PORT}")
            break

    threading.Thread(target=_drain_proc, args=(proc,), daemon=True, name="emu-drain").start()


# ─── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    global GCS_HOST, RELAY_HOST

    parser = argparse.ArgumentParser(description=f"Agri Rover controller")
    parser.add_argument("--sim-gps", action="store_true",
                        help="Launch GPS simulator subprocess (sim.py)")
    parser.add_argument("--sim-rc", action="store_true",
                        help="Launch RP2040 emulator for simulated RC link (Rover 2)")
    parser.add_argument("--real-port", default=UART_PORT,
                        help=f"Real RP2040 serial port for Rover 1 proxy mode (default: {UART_PORT})")
    parser.add_argument("--gcs-host", default=GCS_HOST,
                        help=f"GCS tablet IP or subnet broadcast (default: {GCS_HOST})")
    parser.add_argument("--relay-host", default=RELAY_HOST,
                        help="RV2 machine IP for direct RC relay (Rover 1 only). "
                             "Use when WiFi broadcast does not reach RV2.")
    args = parser.parse_args()

    GCS_HOST   = args.gcs_host
    RELAY_HOST = args.relay_host

    if args.sim_gps:
        _launch_sim_gps(ROVER_ID, args.real_port, GCS_HOST)
    # --sim-rc is redundant when --sim-gps is used for RV2 (sim.py emulate mode
    # now embeds the MAVLink RC listener).  Only launch the standalone emulator
    # when sim-gps is NOT active (e.g. real GPS but simulated RC link).
    if args.sim_rc and not args.sim_gps:
        _launch_sim_rc(14550)   # always listen for RV1's MAVLink on 14550

    print(f"[RV{ROVER_ID}] Starting rover controller")
    print(f"  UART:        {UART_PORT}:{UART_BAUD}")
    print(f"  GPS primary: {GPS_PRIMARY_PORT}  secondary: {GPS_SECONDARY_PORT}")
    print(f"  MAVLink out: {GCS_HOST}:{GCS_PORT}  (sysid={MAV_SYSTEM_ID})")
    if RELAY_HOST:
        print(f"  RC relay:    {RELAY_HOST}:{RELAY_PORT}")

    uart.connect()
    gps.start()
    mav.connect()

    threading.Thread(target=_telemetry_loop, daemon=True, name="telemetry").start()
    threading.Thread(target=_uart_hb_loop,  daemon=True, name="uart-hb").start()
    threading.Thread(target=_status_loop,   daemon=True, name="status").start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print(f"\n[RV{ROVER_ID}] Shutting down…")
    finally:
        gps.stop()
        uart.close()
        mav.close()


if __name__ == "__main__":
    main()
