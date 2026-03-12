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

import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import serial

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
GCS_HOST:             str = os.environ.get("GCS_HOST",             "192.168.144.20")
GCS_PORT:             int = int(os.environ.get("GCS_PORT",         "14550"))
MAVLINK_BIND_PORT:    int = 14550 + (ROVER_ID - 1)   # rover1=14550, rover2=14551

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
        packet = "<" + ",".join(str(v) for v in channels) + ">"
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
        while self._running:
            try:
                raw = self._ser.readline()
                if raw:
                    line = raw.decode("utf-8", errors="ignore").strip()
                    if line:
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
            try:
                n = int(line[4:-1])
                if self._hb_expected >= 0 and n == self._hb_expected:
                    self._hb_last_rx  = time.time()
                    self._hb_ever_rx  = True
            except ValueError:
                pass

    def _parse_channels(self, data: str) -> None:
        try:
            ch_part = data.split(" MODE:")[0]
            vals = [int(v.strip()) for v in ch_part.split(",")]
            if len(vals) < 8:
                return
            swa = vals[CH_EMERGENCY]
            swb = vals[CH_MODE]
            is_emergency  = swa < EMERGENCY_THRESHOLD
            is_autonomous = (not is_emergency) and (swb > AUTONOMOUS_THRESHOLD)
            rover_sel     = vals[CH_ROVER_SEL] if len(vals) > CH_ROVER_SEL else PPM_MIN
            with state_lock:
                state.rc_channels   = vals
                state.ppm_channels  = vals[:8]
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
        self._mav     = None
        self._running = False
        self._lock    = threading.Lock()

    def connect(self) -> None:
        conn = f"udpout:{GCS_HOST}:{GCS_PORT}"
        self._mav = mavutil.mavlink_connection(
            conn,
            source_system    = MAV_SYSTEM_ID,
            source_component = MAV_COMPONENT_ID,
        )
        if hasattr(self._mav, "port") and hasattr(self._mav.port, "settimeout"):
            try:
                self._mav.port.settimeout(0.2)
            except Exception:
                pass
        self._running = True
        threading.Thread(target=self._recv_loop, daemon=True, name="mav-recv").start()

    def close(self) -> None:
        self._running = False
        if self._mav:
            try:
                self._mav.close()
            except Exception:
                pass

    # ── send ──────────────────────────────────────────────────────────────────

    def send_heartbeat(self) -> None:
        if not self._mav:
            return
        with state_lock:
            armed  = state.is_armed
            auto   = state.is_autonomous
        base_mode = 0x01  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if armed:
            base_mode |= 0x80
        custom_mode = 4 if auto else 0   # AUTO=4, MANUAL=0
        self._mav.mav.heartbeat_send(
            MAV_TYPE, MAV_AUTOPILOT, base_mode, custom_mode, 4
        )

    def send_sys_status(self) -> None:
        if not self._mav:
            return
        with state_lock:
            mv  = state.battery_mv
            pct = state.battery_pct
        self._mav.mav.sys_status_send(
            0, 0, 0, 0, mv, -1, pct,
            0, 0, 0, 0, 0, 0
        )

    def send_global_position(self) -> None:
        if not self._mav:
            return
        with state_lock:
            lat = state.lat
            lon = state.lon
            alt = state.alt_m
            hdg = state.heading_deg
        self._mav.mav.global_position_int_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            int(lat * 1e7),
            int(lon * 1e7),
            int(alt * 1000),
            int(alt * 1000),
            0, 0, 0,
            int(hdg * 100) % 36000,
        )

    def send_rc_channels(self) -> None:
        if not self._mav:
            return
        with state_lock:
            ch = list(state.ppm_channels)
        ch += [65535] * (18 - len(ch))
        self._mav.mav.rc_channels_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            len(state.ppm_channels),
            ch[0],  ch[1],  ch[2],  ch[3],
            ch[4],  ch[5],  ch[6],  ch[7],
            ch[8],  ch[9],  ch[10], ch[11],
            ch[12], ch[13], ch[14], ch[15],
            ch[16], ch[17],
            255,
        )

    def send_named_float(self, name: str, value: float) -> None:
        if not self._mav:
            return
        enc = name.encode("ascii", errors="replace")[:10].ljust(10, b"\x00")
        self._mav.mav.named_value_float_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            enc, float(value),
        )

    def send_scaled_pressure(self, pressure_hpa: float, temp_c: float) -> None:
        if not self._mav:
            return
        self._mav.mav.scaled_pressure_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            float(pressure_hpa), 0.0,
            int(temp_c * 100),
        )

    # ── receive ───────────────────────────────────────────────────────────────

    def _recv_loop(self) -> None:
        while self._running:
            try:
                msg = self._mav.recv_msg()
                if msg is None:
                    time.sleep(0.005)
                    continue
                self._dispatch(msg)
            except Exception:
                time.sleep(0.01)

    def _dispatch(self, msg) -> None:
        t = msg.get_type()
        if t == "RC_CHANNELS_OVERRIDE":
            self._handle_rc_override(msg)
        elif t == "COMMAND_LONG":
            self._handle_command(msg)

    def _handle_rc_override(self, msg) -> None:
        ch = [
            msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
            msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
        ]
        # Replace 0/65535 with center
        ch = [PPM_CENTER if v in (0, 65535) else v for v in ch]
        uart.send_ppm(ch)

    def _handle_command(self, msg) -> None:
        cmd = msg.command
        if cmd == 400:   # MAV_CMD_COMPONENT_ARM_DISARM
            with state_lock:
                state.is_armed = (msg.param1 == 1)
            self._mav.mav.command_ack_send(cmd, 0)
        elif cmd == 176:  # MAV_CMD_DO_SET_MODE
            with state_lock:
                state.is_autonomous = (msg.param2 == 3)
            self._mav.mav.command_ack_send(cmd, 0)


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

# ─── Status display ───────────────────────────────────────────────────────────

def _status_loop() -> None:
    fix_names = {0: "NO_FIX", 1: "GPS", 2: "DGPS", 4: "RTK_FIX", 5: "RTK_FLT"}
    while True:
        with state_lock:
            s = RoverState(**state.__dict__)

        ch9 = s.rover_select
        if ch9 > ROVER_SELECT_HIGH:
            sel = "AUTO"
        elif ch9 > ROVER_SELECT_LOW:
            sel = "RV2"
        else:
            sel = "RV1"

        sbus_str  = "OK" if s.sbus_ok else ("LOST" if s.sbus_ok is False else "--")
        fix_str   = fix_names.get(s.fix_quality, str(s.fix_quality))
        hb_str    = f"{s.uart_hb_age:.1f}s" if s.uart_hb_age is not None else "--"
        mode_str  = "AUTO" if s.is_autonomous else ("EMERGENCY" if s.is_emergency else "MANUAL")
        armed_str = "ARMED" if s.is_armed else "DISARMED"

        print(
            f"\r[RV{ROVER_ID}] {armed_str} | {mode_str} | SEL:{sel} | "
            f"SBUS:{sbus_str} | HB:{hb_str} | "
            f"GPS:{fix_str} sats={s.num_sats} acc={s.h_accuracy_m:.2f}m | "
            f"pos={s.lat:.6f},{s.lon:.6f} hdg={s.heading_deg:.1f}° | "
            f"TANK:{s.tank_pct:.0f}% TEMP:{s.temp_c:.1f}°C HUMID:{s.humid_pct:.0f}%    ",
            end="", flush=True,
        )
        time.sleep(0.2)

# ─── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    print(f"[RV{ROVER_ID}] Starting rover controller")
    print(f"  UART:        {UART_PORT}:{UART_BAUD}")
    print(f"  GPS primary: {GPS_PRIMARY_PORT}  secondary: {GPS_SECONDARY_PORT}")
    print(f"  MAVLink out: {GCS_HOST}:{GCS_PORT}  (sysid={MAV_SYSTEM_ID})")

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
        print("\n[RV{ROVER_ID}] Shutting down…")
    finally:
        gps.stop()
        uart.close()
        mav.close()


if __name__ == "__main__":
    main()
