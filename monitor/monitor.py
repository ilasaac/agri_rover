"""
agri_rover/monitor/monitor.py
Live MAVLink monitor for both rovers.

Listens on UDP ports 14550 (rover 1) and 14551 (rover 2).
Decodes all relevant messages and renders a live terminal dashboard.

Usage:
  python monitor/monitor.py [--host 0.0.0.0]

Press Ctrl+C to exit.
"""

from __future__ import annotations

import argparse
import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

try:
    from pymavlink import mavutil
except ImportError:
    raise ImportError("pip install pymavlink")

# ─── Configuration ────────────────────────────────────────────────────────────

ROVER_PORTS = {1: 14550, 2: 14551}   # sysid → UDP listen port
BIND_HOST   = "0.0.0.0"

# Fix quality labels
FIX_NAMES = {0: "NO_FIX", 1: "GPS", 2: "DGPS", 4: "RTK_FIX", 5: "RTK_FLT"}

MAV_SEVERITY = {
    0: "EMERGENCY", 1: "ALERT",   2: "CRITICAL", 3: "ERROR",
    4: "WARNING",   5: "NOTICE",  6: "INFO",      7: "DEBUG",
}

# ─── Rover state ──────────────────────────────────────────────────────────────

@dataclass
class RoverSnapshot:
    rover_id:     int   = 0

    # Heartbeat
    armed:        bool  = False
    mode:         str   = "UNKNOWN"
    mav_type:     int   = 0
    system_status: int  = 0
    last_hb:      float = 0.0

    # GPS
    lat:          float = 0.0
    lon:          float = 0.0
    alt_m:        float = 0.0
    heading_deg:  float = 0.0
    last_gps:     float = 0.0

    # GPS quality
    fix_quality:  int   = 0
    h_acc_m:      float = 99.9
    num_sats:     int   = 0
    last_fix:     float = 0.0

    # Battery
    battery_mv:   int   = 0
    battery_pct:  int   = -1
    last_battery: float = 0.0

    # RC channels (from RC_CHANNELS)
    rc_channels:  list[int] = field(default_factory=lambda: [1500] * 18)
    chancount:    int        = 0
    last_rc:      float      = 0.0

    # Named floats
    tank_pct:     float = 0.0
    humid_pct:    float = 0.0
    last_sensors: float = 0.0

    # Pressure/temp
    pressure_hpa: float = 0.0
    temp_c:       float = 0.0
    last_pressure: float = 0.0

    # NAV controller
    nav_bearing:  int   = 0
    wp_dist_m:    int   = 0
    last_nav:     float = 0.0

    # Mission
    wp_seq:       int   = 0
    last_wp:      float = 0.0

    # Status text log
    status_log:   list[tuple[float, int, str]] = field(default_factory=list)

    # Last MAVLink message time (any)
    last_msg:     float = 0.0


snapshots: dict[int, RoverSnapshot] = {
    1: RoverSnapshot(rover_id=1),
    2: RoverSnapshot(rover_id=2),
}
snap_lock = threading.Lock()

# ─── MAVLink listener ─────────────────────────────────────────────────────────

def _decode_mode(base_mode: int, custom_mode: int) -> str:
    """Decode ArduRover flight mode from MAVLink heartbeat fields."""
    mode_map = {
        0:  "MANUAL",
        2:  "LEARNING",
        3:  "STEERING",
        4:  "HOLD",
        10: "AUTO",
        11: "RTL",
        12: "SMART_RTL",
        15: "GUIDED",
        16: "INITIALISING",
    }
    return mode_map.get(custom_mode, f"MODE_{custom_mode}")


def _listener(rover_id: int, port: int) -> None:
    """Background thread: listens on UDP, decodes messages, updates snapshot."""
    conn_str = f"udpin:{BIND_HOST}:{port}"
    while True:
        try:
            mav = mavutil.mavlink_connection(conn_str)
            if hasattr(mav, "port") and hasattr(mav.port, "settimeout"):
                try:
                    mav.port.settimeout(0.5)
                except Exception:
                    pass
            print(f"[MON] Listening for Rover {rover_id} on UDP:{port}")
            while True:
                msg = mav.recv_msg()
                if msg is None:
                    continue
                _dispatch(rover_id, msg)
        except Exception as exc:
            print(f"[MON] RV{rover_id} listener error: {exc} — retry in 2 s")
            time.sleep(2.0)


def _dispatch(rover_id: int, msg) -> None:
    t    = msg.get_type()
    now  = time.time()

    with snap_lock:
        s = snapshots[rover_id]
        s.last_msg = now

        if t == "HEARTBEAT":
            s.armed   = bool(msg.base_mode & 0x80)
            s.mode    = _decode_mode(msg.base_mode, msg.custom_mode)
            s.mav_type = msg.type
            s.system_status = msg.system_status
            s.last_hb = now

        elif t == "SYS_STATUS":
            s.battery_mv  = msg.voltage_battery
            s.battery_pct = msg.battery_remaining
            s.last_battery = now

        elif t == "GLOBAL_POSITION_INT":
            s.lat         = msg.lat  / 1e7
            s.lon         = msg.lon  / 1e7
            s.alt_m       = msg.alt  / 1000.0
            s.heading_deg = (msg.hdg / 100.0) if msg.hdg != 65535 else s.heading_deg
            s.last_gps    = now

        elif t == "GPS_RAW_INT":
            s.fix_quality = msg.fix_type
            s.num_sats    = msg.satellites_visible
            s.h_acc_m     = (msg.h_acc / 1000.0) if msg.h_acc != 65535 else s.h_acc_m
            s.last_fix    = now

        elif t == "RC_CHANNELS":
            s.chancount   = msg.chancount
            s.rc_channels = [
                msg.chan1_raw,  msg.chan2_raw,  msg.chan3_raw,  msg.chan4_raw,
                msg.chan5_raw,  msg.chan6_raw,  msg.chan7_raw,  msg.chan8_raw,
                msg.chan9_raw,  msg.chan10_raw, msg.chan11_raw, msg.chan12_raw,
                msg.chan13_raw, msg.chan14_raw, msg.chan15_raw, msg.chan16_raw,
                msg.chan17_raw, msg.chan18_raw,
            ]
            s.last_rc = now

        elif t == "NAMED_VALUE_FLOAT":
            name = msg.name.decode("ascii", errors="ignore").rstrip("\x00")
            val  = msg.value
            if name == "TANK":
                s.tank_pct    = val
                s.last_sensors = now
            elif name == "HUMID":
                s.humid_pct   = val
                s.last_sensors = now

        elif t == "SCALED_PRESSURE":
            s.pressure_hpa = msg.press_abs
            s.temp_c       = msg.temperature / 100.0
            s.last_pressure = now

        elif t == "NAV_CONTROLLER_OUTPUT":
            s.nav_bearing  = msg.nav_bearing
            s.wp_dist_m    = msg.wp_dist
            s.last_nav     = now

        elif t == "MISSION_CURRENT":
            s.wp_seq  = msg.seq
            s.last_wp = now

        elif t == "STATUSTEXT":
            sev  = msg.severity
            text = msg.text.decode("ascii", errors="ignore").rstrip("\x00")
            s.status_log.append((now, sev, text))
            if len(s.status_log) > 20:
                s.status_log = s.status_log[-20:]


# ─── Terminal display ─────────────────────────────────────────────────────────

RESET  = "\033[0m"
BOLD   = "\033[1m"
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
WHITE  = "\033[97m"
DIM    = "\033[2m"


def _age_str(ts: float, warn: float = 2.0, dead: float = 5.0) -> str:
    if ts == 0.0:
        return f"{DIM}---{RESET}"
    age = time.time() - ts
    if age < warn:
        return f"{GREEN}{age:.1f}s{RESET}"
    if age < dead:
        return f"{YELLOW}{age:.1f}s{RESET}"
    return f"{RED}{age:.1f}s{RESET}"


def _ppm_bar(val: int, width: int = 10) -> str:
    """Render a PPM value as a small horizontal bar."""
    frac = (val - 1000) / 1000.0
    frac = max(0.0, min(1.0, frac))
    filled = round(frac * width)
    bar = "█" * filled + "░" * (width - filled)
    return f"{bar} {val}"


def _fix_color(fix: int) -> str:
    if fix == 4:
        return GREEN
    if fix == 5:
        return YELLOW
    if fix in (1, 2):
        return CYAN
    return RED


def _render_rover(s: RoverSnapshot) -> list[str]:
    lines = []
    now   = time.time()

    # ── Header ────────────────────────────────────────────────────────────────
    hb_age = (now - s.last_hb) if s.last_hb else None
    hb_str = f"{hb_age:.1f}s" if hb_age is not None else "---"
    hb_col = GREEN if (hb_age is not None and hb_age < 2) else RED

    arm_str = f"{GREEN}ARMED{RESET}"  if s.armed else f"{RED}DISARMED{RESET}"
    lines.append(
        f"  {BOLD}{'━'*60}{RESET}"
    )
    lines.append(
        f"  {BOLD}{CYAN}ROVER {s.rover_id}{RESET}  "
        f"HB: {hb_col}{hb_str}{RESET}  "
        f"Mode: {BOLD}{s.mode}{RESET}  "
        f"{arm_str}"
    )
    lines.append(f"  {'━'*60}")

    # ── GPS ───────────────────────────────────────────────────────────────────
    fix_col = _fix_color(s.fix_quality)
    fix_str = FIX_NAMES.get(s.fix_quality, str(s.fix_quality))
    gps_age = _age_str(s.last_gps)
    lines.append(
        f"  {BOLD}GPS{RESET}  "
        f"{fix_col}{fix_str}{RESET}  "
        f"sats={s.num_sats}  "
        f"acc={s.h_acc_m:.3f}m  "
        f"age={gps_age}"
    )
    lines.append(
        f"       lat={s.lat:.7f}  lon={s.lon:.7f}  "
        f"alt={s.alt_m:.1f}m  hdg={s.heading_deg:.1f}°"
    )

    # ── RC channels ───────────────────────────────────────────────────────────
    rc_age = _age_str(s.last_rc, warn=0.5, dead=2.0)
    lines.append(f"  {BOLD}RC{RESET}  age={rc_age}  ch={s.chancount}")
    if s.rc_channels:
        ch = s.rc_channels
        ch9_val = ch[8] if len(ch) > 8 else 1500
        if ch9_val > 1750:
            sel = f"{YELLOW}AUTO{RESET}"
        elif ch9_val > 1250:
            sel = f"{CYAN}RV2{RESET}"
        else:
            sel = f"{GREEN}RV1{RESET}"
        lines.append(
            f"    THR: {_ppm_bar(ch[0])}  STR: {_ppm_bar(ch[1])}"
            f"  SEL(CH9): {sel} ({ch9_val})"
        )
        lines.append(
            f"    CH3:{ch[2]:4d} CH4:{ch[3]:4d} "
            f"CH5:{ch[4]:4d} CH6:{ch[5]:4d} "
            f"CH7:{ch[6]:4d} CH8:{ch[7]:4d}"
        )

    # ── Sensors ───────────────────────────────────────────────────────────────
    batt_col = GREEN if s.battery_pct > 30 else (YELLOW if s.battery_pct > 15 else RED)
    batt_str = (
        f"{batt_col}{s.battery_pct}%{RESET}  {s.battery_mv}mV"
        if s.battery_pct >= 0
        else f"{DIM}---{RESET}"
    )
    lines.append(
        f"  {BOLD}Sensors{RESET}  "
        f"TANK:{s.tank_pct:.0f}%  "
        f"TEMP:{s.temp_c:.1f}°C  "
        f"HUMID:{s.humid_pct:.0f}%  "
        f"PRES:{s.pressure_hpa:.1f}hPa  "
        f"BATT:{batt_str}"
    )

    # ── Mission / nav ─────────────────────────────────────────────────────────
    if s.last_nav > 0:
        lines.append(
            f"  {BOLD}Nav{RESET}  "
            f"WP#{s.wp_seq}  "
            f"bearing={s.nav_bearing}°  "
            f"dist={s.wp_dist_m}m"
        )

    # ── Status log ────────────────────────────────────────────────────────────
    if s.status_log:
        lines.append(f"  {BOLD}Log{RESET}")
        for ts, sev, text in s.status_log[-4:]:
            sev_str = MAV_SEVERITY.get(sev, str(sev))
            t_str   = time.strftime("%H:%M:%S", time.localtime(ts))
            sev_col = RED if sev <= 3 else (YELLOW if sev == 4 else DIM)
            lines.append(f"    {DIM}{t_str}{RESET} [{sev_col}{sev_str}{RESET}] {text}")

    return lines


def _display_loop() -> None:
    os.system("clear" if os.name != "nt" else "cls")
    while True:
        with snap_lock:
            snaps = [RoverSnapshot(**snapshots[i].__dict__) for i in (1, 2)]

        # Build output
        out = []
        out.append(
            f"\n  {BOLD}{WHITE}=== Agri Rover Monitor ==={RESET}  "
            f"{DIM}{time.strftime('%Y-%m-%d %H:%M:%S')}{RESET}\n"
        )
        for s in snaps:
            out.extend(_render_rover(s))
            out.append("")

        # Relative distance between rovers (if both have GPS)
        s1, s2 = snaps
        if s1.last_gps > 0 and s2.last_gps > 0:
            dist = _haversine(s1.lat, s1.lon, s2.lat, s2.lon)
            out.append(f"  {BOLD}Rover separation:{RESET}  {dist:.2f} m\n")

        # Move cursor to top and redraw
        print("\033[H", end="")
        print("\n".join(out), flush=True)

        time.sleep(0.25)


def _haversine(lat1, lon1, lat2, lon2) -> float:
    R    = 6_371_000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    dphi = phi2 - phi1
    a    = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ─── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    global BIND_HOST
    parser = argparse.ArgumentParser(description="Agri Rover MAVLink Monitor")
    parser.add_argument("--host", default=BIND_HOST)
    args = parser.parse_args()

    BIND_HOST = args.host

    for rover_id, port in ROVER_PORTS.items():
        threading.Thread(
            target=_listener, args=(rover_id, port),
            daemon=True, name=f"listener-rv{rover_id}",
        ).start()

    try:
        _display_loop()
    except KeyboardInterrupt:
        print("\n[MON] Exiting.")


if __name__ == "__main__":
    main()
