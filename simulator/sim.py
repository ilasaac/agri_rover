"""
agri_rover/simulator/sim.py
RTK GPS + rover physics simulator for bench testing without real hardware.

Modes
-----
  proxy   — bridges rover/main.py ↔ real RP2040 transparently via pty.
            Sniffs CH: lines for physics. Start BEFORE rover/main.py.
  emulate — no real RP2040 required. Generates synthetic CH: lines.

Usage
-----
  python simulator/sim.py --rover 1 [--mode proxy|emulate]
  python simulator/sim.py --rover 2 [--mode proxy|emulate]

What it provides
----------------
  - Two virtual pty ports that look like ublox EVK-X20P GPS receivers.
    Print their paths on startup; pass them to rover/main.py via env vars:
      GPS_PRIMARY_PORT=/dev/pts/N  GPS_SECONDARY_PORT=/dev/pts/M
  - In proxy mode: a pty that bridges rover/main.py to the real RP2040.
    Pass its path via: UART_PORT=/dev/pts/K
  - Unicycle physics driven by PPM throttle/steering channels.
  - Two rovers share a common state file (/tmp/agri_sim_state.json) so
    both simulators can display each other on a shared map view.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pty
import select
import sys
import termios
import threading
import time
from typing import Optional

try:
    import serial
except ImportError:
    raise ImportError("pip install pyserial")

# ─── Physics ──────────────────────────────────────────────────────────────────

class UnicyclePhysics:
    """Simple unicycle rover physics."""

    MAX_SPEED_MS    = 2.0    # m/s at full throttle
    MAX_TURN_RAD_S  = 1.2    # rad/s at full steering
    WHEELBASE_M     = 0.8    # used only for baseline display
    DT              = 0.05   # seconds per physics tick

    def __init__(self, lat0: float, lon0: float, heading0_deg: float = 0.0):
        self.lat     = lat0
        self.lon     = lon0
        self.heading = math.radians(heading0_deg)   # radians, 0=north
        self.alt_m   = 10.0
        self._lock   = threading.Lock()
        self._thr    = 1500   # current throttle PPM
        self._str    = 1500   # current steering PPM
        self._running = True
        threading.Thread(target=self._step_loop, daemon=True, name="physics").start()

    def set_ppm(self, throttle: int, steering: int) -> None:
        with self._lock:
            self._thr = throttle
            self._str = steering

    def get_position(self) -> tuple[float, float, float, float]:
        """Returns (lat, lon, alt_m, heading_deg)."""
        with self._lock:
            return self.lat, self.lon, self.alt_m, math.degrees(self.heading) % 360.0

    def _step_loop(self) -> None:
        while self._running:
            with self._lock:
                thr = self._thr
                s   = self._str
            self._step(thr, s)
            time.sleep(self.DT)

    def _step(self, thr: int, steer: int) -> None:
        speed    = (thr - 1500) / 500.0 * self.MAX_SPEED_MS
        turn     = (steer - 1500) / 500.0 * self.MAX_TURN_RAD_S

        dheading = turn * self.DT
        ds       = speed * self.DT

        with self._lock:
            self.heading = (self.heading + dheading) % (2 * math.pi)
            # Move north/east using flat-earth approximation
            R     = 6_371_000.0
            dlat  = (ds * math.cos(self.heading)) / R
            dlon  = (ds * math.sin(self.heading)) / (R * math.cos(math.radians(self.lat)))
            self.lat += math.degrees(dlat)
            self.lon += math.degrees(dlon)

    def stop(self) -> None:
        self._running = False


# ─── NMEA generation ──────────────────────────────────────────────────────────

def _nmea_checksum(sentence: str) -> str:
    chk = 0
    for c in sentence:
        chk ^= ord(c)
    return f"{chk:02X}"


def _deg_to_nmea(deg: float, is_lat: bool) -> tuple[str, str]:
    if is_lat:
        hem = "N" if deg >= 0 else "S"
    else:
        hem = "E" if deg >= 0 else "W"
    deg = abs(deg)
    d   = int(deg)
    m   = (deg - d) * 60.0
    if is_lat:
        val = f"{d:02d}{m:010.7f}"
    else:
        val = f"{d:03d}{m:010.7f}"
    return val, hem


def make_gga(lat: float, lon: float, alt: float,
             fix: int = 4, sats: int = 12) -> str:
    lt, lh = _deg_to_nmea(lat, True)
    ln, lnh = _deg_to_nmea(lon, False)
    t = time.gmtime()
    ts = f"{t.tm_hour:02d}{t.tm_min:02d}{t.tm_sec:02d}.00"
    body = f"GNGGA,{ts},{lt},{lh},{ln},{lnh},{fix},{sats},0.6,{alt:.2f},M,0.0,M,,"
    return f"${body}*{_nmea_checksum(body)}\r\n"


def make_gst(lat_sig: float = 0.01, lon_sig: float = 0.01) -> str:
    t = time.gmtime()
    ts = f"{t.tm_hour:02d}{t.tm_min:02d}{t.tm_sec:02d}.00"
    rms = math.sqrt(lat_sig**2 + lon_sig**2)
    body = f"GNGST,{ts},{rms:.4f},0,0,0,{lat_sig:.4f},{lon_sig:.4f}"
    return f"${body}*{_nmea_checksum(body)}\r\n"


# ─── Pty helpers ──────────────────────────────────────────────────────────────

def open_pty() -> tuple[int, int, str]:
    """Open a pty pair. Returns (master_fd, slave_fd, slave_path)."""
    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    # Disable echo and canonical mode on master side
    try:
        attrs = termios.tcgetattr(master_fd)
        attrs[3] &= ~(termios.ECHO | termios.ICANON)
        termios.tcsetattr(master_fd, termios.TCSANOW, attrs)
    except Exception:
        pass
    return master_fd, slave_fd, slave_path


# ─── GPS pty writer ───────────────────────────────────────────────────────────

class GpsPtyWriter:
    """Writes simulated NMEA sentences to a pty master fd at 10 Hz."""

    BASELINE_M = 0.6   # simulated antenna separation

    def __init__(self, physics: UnicyclePhysics):
        self._physics  = physics
        self._running  = False
        self.primary_path:   str = ""
        self.secondary_path: str = ""
        self._mfd_p:  int = -1
        self._mfd_s:  int = -1

    def start(self) -> None:
        mfd_p, sfd_p, self.primary_path   = open_pty()
        mfd_s, sfd_s, self.secondary_path = open_pty()
        self._mfd_p = mfd_p
        self._mfd_s = mfd_s
        # Keep slave fds open so the pty stays alive
        self._sfd_p = sfd_p
        self._sfd_s = sfd_s
        self._running = True
        threading.Thread(target=self._write_loop, daemon=True, name="gps-writer").start()

    def stop(self) -> None:
        self._running = False

    def _write_loop(self) -> None:
        while self._running:
            lat, lon, alt, hdg = self._physics.get_position()

            # Primary antenna (front): actual rover position
            gga_p = make_gga(lat, lon, alt).encode()
            gst_p = make_gst().encode()

            # Secondary antenna (rear): offset by BASELINE behind primary
            hdg_rad = math.radians(hdg)
            R = 6_371_000.0
            dlat_r = -(self.BASELINE_M * math.cos(hdg_rad)) / R
            dlon_r = -(self.BASELINE_M * math.sin(hdg_rad)) / (R * math.cos(math.radians(lat)))
            lat2 = lat + math.degrees(dlat_r)
            lon2 = lon + math.degrees(dlon_r)
            gga_s = make_gga(lat2, lon2, alt).encode()

            try:
                os.write(self._mfd_p, gga_p)
                os.write(self._mfd_p, gst_p)
            except OSError:
                pass
            try:
                os.write(self._mfd_s, gga_s)
            except OSError:
                pass

            time.sleep(0.1)   # 10 Hz


# ─── UART pty (proxy mode) ────────────────────────────────────────────────────

class UartProxy:
    """
    Bridges rover/main.py ↔ real RP2040 via pty.
    Sniffs CH: lines from the RP2040 to extract PPM for physics.
    """

    def __init__(self, real_port: str, physics: UnicyclePhysics):
        self._real_port = real_port
        self._physics   = physics
        self._running   = False
        self.pty_path:  str = ""
        self._mfd:      int = -1

    def start(self) -> None:
        mfd, sfd, self.pty_path = open_pty()
        self._mfd  = mfd
        self._sfd  = sfd
        self._running = True
        threading.Thread(target=self._proxy_loop, daemon=True, name="uart-proxy").start()

    def stop(self) -> None:
        self._running = False

    def _proxy_loop(self) -> None:
        ser: Optional[serial.Serial] = None
        while self._running:
            try:
                ser = serial.Serial(self._real_port, 115200, timeout=0.1)
                buf = b""
                while self._running:
                    # RP2040 → main.py
                    raw = ser.read(256)
                    if raw:
                        try:
                            os.write(self._mfd, raw)
                        except OSError:
                            pass
                        buf += raw
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            self._sniff(line.decode("utf-8", errors="ignore").strip())

                    # main.py → RP2040
                    rlist, _, _ = select.select([self._mfd], [], [], 0)
                    if rlist:
                        try:
                            data = os.read(self._mfd, 256)
                            if data:
                                ser.write(data)
                        except OSError:
                            pass

            except serial.SerialException as exc:
                print(f"[PROXY] {self._real_port}: {exc} — retry in 2 s")
                time.sleep(2.0)
            finally:
                if ser and ser.is_open:
                    ser.close()

    def _sniff(self, line: str) -> None:
        if line.startswith("CH:"):
            try:
                ch_part = line[3:].split(" MODE:")[0]
                vals = [int(v) for v in ch_part.split(",")]
                if len(vals) >= 2:
                    self._physics.set_ppm(vals[0], vals[1])
            except ValueError:
                pass


# ─── UART emulator (emulate mode) ─────────────────────────────────────────────

class UartEmulator:
    """
    No real RP2040.  Provides a pty that main.py can connect to.
    Parses PPM commands from main.py; drives physics.
    Sends synthetic CH: lines back to main.py.
    """

    def __init__(self, physics: UnicyclePhysics):
        self._physics = physics
        self._running = False
        self.pty_path: str = ""
        self._mfd:     int = -1
        self._thr      = 1500
        self._str      = 1500

    def start(self) -> None:
        mfd, sfd, self.pty_path = open_pty()
        self._mfd = mfd
        self._sfd = sfd
        self._running = True
        threading.Thread(target=self._recv_loop, daemon=True, name="emu-recv").start()
        threading.Thread(target=self._send_loop, daemon=True, name="emu-send").start()

    def stop(self) -> None:
        self._running = False

    def _recv_loop(self) -> None:
        buf = b""
        while self._running:
            rlist, _, _ = select.select([self._mfd], [], [], 0.1)
            if rlist:
                try:
                    data = os.read(self._mfd, 256)
                    if data:
                        buf += data
                        while b"\n" in buf or b">" in buf:
                            if b">" in buf:
                                idx = buf.index(b">")
                                pkt = buf[:idx + 1].decode("utf-8", errors="ignore")
                                buf = buf[idx + 1:]
                                self._parse_pkt(pkt.strip())
                            elif b"\n" in buf:
                                line, buf = buf.split(b"\n", 1)
                                self._parse_pkt(line.decode("utf-8", errors="ignore").strip())
                            else:
                                break
                except OSError:
                    pass

    def _parse_pkt(self, pkt: str) -> None:
        if pkt.startswith("<") and pkt.endswith(">"):
            inner = pkt[1:-1]
            if inner.startswith("HB:"):
                # Echo heartbeat back with N+1
                try:
                    n = int(inner[3:])
                    reply = f"<HB:{(n+1)%10000}>\n"
                    os.write(self._mfd, reply.encode())
                except (ValueError, OSError):
                    pass
                return
            if inner.startswith("J:"):
                inner = inner[2:]
            try:
                vals = [int(v) for v in inner.split(",")]
                if len(vals) >= 2:
                    self._thr = vals[0]
                    self._str = vals[1]
                    self._physics.set_ppm(self._thr, self._str)
            except ValueError:
                pass

    def _send_loop(self) -> None:
        while self._running:
            thr = self._thr
            steer = self._str
            line = f"CH:{thr},{steer},2000,2000,2000,2000,1500,1500,1000\n"
            try:
                os.write(self._mfd, line.encode())
            except OSError:
                pass
            time.sleep(0.1)   # 10 Hz


# ─── Shared state file ────────────────────────────────────────────────────────

STATE_FILE = "/tmp/agri_sim_state.json"

def write_state(rover_id: int, physics: UnicyclePhysics) -> None:
    lat, lon, alt, hdg = physics.get_position()
    try:
        try:
            with open(STATE_FILE) as f:
                data = json.load(f)
        except Exception:
            data = {}
        data[str(rover_id)] = {"lat": lat, "lon": lon, "alt": alt, "heading": hdg,
                                "ts": time.time()}
        with open(STATE_FILE, "w") as f:
            json.dump(data, f)
    except Exception:
        pass


def _state_writer(rover_id: int, physics: UnicyclePhysics) -> None:
    while True:
        write_state(rover_id, physics)
        time.sleep(0.2)


# ─── Main ─────────────────────────────────────────────────────────────────────

# Default start positions (lat/lon) for each rover
DEFAULT_POSITIONS = {
    1: (37.7749, -122.4194, 0.0),   # Rover 1 start position
    2: (37.7750, -122.4193, 0.0),   # Rover 2 start position (nearby)
}

def main() -> None:
    parser = argparse.ArgumentParser(description="Agri Rover GPS Simulator")
    parser.add_argument("--rover", type=int, choices=[1, 2], default=1)
    parser.add_argument("--mode",  choices=["proxy", "emulate"], default="emulate")
    parser.add_argument("--real-port", default="/dev/ttyACM0",
                        help="Real RP2040 port (proxy mode only)")
    parser.add_argument("--lat", type=float, default=None)
    parser.add_argument("--lon", type=float, default=None)
    parser.add_argument("--heading", type=float, default=0.0)
    args = parser.parse_args()

    if sys.platform == "win32":
        print("ERROR: Simulator requires Linux (pty-based).  Run on Jetson/RPi.")
        sys.exit(1)

    lat0, lon0, _ = DEFAULT_POSITIONS[args.rover]
    if args.lat is not None:
        lat0 = args.lat
    if args.lon is not None:
        lon0 = args.lon

    physics = UnicyclePhysics(lat0, lon0, args.heading)

    # GPS pty writer (always active)
    gps_writer = GpsPtyWriter(physics)
    gps_writer.start()

    # UART interface
    if args.mode == "proxy":
        uart_iface = UartProxy(args.real_port, physics)
    else:
        uart_iface = UartEmulator(physics)
    uart_iface.start()

    # Background shared state writer
    threading.Thread(target=_state_writer, args=(args.rover, physics),
                     daemon=True, name="state-writer").start()

    print(f"\n=== Agri Rover Simulator  (Rover {args.rover}, mode={args.mode}) ===")
    print(f"\nSet these environment variables before starting rover/main.py:\n")
    print(f"  ROVER_ID={args.rover} \\")
    print(f"  UART_PORT={uart_iface.pty_path} \\")
    print(f"  GPS_PRIMARY_PORT={gps_writer.primary_path} \\")
    print(f"  GPS_SECONDARY_PORT={gps_writer.secondary_path} \\")
    print(f"  python rover/main.py\n")

    # Live display
    try:
        while True:
            lat, lon, alt, hdg = physics.get_position()
            print(
                f"\r[SIM RV{args.rover}] lat={lat:.6f} lon={lon:.6f} "
                f"hdg={hdg:.1f}° alt={alt:.1f}m    ",
                end="", flush=True,
            )
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[SIM] Shutting down…")
    finally:
        physics.stop()
        gps_writer.stop()
        uart_iface.stop()


if __name__ == "__main__":
    main()
