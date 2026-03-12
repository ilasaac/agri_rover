"""
tools/rp2040_emulator.py
Virtual RP2040 receiver for Rover 2 when the CC1101 RF link is unavailable.

Listens for RC_CHANNELS MAVLink packets broadcast by the master rover (sysid=1)
on UDP and re-emits them over a pty as if they came from the receiver RP2040
firmware over USB serial.

Output format (identical to receiver/main.c):
  CH:ch1,ch2,...,ch9 MODE:MANUAL\n   — at 10 Hz
  [RF_LINK_OK]\n / [RF_LINK_LOST]\n  — when UDP link state changes
  [SBUS_OK]\n    / [SBUS_LOST]\n     — mirrors RF link state
  <HB:N+1>\n                         — heartbeat echo

Usage:
  python tools/rp2040_emulator.py [--listen-port 14550] [--master-sysid 1]

  It prints the pty path on startup. Pass it to rover/main.py as UART_PORT:

    ROVER_ID=2 UART_PORT=/dev/pts/X GPS_PRIMARY_PORT=... python rover/main.py
"""

from __future__ import annotations

import argparse
import os
import pty
import select
import sys
import termios
import threading
import time

try:
    from pymavlink import mavutil
except ImportError:
    raise ImportError("pip install pymavlink")

# ─── Configuration ─────────────────────────────────────────────────────────────

LISTEN_PORT  = 14550
MASTER_SYSID = 1

PPM_CENTER = 1500

EMERGENCY_THRESHOLD  = 1700
AUTONOMOUS_THRESHOLD = 1700
ROVER_SELECT_LOW     = 1250
ROVER_SELECT_HIGH    = 1750

LINK_TIMEOUT_S = 0.5   # seconds without RC_CHANNELS → declare RF link lost

# ─── Shared state ──────────────────────────────────────────────────────────────

_channels: list[int] = [PPM_CENTER] * 9
_ch_lock  = threading.Lock()
_last_rx  = 0.0   # monotonic time of last RC_CHANNELS received

# ─── MAVLink listener ──────────────────────────────────────────────────────────

def _mav_listener(port: int, master_sysid: int) -> None:
    global _last_rx
    import socket as _socket
    while True:
        try:
            mav = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}")
            if hasattr(mav, "port") and hasattr(mav.port, "settimeout"):
                try:
                    mav.port.settimeout(0.5)
                except Exception:
                    pass
            print(f"[EMU] Listening for master (sysid={master_sysid}) on UDP:{port}")
            while True:
                try:
                    msg = mav.recv_msg()
                except _socket.timeout:
                    continue   # no data yet — keep waiting
                if msg is None:
                    continue
                if msg.get_srcSystem() != master_sysid:
                    continue
                if msg.get_type() == "RC_CHANNELS":
                    raw = [
                        msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                        msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
                        msg.chan9_raw,
                    ]
                    # 65535 = not present — replace with center
                    ch = [PPM_CENTER if v == 65535 else v for v in raw]
                    with _ch_lock:
                        _channels[:] = ch
                    _last_rx = time.monotonic()
        except Exception as exc:
            print(f"[EMU] Listener error: {exc} — retry in 2 s")
            time.sleep(2.0)


# ─── Mode logic (mirrors emitter firmware logic) ──────────────────────────────

def _mode_str(ch: list[int]) -> str:
    swa = ch[4]   # CH5 — emergency stop switch
    swb = ch[5]   # CH6 — autonomous mode switch
    sel = ch[8]   # CH9 — rover selection
    if swa < EMERGENCY_THRESHOLD:
        return "EMERGENCY"
    if swb > AUTONOMOUS_THRESHOLD:
        return "AUTONOMOUS"
    if ROVER_SELECT_LOW < sel <= ROVER_SELECT_HIGH:
        return "RELAY"
    return "MANUAL"


# ─── PTY helpers ──────────────────────────────────────────────────────────────

def _open_pty() -> tuple[int, int, str]:
    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    try:
        attrs = termios.tcgetattr(master_fd)
        attrs[3] &= ~(termios.ECHO | termios.ICANON)
        termios.tcsetattr(master_fd, termios.TCSANOW, attrs)
    except Exception:
        pass
    return master_fd, slave_fd, slave_path


def _pty_write(fd: int, data: bytes) -> None:
    try:
        os.write(fd, data)
    except OSError:
        pass


# ─── PTY emulation loop ───────────────────────────────────────────────────────

def _pty_loop(master_fd: int) -> None:
    link_ok     = False
    last_ch_out = 0.0
    in_buf      = b""

    while True:
        now      = time.monotonic()
        rf_alive = (_last_rx > 0) and (now - _last_rx < LINK_TIMEOUT_S)

        # ── RF / SBUS link state transitions ─────────────────────────────────
        if rf_alive and not link_ok:
            link_ok = True
            _pty_write(master_fd, b"[RF_LINK_OK]\n")
            _pty_write(master_fd, b"[SBUS_OK]\n")
            print("[EMU] RF link UP")
        elif not rf_alive and link_ok:
            link_ok = False
            _pty_write(master_fd, b"[RF_LINK_LOST]\n")
            _pty_write(master_fd, b"[SBUS_LOST]\n")
            print("[EMU] RF link LOST")

        # ── CH: output at 10 Hz ───────────────────────────────────────────────
        if now - last_ch_out >= 0.1:
            with _ch_lock:
                ch = list(_channels)
            mode = _mode_str(ch) if rf_alive else "MANUAL"
            line = "CH:" + ",".join(str(v) for v in ch) + f" MODE:{mode}\n"
            _pty_write(master_fd, line.encode())
            last_ch_out = now

        # ── Read incoming data from rover/main.py (heartbeat packets) ─────────
        rlist, _, _ = select.select([master_fd], [], [], 0.02)
        if rlist:
            try:
                data = os.read(master_fd, 256)
                if data:
                    in_buf += data
                    # Parse complete <...> packets
                    while b">" in in_buf:
                        idx  = in_buf.index(b">")
                        pkt  = in_buf[:idx + 1].decode("utf-8", errors="ignore").strip()
                        in_buf = in_buf[idx + 1:]
                        if pkt.startswith("<HB:") and pkt.endswith(">"):
                            try:
                                n = int(pkt[4:-1])
                                _pty_write(master_fd,
                                           f"<HB:{(n + 1) % 10000}>\n".encode())
                            except ValueError:
                                pass
            except OSError:
                pass


# ─── Live status display ──────────────────────────────────────────────────────

def _status_loop() -> None:
    GR = "\033[92m"
    RD = "\033[91m"
    DM = "\033[2m"
    RS = "\033[0m"
    while True:
        now      = time.monotonic()
        rf_alive = (_last_rx > 0) and (now - _last_rx < LINK_TIMEOUT_S)
        age      = now - _last_rx if _last_rx else None
        with _ch_lock:
            ch = list(_channels)
        link_s = f"{GR}OK{RS}" if rf_alive else f"{RD}LOST{RS}"
        age_s  = f"{age:.2f}s" if age is not None else "---"
        mode   = _mode_str(ch) if rf_alive else "NO_LINK"
        print(
            f"\r[EMU] UDP:{link_s} age={age_s}  "
            f"THR:{ch[0]:4d} STR:{ch[1]:4d} CH9:{ch[8]:4d}  MODE:{mode}    ",
            end="", flush=True,
        )
        time.sleep(0.2)


# ─── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="RP2040 Receiver Emulator")
    parser.add_argument("--listen-port",  type=int, default=LISTEN_PORT,
                        help="UDP port to listen on (default 14550)")
    parser.add_argument("--master-sysid", type=int, default=MASTER_SYSID,
                        help="MAVLink sysid of the master rover (default 1)")
    args = parser.parse_args()

    if sys.platform == "win32":
        print("ERROR: Requires Linux (pty). Run on Jetson/RPi.")
        sys.exit(1)

    master_fd, slave_fd, slave_path = _open_pty()

    threading.Thread(
        target=_mav_listener,
        args=(args.listen_port, args.master_sysid),
        daemon=True, name="mav-listener",
    ).start()

    threading.Thread(
        target=_status_loop,
        daemon=True, name="status",
    ).start()

    print(f"\n=== RP2040 Receiver Emulator ===")
    print(f"UDP listen  : port {args.listen_port}  (master sysid={args.master_sysid})")
    print(f"Virtual pty : {slave_path}\n")
    print(f"Start Rover 2 with:\n")
    print(f"  ROVER_ID=2 UART_PORT={slave_path} \\")
    print(f"  GPS_PRIMARY_PORT=<sim_pty1> GPS_SECONDARY_PORT=<sim_pty2> \\")
    print(f"  python rover/main.py\n")

    try:
        _pty_loop(master_fd)
    except KeyboardInterrupt:
        print("\n[EMU] Exiting.")


if __name__ == "__main__":
    main()
