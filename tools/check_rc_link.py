#!/usr/bin/env python3
"""
check_rc_link.py  —  Diagnostic monitor for controller_nrf24_link slave firmware.

Connects to the RP2040 slave over USB serial and displays the RF link health,
channel values, mode, and packet rate in real time.

Usage:
    python check_rc_link.py                     # auto-detect /dev/ttyACM*
    python check_rc_link.py --port /dev/ttyACM1
    python check_rc_link.py --port COM5         # Windows
    python check_rc_link.py --port /dev/ttyACM0 --raw  # also print raw lines
"""

import argparse
import glob
import sys
import time
import threading
import serial

# ── ANSI colours ──────────────────────────────────────────────────────────────
R   = "\033[0m"
RED = "\033[31;1m"
YEL = "\033[33;1m"
GRN = "\033[32;1m"
CYN = "\033[36;1m"
WHT = "\033[37;1m"
DIM = "\033[2m"

MODE_COLOUR = {
    "EMERGENCY":    RED,
    "LINK_LOST":    RED,
    "AUTO-NO-HB":   YEL,
    "AUTO-TIMEOUT": YEL,
    "STANDBY":      DIM,
    "AUTONOMOUS":   CYN,
    "MANUAL":       GRN,
    "RELAY":        CYN,
}

CHANNEL_NAMES = [
    "THR", "STR", "SWA", "SWB", "CH5", "CH6", "CH7", "CH8", "SEL"
]

# ── Auto-detect port ───────────────────────────────────────────────────────────
def auto_detect_port() -> str:
    candidates = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    if candidates:
        return sorted(candidates)[0]
    # Windows fallback hint
    raise RuntimeError(
        "No /dev/ttyACM* found. Specify port with --port COM5 or --port /dev/ttyACM1"
    )

# ── State ─────────────────────────────────────────────────────────────────────
state = {
    "rf_ok":        False,
    "mode":         "—",
    "channels":     [0] * 9,
    "pkt_count":    0,
    "pkt_rate":     0.0,
    "last_pkt_ts":  None,
    "events":       [],          # recent status events
    "raw_lines":    [],
}
state_lock = threading.Lock()

# ── Serial reader thread ───────────────────────────────────────────────────────
def reader_thread(ser: serial.Serial, show_raw: bool):
    pkt_times = []  # sliding window of packet timestamps

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
        except serial.SerialException as e:
            with state_lock:
                state["events"].append(f"{RED}[SERIAL ERROR] {e}{R}")
            break
        if not line:
            continue

        now = time.time()

        with state_lock:
            if show_raw:
                state["raw_lines"].append(line)
                if len(state["raw_lines"]) > 5:
                    state["raw_lines"].pop(0)

            if line.startswith("CH:"):
                # CH:thr,str,ch3,ch4,ch5,ch6,ch7,ch8,ch9 MODE:str
                try:
                    ch_part, mode_part = line.split(" MODE:")
                    vals = [int(v) for v in ch_part[3:].split(",")]
                    state["channels"] = vals
                    state["mode"]     = mode_part.strip()
                    state["last_pkt_ts"] = now
                    state["pkt_count"] += 1

                    pkt_times.append(now)
                    pkt_times = [t for t in pkt_times if now - t <= 2.0]
                    state["pkt_rate"] = len(pkt_times) / 2.0
                except Exception:
                    pass

            elif "[RF_LINK_OK]" in line:
                state["rf_ok"] = True
                state["events"].append(f"{GRN}[RF_LINK_OK]{R}  {DIM}{_ts()}{R}")

            elif "[LINK_LOST]" in line:
                state["rf_ok"] = False
                state["mode"]  = "LINK_LOST"
                state["pkt_rate"] = 0.0
                state["events"].append(f"{RED}[LINK_LOST]{R}  {DIM}{_ts()}{R}")

            elif line.startswith("<HB:"):
                pass   # heartbeat echo — ignore in display

            elif line.startswith("["):
                state["events"].append(f"{YEL}{line}{R}  {DIM}{_ts()}{R}")

            if len(state["events"]) > 8:
                state["events"].pop(0)

def _ts() -> str:
    return time.strftime("%H:%M:%S")

# ── Display ────────────────────────────────────────────────────────────────────
def render(port: str, show_raw: bool):
    try:
        while True:
            time.sleep(0.1)
            with state_lock:
                s = dict(state)
                events   = list(s["events"])
                channels = list(s["channels"])
                raw_lines = list(s.get("raw_lines", []))

            # Clear screen
            print("\033[2J\033[H", end="")

            # Header
            print(f"{WHT}══ RC Link Check ══  port: {port}  {DIM}{_ts()}{R}")
            print()

            # RF link status
            rf_colour = GRN if s["rf_ok"] else RED
            rf_label  = "RF LINK OK" if s["rf_ok"] else "LINK LOST"
            rate_str  = f"{s['pkt_rate']:.1f} Hz" if s["rf_ok"] else "  0.0 Hz"
            age = ""
            if s["last_pkt_ts"]:
                ms = int((time.time() - s["last_pkt_ts"]) * 1000)
                age = f"  last pkt: {ms} ms ago"

            print(f"  Status  {rf_colour}{rf_label}{R}   {WHT}{rate_str}{R}{DIM}{age}{R}")

            # Mode
            mc = MODE_COLOUR.get(s["mode"], WHT)
            print(f"  Mode    {mc}{s['mode']}{R}")
            print(f"  Packets {s['pkt_count']}")
            print()

            # Channel bar display
            if channels and len(channels) == 9:
                print(f"  {DIM}{'Chan':<6} {'Value':>6}  {'Bar'}{R}")
                print(f"  {DIM}{'─'*40}{R}")
                for i, (name, val) in enumerate(zip(CHANNEL_NAMES, channels)):
                    bar_len = int((val - 1000) / 1000 * 20)
                    bar_len = max(0, min(20, bar_len))
                    bar = "█" * bar_len + "░" * (20 - bar_len)

                    # Highlight SWA/SWB for emergency/auto detection
                    if i == 2 and val < 1700:
                        col = RED
                    elif i == 3 and val > 1700:
                        col = CYN
                    elif i in (0, 1):
                        col = WHT
                    else:
                        col = DIM

                    print(f"  {col}{name:<6} {val:>5}  {bar}{R}")
            else:
                print(f"  {DIM}Waiting for first packet…{R}")

            print()

            # Recent events
            if events:
                print(f"  {DIM}── Recent events ──────────────────────{R}")
                for ev in events[-6:]:
                    print(f"  {ev}")
                print()

            # Raw lines (--raw mode)
            if show_raw and raw_lines:
                print(f"  {DIM}── Raw output ─────────────────────────{R}")
                for rl in raw_lines:
                    print(f"  {DIM}{rl}{R}")
                print()

            # Legend
            print(f"  {DIM}SWA(CH3) < 1700 → EMERGENCY   "
                  f"SWB(CH4) > 1700 → AUTO   "
                  f"SEL(CH9) 1250-1750 → this rover{R}")
            print(f"  {DIM}Ctrl-C to quit{R}")

    except KeyboardInterrupt:
        pass

# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="RC link health check for slave RP2040")
    parser.add_argument("--port", default=None,
                        help="Serial port (e.g. /dev/ttyACM0, COM5). Auto-detected if omitted.")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default 115200 — USB CDC-ACM ignores this)")
    parser.add_argument("--raw", action="store_true",
                        help="Also show the last 5 raw lines from the RP2040")
    args = parser.parse_args()

    port = args.port
    if port is None:
        try:
            port = auto_detect_port()
            print(f"Auto-detected port: {port}")
        except RuntimeError as e:
            print(f"ERROR: {e}", file=sys.stderr)
            sys.exit(1)

    try:
        ser = serial.Serial(port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Opened {port} at {args.baud} baud. Waiting for data…")
    time.sleep(0.5)

    t = threading.Thread(target=reader_thread, args=(ser, args.raw), daemon=True)
    t.start()

    render(port, args.raw)

    ser.close()
    print("Done.")

if __name__ == "__main__":
    main()
