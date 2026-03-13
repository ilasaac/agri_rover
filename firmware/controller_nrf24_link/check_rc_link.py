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
from pathlib import Path

# ── ANSI colours ──────────────────────────────────────────────────────────────
R   = "\033[0m"
RED = "\033[31;1m"
YEL = "\033[33;1m"
GRN = "\033[32;1m"
CYN = "\033[36;1m"
WHT = "\033[37;1m"
DIM = "\033[2m"
MAG = "\033[35;1m"

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

PPM_MIN = 800
PPM_MAX = 2200

# ── Auto-detect port ──────────────────────────────────────────────────────────
def auto_detect_port() -> str:
    candidates = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    if candidates:
        return sorted(candidates)[0]
    raise RuntimeError(
        "No /dev/ttyACM* found. Specify port with --port COM5 or --port /dev/ttyACM1"
    )

# ── State ─────────────────────────────────────────────────────────────────────
state = {
    "rf_ok":         False,
    "rf_ever_ok":    False,   # went RF_LINK_OK at least once
    "mode":          "—",
    "channels":      [0] * 9,
    "payload_valid": False,   # True when all channels are in 800-2200
    "pkt_count":     0,
    "pkt_rate":      0.0,     # 2-second sliding window Hz
    "inst_rate":     0.0,     # instantaneous inter-packet rate
    "last_pkt_ts":   None,
    "events":        [],
    "raw_lines":     [],
}
state_lock = threading.Lock()
log_file   = None             # set in main() when --log is used

def _payload_valid(channels) -> bool:
    """Return True only if all 9 channels are in the valid PPM range."""
    return all(PPM_MIN <= v <= PPM_MAX for v in channels)

# ── Serial reader thread ───────────────────────────────────────────────────────
def reader_thread(ser: serial.Serial, show_raw: bool):
    pkt_times    = []
    last_pkt_now = None

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

        # ── Log every raw line to file if requested ───────────────────────
        if log_file:
            log_file.write(f"{time.strftime('%H:%M:%S.') + f'{int((now % 1)*1000):03d}'}  {line}\n")
            log_file.flush()

        with state_lock:
            if show_raw:
                state["raw_lines"].append(line)
                if len(state["raw_lines"]) > 5:
                    state["raw_lines"].pop(0)

            if line.startswith("CH:"):
                try:
                    ch_part, mode_part = line.split(" MODE:")
                    vals = [int(v) for v in ch_part[3:].split(",")]
                    valid = _payload_valid(vals)

                    state["channels"]      = vals
                    state["mode"]          = mode_part.strip()
                    state["payload_valid"] = valid
                    state["last_pkt_ts"]   = now
                    state["pkt_count"]    += 1

                    # 2-second sliding-window rate
                    pkt_times.append(now)
                    pkt_times = [t for t in pkt_times if now - t <= 2.0]
                    state["pkt_rate"] = len(pkt_times) / 2.0

                    # Instantaneous inter-packet rate
                    if last_pkt_now is not None:
                        gap = now - last_pkt_now
                        state["inst_rate"] = 1.0 / gap if gap > 0 else 0.0
                    last_pkt_now = now

                    if not valid:
                        # Only add one warning at a time (avoid flooding)
                        last_ev = state["events"][-1] if state["events"] else ""
                        if "INVALID" not in last_ev:
                            state["events"].append(
                                f"{MAG}[INVALID PAYLOAD] zeros — noise or SPI/wiring fault{R}"
                                f"  {DIM}{_ts()}{R}"
                            )
                except Exception:
                    pass

            elif "[RF_LINK_OK]" in line:
                state["rf_ok"]      = True
                state["rf_ever_ok"] = True
                state["events"].append(f"{GRN}[RF_LINK_OK]{R}  {DIM}{_ts()}{R}")

            elif "[LINK_LOST]" in line:
                state["rf_ok"]    = False
                state["mode"]     = "LINK_LOST"
                state["pkt_rate"] = 0.0
                state["events"].append(f"{RED}[LINK_LOST]{R}  {DIM}{_ts()}{R}")

            elif line.startswith("<HB:"):
                pass

            else:
                # Capture all other lines — boot messages, errors, etc.
                state["events"].append(f"{YEL}{line}{R}  {DIM}{_ts()}{R}")

            if len(state["events"]) > 10:
                state["events"].pop(0)

def _ts() -> str:
    return time.strftime("%H:%M:%S")

# ── Display ───────────────────────────────────────────────────────────────────
def render(port: str, show_raw: bool):
    try:
        while True:
            time.sleep(0.1)
            with state_lock:
                s             = dict(state)
                events        = list(s["events"])
                channels      = list(s["channels"])
                raw_lines     = list(s.get("raw_lines", []))
                payload_valid = s["payload_valid"]

            print("\033[2J\033[H", end="")

            # ── Header ──────────────────────────────────────────────────────
            print(f"{WHT}══ RC Link Check ══  port: {port}  {DIM}{_ts()}{R}")
            print()

            # ── RF link status ───────────────────────────────────────────────
            rf_colour = GRN if s["rf_ok"] else RED
            rf_label  = "RF LINK OK" if s["rf_ok"] else "LINK LOST"
            rate_str  = f"{s['pkt_rate']:.1f} Hz" if s["rf_ok"] else "  0.0 Hz"
            inst      = s["inst_rate"]
            # Flag suspiciously high rate (radio.available() stuck → SPI fault)
            rate_warn = ""
            if inst > 200:
                rate_warn = f"  {RED}RATE TOO HIGH — radio.available() stuck? check MISO wiring{R}"
            elif 0 < inst < 5 and s["rf_ok"]:
                rate_warn = f"  {YEL}rate low — interference or range?{R}"

            age = ""
            if s["last_pkt_ts"]:
                ms = int((time.time() - s["last_pkt_ts"]) * 1000)
                age = f"  last pkt: {ms} ms ago"

            print(f"  Status   {rf_colour}{rf_label}{R}   {WHT}{rate_str}{R}"
                  f"  inst: {inst:6.1f} Hz{DIM}{age}{R}{rate_warn}")

            # ── Payload validity warning ─────────────────────────────────────
            if not payload_valid and s["pkt_count"] > 0:
                print(f"  {MAG}WARNING  Payload out of range (got zeros) — "
                      f"noise or master not sending SBUS{R}")
            elif payload_valid:
                print(f"  {GRN}Payload  Valid PPM range (800–2200){R}")
            else:
                print(f"  {DIM}Payload  No packets yet{R}")

            # ── Mode ─────────────────────────────────────────────────────────
            mc = MODE_COLOUR.get(s["mode"], WHT)
            print(f"  Mode     {mc}{s['mode']}{R}")
            print(f"  Packets  {s['pkt_count']}")

            if not s["rf_ever_ok"] and s["pkt_count"] == 0:
                print(f"\n  {YEL}Waiting — no packets yet. "
                      f"Is the slave RP2040 connected and powered?{R}")
            elif not s["rf_ever_ok"] and s["pkt_count"] > 0:
                print(f"\n  {YEL}Packets arriving but [RF_LINK_OK] never received. "
                      f"Master may not be transmitting valid packets.{R}")

            print()

            # ── Channel bars ─────────────────────────────────────────────────
            if s["pkt_count"] > 0:
                print(f"  {DIM}{'Chan':<6} {'Value':>6}  {'1000':>4}{'':16}{'2000'}{R}")
                print(f"  {DIM}{'─'*46}{R}")
                for i, (name, val) in enumerate(zip(CHANNEL_NAMES, channels)):
                    # Bar: 0 = 1000 µs, 20 = 2000 µs
                    bar_len = int((val - 1000) / 1000 * 20) if val >= 1000 else 0
                    bar_len = max(0, min(20, bar_len))
                    bar = "█" * bar_len + "░" * (20 - bar_len)

                    # Colour coding
                    if val < PPM_MIN or val > PPM_MAX:
                        col = MAG            # out of range — bad
                    elif i == 2 and val < 1700:
                        col = RED            # SWA: EMERGENCY
                    elif i == 3 and val > 1700:
                        col = CYN            # SWB: autonomous requested
                    elif i in (0, 1):
                        col = WHT            # throttle / steering
                    else:
                        col = DIM

                    flag = ""
                    if i == 2 and val < 1700:
                        flag = f"  {RED}← EMERGENCY{R}"
                    elif i == 3 and val > 1700:
                        flag = f"  {CYN}← AUTO arm{R}"
                    elif i == 8 and 1250 < val <= 1750:
                        flag = f"  {GRN}← this rover selected{R}"

                    print(f"  {col}{name:<6} {val:>5}  {bar}{R}{flag}")
            else:
                print(f"  {DIM}No packets received yet{R}")

            print()

            # ── Events ───────────────────────────────────────────────────────
            if events:
                print(f"  {DIM}── Events ─────────────────────────────{R}")
                for ev in events[-6:]:
                    print(f"  {ev}")
                print()

            # ── Raw lines ────────────────────────────────────────────────────
            if show_raw and raw_lines:
                print(f"  {DIM}── Raw ────────────────────────────────{R}")
                for rl in raw_lines:
                    print(f"  {DIM}{rl}{R}")
                print()

            # ── Legend ───────────────────────────────────────────────────────
            print(f"  {DIM}Valid PPM: 1000–2000 µs  |  "
                  f"SWA(CH3)<1700→EMERGENCY  |  "
                  f"SWB(CH4)>1700→AUTO  |  "
                  f"SEL(CH9) 1250–1750→this rover active{R}")
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
    parser.add_argument("--log", metavar="FILE", default=None,
                        help="Log every raw serial line with timestamps to FILE")
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

    global log_file
    if args.log:
        log_file = open(args.log, "a")
        log_file.write(f"\n{'='*60}\n")
        log_file.write(f"Session start  {time.strftime('%Y-%m-%d %H:%M:%S')}  port={port}\n")
        log_file.write(f"{'='*60}\n")
        log_file.flush()
        print(f"Logging to {args.log}")

    print(f"Opened {port} at {args.baud} baud. Waiting for data…")
    time.sleep(0.5)

    t = threading.Thread(target=reader_thread, args=(ser, args.raw), daemon=True)
    t.start()

    render(port, args.raw)

    ser.close()
    if log_file:
        log_file.close()
    print("Done.")

if __name__ == "__main__":
    main()
