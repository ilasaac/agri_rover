"""
tools/uart_monitor.py
Live monitor for RP2040 UART output.

Shows all channels (up to 16), mode, SBUS/RF status, and heartbeat.

Usage:
  python tools/uart_monitor.py [PORT] [BAUD]

  PORT defaults to /dev/ttyACM0
  BAUD defaults to 115200

Examples:
  python tools/uart_monitor.py
  python tools/uart_monitor.py /dev/ttyACM1
  python tools/uart_monitor.py /dev/ttyUSB0 57600
"""

import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

RESET  = "\033[0m"
BOLD   = "\033[1m"
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
DIM    = "\033[2m"

CH_NAMES = [
    "THR", "STR", "CH3", "CH4",
    "CH5", "CH6", "CH7", "CH8",
    "CH9", "CH10","CH11","CH12",
    "CH13","CH14","CH15","CH16",
]

def ppm_bar(val: int, width: int = 12) -> str:
    frac   = max(0.0, min(1.0, (val - 1000) / 1000.0))
    filled = round(frac * width)
    bar    = "█" * filled + "░" * (width - filled)
    if val < 1100:
        col = RED
    elif val > 1900:
        col = GREEN
    elif 1450 <= val <= 1550:
        col = DIM
    else:
        col = YELLOW
    return f"{col}{bar}{RESET} {val:4d}"


def render(channels: list[int], mode: str, sbus: str, hb_age: float | None) -> None:
    lines = []

    # Header
    hb_str = f"{hb_age:.1f}s" if hb_age is not None else "---"
    hb_col = GREEN if (hb_age is not None and hb_age < 2.0) else RED

    mode_col = {
        "MANUAL":    GREEN,
        "RELAY":     CYAN,
        "EMERGENCY": RED,
        "AUTO":      YELLOW,
    }.get(mode.upper(), DIM)

    sbus_col = GREEN if sbus == "OK" else (RED if sbus == "LOST" else DIM)

    lines.append(
        f"\033[H"   # cursor home
        f"\n  {BOLD}RP2040 UART Monitor{RESET}  "
        f"port={PORT}  baud={BAUD}\n"
        f"  Mode: {mode_col}{BOLD}{mode}{RESET}  "
        f"SBUS: {sbus_col}{sbus}{RESET}  "
        f"HB: {hb_col}{hb_str}{RESET}\n"
        f"  {'─'*54}\n"
    )

    # Channels in two columns
    n = max(len(channels), 8)
    for i in range(0, n, 2):
        left_name  = CH_NAMES[i]   if i   < len(CH_NAMES) else f"CH{i+1}"
        right_name = CH_NAMES[i+1] if i+1 < len(CH_NAMES) else f"CH{i+2}"
        left_val   = channels[i]   if i   < len(channels) else 0
        right_val  = channels[i+1] if i+1 < len(channels) else 0

        left_bar  = ppm_bar(left_val)
        right_bar = ppm_bar(right_val)

        lines.append(
            f"  {left_name:>4}  {left_bar}    "
            f"{right_name:>4}  {right_bar}"
        )

    # Raw line
    raw = ",".join(str(v) for v in channels)
    lines.append(f"\n  {DIM}raw: {raw}{RESET}")
    lines.append(f"  {DIM}last update: {time.strftime('%H:%M:%S')}{RESET}\n")

    print("\n".join(lines), end="", flush=True)


def main() -> None:
    channels: list[int] = [1500] * 8
    mode     = "---"
    sbus     = "---"
    hb_counter  = -1
    hb_last_rx  = None
    hb_expected = -1

    print(f"\033[2J")   # clear screen
    print(f"Connecting to {PORT} at {BAUD} baud…")

    while True:
        try:
            ser = serial.Serial(PORT, BAUD, timeout=0.1)
            ser.reset_input_buffer()
            print(f"\033[2J")   # clear on connect
            render(channels, mode, sbus, None)

            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                # ── CH: line ──────────────────────────────────────────────
                if line.startswith("CH:"):
                    body = line[3:]
                    mode_part = "MANUAL"
                    if " MODE:" in body:
                        body, mode_part = body.split(" MODE:", 1)
                    try:
                        vals = [int(v) for v in body.split(",")]
                        if len(vals) >= 2:
                            channels = vals
                            mode = mode_part.strip()
                            hb_age = (time.time() - hb_last_rx) if hb_last_rx else None
                            render(channels, mode, sbus, hb_age)
                    except ValueError:
                        pass

                # ── Status messages ────────────────────────────────────────
                elif line in ("[SBUS_OK]", "[FAILSAFE_CLEARED]", "[RF_LINK_OK]"):
                    sbus = "OK"
                elif line in ("[SBUS_LOST]", "[FAILSAFE]", "[RF_LINK_LOST]"):
                    sbus = "LOST"

                # ── Heartbeat echo ─────────────────────────────────────────
                elif line.startswith("<HB:") and line.endswith(">"):
                    try:
                        n = int(line[4:-1])
                        if hb_expected >= 0 and n == hb_expected:
                            hb_last_rx = time.time()
                    except ValueError:
                        pass

                # ── Anything else — print below the display ────────────────
                else:
                    print(f"\n  {DIM}>> {line}{RESET}", flush=True)

        except serial.SerialException as exc:
            print(f"\r[UART] {exc} — retry in 2 s   ", end="", flush=True)
            time.sleep(2.0)
        except KeyboardInterrupt:
            print("\nBye.")
            break


if __name__ == "__main__":
    main()
