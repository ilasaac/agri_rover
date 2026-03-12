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


def ppm_blank(width: int = 12) -> str:
    return f"{DIM}{'─' * width}{RESET}  ---"


def render(channels: list[int], mode: str, sbus: str, hb_age: float | None,
           log: list[str]) -> None:
    out = ["\033[H"]   # cursor home (no clear = no flicker)

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

    out.append(
        f"\n  {BOLD}RP2040 UART Monitor{RESET}  "
        f"port={PORT}  baud={BAUD}                    \n"
        f"  Mode: {mode_col}{BOLD}{mode}{RESET}  "
        f"SBUS: {sbus_col}{sbus}{RESET}  "
        f"HB: {hb_col}{hb_str}{RESET}              \n"
        f"  {'─'*60}\n"
    )

    # Always show 16 channels in two columns (8 rows)
    for i in range(0, 16, 2):
        left_name  = CH_NAMES[i]
        right_name = CH_NAMES[i + 1]
        left_has   = i     < len(channels)
        right_has  = i + 1 < len(channels)
        left_bar   = ppm_bar(channels[i])     if left_has  else ppm_blank()
        right_bar  = ppm_bar(channels[i + 1]) if right_has else ppm_blank()
        out.append(
            f"  {left_name:>4}  {left_bar}    "
            f"{right_name:>4}  {right_bar}    \n"
        )

    # Raw line (fixed width, pad to overwrite longer previous content)
    raw = ",".join(str(v) for v in channels)
    out.append(f"\n  {DIM}raw: {raw:<80}{RESET}\n")
    out.append(f"  {DIM}last update: {time.strftime('%H:%M:%S')}{RESET}    \n")

    # Event log (last 6 lines, fixed height)
    out.append(f"  {'─'*60}\n  {DIM}events:{RESET}\n")
    for entry in log[-6:]:
        out.append(f"  {DIM}{entry:<70}{RESET}\n")
    # Pad to fixed height so old lines are overwritten
    for _ in range(6 - min(len(log), 6)):
        out.append(f"  {' ' * 70}\n")

    # Clear to end of screen (removes any leftover lines from prior renders)
    out.append("\033[J")

    sys.stdout.write("".join(out))
    sys.stdout.flush()


def main() -> None:
    channels: list[int] = []
    mode        = "---"
    sbus        = "---"
    hb_last_rx  = None
    hb_expected = -1
    hb_counter  = 0
    log: list[str] = []

    def add_log(msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        log.append(f"{ts}  {msg}")

    sys.stdout.write("\033[2J")   # clear screen once
    sys.stdout.flush()
    add_log(f"Connecting to {PORT} at {BAUD} baud…")
    render(channels, mode, sbus, None, log)

    while True:
        try:
            ser = serial.Serial(PORT, BAUD, timeout=0.1)
            ser.reset_input_buffer()
            add_log(f"Connected to {PORT}")
            render(channels, mode, sbus, None, log)

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
                            channels  = vals
                            mode      = mode_part.strip()
                            hb_age    = (time.time() - hb_last_rx) if hb_last_rx else None
                            render(channels, mode, sbus, hb_age, log)
                    except ValueError:
                        pass

                # ── Status messages ────────────────────────────────────────
                elif line in ("[SBUS_OK]", "[FAILSAFE_CLEARED]", "[RF_LINK_OK]"):
                    sbus = "OK"
                    add_log(line)
                elif line in ("[SBUS_LOST]", "[FAILSAFE]", "[RF_LINK_LOST]",
                              "[FRAME_LOST]"):
                    sbus = "LOST"
                    add_log(line)

                # ── Heartbeat echo ─────────────────────────────────────────
                elif line.startswith("<HB:") and line.endswith(">"):
                    try:
                        n = int(line[4:-1])
                        if hb_expected >= 0 and n == hb_expected:
                            hb_last_rx = time.time()
                    except ValueError:
                        pass

                # ── Anything else → event log ──────────────────────────────
                else:
                    add_log(f">> {line}")
                    hb_age = (time.time() - hb_last_rx) if hb_last_rx else None
                    render(channels, mode, sbus, hb_age, log)

        except serial.SerialException as exc:
            add_log(f"SerialException: {exc} — retry in 2 s")
            render(channels, mode, sbus, None, log)
            time.sleep(2.0)
        except KeyboardInterrupt:
            sys.stdout.write("\n\033[?25h")   # restore cursor
            print("Bye.")
            break


if __name__ == "__main__":
    main()
