#!/usr/bin/env python3
"""
tools/catch_hb.py
Listens on UDP :14550 and prints every packet with its source IP.

Use this on the Jetson to verify that the MK32 Android app heartbeats
arrive over WiFi before wiring up dynamic GCS discovery in main.py.

Usage:
  python tools/catch_hb.py [--port 14550]
"""

import argparse
import socket
import time


def _parse_mavlink(data: bytes) -> str:
    """Return a human-readable description of a MAVLink packet (no library needed)."""
    if not data:
        return f"empty"

    # MAVLink v1  (start byte 0xFE)
    # [0]=0xFE [1]=len [2]=seq [3]=sysid [4]=compid [5]=msgid [6..]=payload
    if data[0] == 0xFE and len(data) >= 6:
        sysid  = data[3]
        compid = data[4]
        msgid  = data[5]
        label  = _msgid_label(msgid)
        return f"MAVLink v1  sysid={sysid} compid={compid} msg={msgid}({label})"

    # MAVLink v2  (start byte 0xFD)
    # [0]=0xFD [1]=len [2]=incompat [3]=compat [4]=seq [5]=sysid [6]=compid
    # [7][8][9]=msgid (24-bit LE) [10..]=payload
    if data[0] == 0xFD and len(data) >= 10:
        sysid  = data[5]
        compid = data[6]
        msgid  = data[7] | (data[8] << 8) | (data[9] << 16)
        label  = _msgid_label(msgid)
        return f"MAVLink v2  sysid={sysid} compid={compid} msg={msgid}({label})"

    # Unknown / raw
    return f"raw {len(data)} bytes: {data[:24].hex()}"


def _msgid_label(msgid: int) -> str:
    return {
        0:  "HEARTBEAT",
        1:  "SYS_STATUS",
        33: "GLOBAL_POSITION_INT",
        65: "RC_CHANNELS",
        70: "RC_CHANNELS_OVERRIDE",
        76: "COMMAND_LONG",
    }.get(msgid, "?")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=14550)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(1.0)
    sock.bind(("0.0.0.0", args.port))
    print(f"Listening on UDP :{args.port} — Ctrl+C to stop\n")

    try:
        while True:
            try:
                data, (ip, port) = sock.recvfrom(2048)
                desc = _parse_mavlink(data)
                print(f"[{time.strftime('%H:%M:%S')}]  {ip}:{port:<5}  {desc}")
            except socket.timeout:
                pass
    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
