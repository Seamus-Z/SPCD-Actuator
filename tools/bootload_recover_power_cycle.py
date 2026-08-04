#!/usr/bin/env python3
"""Recover CAN bootload when APP floods telemetry and gs_usb cannot TX BOOT.

Usage:
  1) Run this script first
  2) Power-cycle the motor controller while it runs
  3) On success it flashes bazel-bin/fw/app.bin and resets into APP
"""

from __future__ import annotations

import argparse
import struct
import socket
import sys
import time
from pathlib import Path

PF_CAN = 29
CAN_RAW = 1
SOL_CAN_RAW = 101
CAN_RAW_FILTER = 1


def tx_only_socket(channel: str):
    sock = socket.socket(PF_CAN, socket.SOCK_RAW, CAN_RAW)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2 * 1024 * 1024)
    sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, b"")
    sock.bind((channel,))
    return sock


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--flash", default="bazel-bin/fw/app.bin")
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()

    image = Path(args.flash).read_bytes()
    if not image:
        sys.exit("empty image")

    import can

    repo = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(repo))
    from tools.bootload_test import BootloaderClient

    print(f"Spamming BOOT on {args.interface}.")
    print(">>> Power-cycle the motor board NOW (unplug/replug power) <<<")
    print("Ctrl+C to abort")

    sock = tx_only_socket(args.interface)
    frame = struct.pack("=IB3x8s", 0x7E, 4, b"BOOT" + bytes(4))

    deadline = time.monotonic() + args.timeout
    sent = 0
    while time.monotonic() < deadline:
        try:
            sock.send(frame)
            sent += 1
        except OSError:
            pass

        if sent % 40 == 0:
            bus = None
            try:
                bus = can.Bus(interface="socketcan", channel=args.interface, fd=True)
                client = BootloaderClient(bus, can, verbose=False)
                t0 = time.monotonic()
                while time.monotonic() - t0 < 0.05:
                    bus.recv(0)
                client.poll(timeout=0.25, retries=2)
                print(f"\nBootloader responded after ~{sent} BOOT attempts")
                bus.shutdown()
                sock.close()

                bus = can.Bus(interface="socketcan", channel=args.interface, fd=True)
                client = BootloaderClient(bus, can, verbose=True)
                client.drain_banner()
                client.flash(image)
                bus.shutdown()
                print("Recover flash OK")
                return
            except Exception:
                if bus is not None:
                    try:
                        bus.shutdown()
                    except Exception:
                        pass
        time.sleep(0.01)

    sock.close()
    sys.exit("timeout: no bootloader. Power-cycle while script runs, or use SWD.")


if __name__ == "__main__":
    main()
