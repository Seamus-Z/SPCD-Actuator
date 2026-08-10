#!/usr/bin/env python3
"""1 kHz host-side sinusoidal velocity command for bandwidth tests.

Default: ω = 200 * sin(2π * 0.5 * t) rad/s on can0.

Usage:
  # bring CAN up first if needed
  sudo bash tools/host/can_setup.sh can0 up

  # stop GUI vel stream first (only one host should command the motor)
  python3 tools/host/sine_vel.py
  python3 tools/host/sine_vel.py --freq 0.5 --amp 200 --rate 1000 --seconds 10

Ctrl+C sends stop.
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from pathlib import Path

HOST_ROOT = Path(__file__).resolve().parent
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from xt_proto import (  # noqa: E402
    CtrlReply,
    cmd_id,
    pack_stop,
    pack_servo,
    parse_frame,
    tel_id,
)


def open_bus(interface: str):
    try:
        import can
    except ImportError:
        sys.exit("python-can required: pip install python-can")
    return can.Bus(interface="socketcan", channel=interface, fd=True), can


def main() -> int:
    p = argparse.ArgumentParser(
        description="Stream a sinusoidal vel command at a fixed host rate"
    )
    p.add_argument("--iface", "--interface", default="can0", dest="iface")
    p.add_argument("--node", type=int, default=1)
    p.add_argument("--freq", type=float, default=0.5, help="sine frequency [Hz]")
    p.add_argument(
        "--amp",
        type=float,
        default=200.0,
        help="sine amplitude [rad/s], peak = ±amp",
    )
    p.add_argument("--rate", type=float, default=1000.0, help="command rate [Hz]")
    p.add_argument("--id", type=float, default=0.0, help="optional Id [A]")
    p.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="duration; 0 = until Ctrl+C",
    )
    p.add_argument(
        "--phase-deg",
        type=float,
        default=0.0,
        help="initial phase [deg]",
    )
    p.add_argument(
        "--status-hz",
        type=float,
        default=5.0,
        help="print CtrlReply status this often (0=off)",
    )
    args = p.parse_args()

    if args.freq <= 0.0:
        sys.exit("--freq must be > 0")
    if args.rate < 10.0:
        sys.exit("--rate must be >= 10")
    if args.amp < 0.0:
        sys.exit("--amp must be >= 0")

    peak_accel = abs(args.amp) * 2.0 * math.pi * args.freq
    print(
        f"sine vel: iface={args.iface} node={args.node} "
        f"f={args.freq:g} Hz amp=±{args.amp:g} rad/s rate={args.rate:g} Hz "
        f"peak|α|≈{peak_accel:.0f} rad/s²"
    )
    print("note: stop GUI vel stream before running; Ctrl+C -> stop")

    bus, can_mod = open_bus(args.iface)
    mid = cmd_id(args.node)
    tid = tel_id(args.node)
    seq = 0
    stop = False

    def _on_sig(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _on_sig)
    signal.signal(signal.SIGTERM, _on_sig)

    def send(payload: bytes):
        bus.send(
            can_mod.Message(
                arbitration_id=mid,
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=True,
                data=payload,
            )
        )

    period = 1.0 / args.rate
    t0 = time.monotonic()
    next_t = t0
    next_status = t0
    status_period = (1.0 / args.status_hz) if args.status_hz > 0.0 else None
    n_tx = 0
    last_cmd = 0.0
    last_meas = float("nan")
    late = 0

    try:
        while not stop:
            now = time.monotonic()
            if args.seconds > 0.0 and (now - t0) >= args.seconds:
                break

            t = now - t0
            phase = 2.0 * math.pi * args.freq * t + math.radians(args.phase_deg)
            omega = args.amp * math.sin(phase)
            last_cmd = omega
            seq = (seq + 1) & 0xFF
            send(pack_servo(omega, args.id, seq))
            n_tx += 1

            while True:
                frame = bus.recv(timeout=0.0)
                if frame is None:
                    break
                if frame.arbitration_id != tid:
                    continue
                msg = parse_frame(bytes(frame.data))
                if isinstance(msg, CtrlReply):
                    last_meas = msg.omega_mech_rad_s

            if status_period is not None and now >= next_status:
                err = (
                    last_meas - last_cmd
                    if last_meas == last_meas
                    else float("nan")
                )
                print(
                    f"t={t:6.3f}s cmd={last_cmd:+8.2f} "
                    f"meas={last_meas:+8.2f} err={err:+7.2f} "
                    f"tx={n_tx} late={late}",
                    flush=True,
                )
                next_status = now + status_period

            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                late += 1
                if delay < -0.05:
                    next_t = time.monotonic()
    finally:
        try:
            seq = (seq + 1) & 0xFF
            send(pack_stop(seq))
            time.sleep(0.002)
            seq = (seq + 1) & 0xFF
            send(pack_stop(seq))
        except Exception as exc:  # noqa: BLE001
            print(f"stop failed: {exc}", file=sys.stderr)
        bus.shutdown()
        elapsed = time.monotonic() - t0
        hz = n_tx / elapsed if elapsed > 0 else 0.0
        print(
            f"done: tx={n_tx} over {elapsed:.2f}s (~{hz:.1f} Hz) late={late}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
