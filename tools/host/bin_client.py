#!/usr/bin/env python3
"""Minimal CLI over xtellar binary CAN-FD protocol."""
from __future__ import annotations

import argparse
import sys
import time

from xt_proto import (
    STATUS_OK,
    Ack,
    Info,
    Telemetry,
    cmd_id,
    pack_dq,
    pack_info,
    pack_raw,
    pack_stop,
    pack_vfoc,
    parse_frame,
    tel_id,
)


def open_bus(interface: str):
    try:
        import can
    except ImportError:
        sys.exit("python-can required: pip install python-can")
    return can.Bus(interface="socketcan", channel=interface, fd=True), can


class BinaryClient:
    def __init__(self, bus, can_mod, node_id=1, verbose=False):
        self.bus = bus
        self.can = can_mod
        self.node_id = node_id
        self.verbose = verbose
        self._seq = 0

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def send_cmd(self, payload: bytes, timeout=1.0):
        mid = cmd_id(self.node_id)
        self.bus.send(
            self.can.Message(
                arbitration_id=mid,
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=True,
                data=payload,
            )
        )
        if self.verbose:
            print(f"> cmd id=0x{mid:03x} {payload.hex()}")
        deadline = time.monotonic() + timeout
        want = tel_id(self.node_id)
        info = None
        while time.monotonic() < deadline:
            frame = self.bus.recv(timeout=0.05)
            if frame is None or frame.arbitration_id != want:
                continue
            msg = parse_frame(bytes(frame.data))
            if msg is None:
                continue
            if isinstance(msg, Info):
                info = msg
                continue
            if isinstance(msg, Ack):
                return msg, info
        raise TimeoutError("no ACK")

    def probe(self, timeout=1.5):
        ack, info = self.send_cmd(pack_info(self._next_seq()), timeout=timeout)
        if ack.status != STATUS_OK or info is None:
            raise RuntimeError(
                f"probe failed: ack.status={ack.status} info={info}"
            )
        return info

    def recv_telem(self, timeout=1.0):
        want = tel_id(self.node_id)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.bus.recv(timeout=0.05)
            if frame is None or frame.arbitration_id != want:
                continue
            msg = parse_frame(bytes(frame.data))
            if isinstance(msg, Telemetry):
                return msg
        raise TimeoutError("no telemetry")


def main():
    p = argparse.ArgumentParser(description="xtellar binary CAN client")
    p.add_argument("--interface", default="can0")
    p.add_argument("--node", type=int, default=1)
    p.add_argument("--verbose", action="store_true")
    p.add_argument("--stop", action="store_true")
    p.add_argument("--info", action="store_true")
    p.add_argument("--dq", nargs=3, type=float, metavar=("ID", "IQ", "W"))
    p.add_argument("--vfoc", nargs=3, type=float, metavar=("TH", "V", "W"))
    p.add_argument("--raw", nargs=3, type=int, metavar=("A", "B", "C"))
    p.add_argument("--telem", action="store_true")
    p.add_argument("--stream", action="store_true")
    p.add_argument("--hz", type=float, default=10.0)
    args = p.parse_args()

    bus, can_mod = open_bus(args.interface)
    client = BinaryClient(bus, can_mod, args.node, args.verbose)
    try:
        if args.info:
            info = client.probe()
            print(
                f"motor={info.motor} fw={info.fw_version} node={info.node_id} "
                f"family={info.family} pwm={info.pwm_hz}Hz bus={info.bus_v}V "
                f"Imax={info.i_max_a}A poles={info.pole_pairs} "
                f"R={info.r_ohm}ohm L={info.l_h*1e6:.0f}uH"
            )
        if args.stop:
            ack, _ = client.send_cmd(pack_stop(client._next_seq()))
            print(f"ACK stop status={ack.status}")
        if args.dq is not None:
            id_a, iq_a, w = args.dq
            ack, _ = client.send_cmd(pack_dq(id_a, iq_a, w, client._next_seq()))
            print(f"ACK dq status={ack.status}")
        if args.vfoc is not None:
            th, v, w = args.vfoc
            ack, _ = client.send_cmd(pack_vfoc(th, v, w, client._next_seq()))
            print(f"ACK vfoc status={ack.status}")
        if args.raw is not None:
            a, b, c = args.raw
            ack, _ = client.send_cmd(pack_raw(a, b, c, client._next_seq()))
            print(f"ACK raw status={ack.status}")
        if args.telem or args.stream or not any(
            [args.stop, args.dq, args.vfoc, args.raw, args.info]
        ):
            if args.stream:
                period = 1.0 / args.hz if args.hz > 0 else 0.1
                while True:
                    t = client.recv_telem(timeout=1.0)
                    print(
                        f"mode={t.mode} id={t.id_a:+.3f} iq={t.iq_a:+.3f} "
                        f"iqref={t.iqref_a:+.3f} i1={t.i1_a:+.3f} "
                        f"cisr={int(t.cisr)} duty={t.duty_a}/{t.duty_b}/{t.duty_c}"
                    )
                    time.sleep(period)
            else:
                t = client.recv_telem(timeout=1.0)
                print(
                    f"mode={t.mode} flags=0x{t.flags:x} "
                    f"id={t.id_a:+.3f}A iq={t.iq_a:+.3f}A "
                    f"idref={t.idref_a:+.3f} iqref={t.iqref_a:+.3f} "
                    f"i1={t.i1_a:+.3f} i2={t.i2_a:+.3f} i3={t.i3_a:+.3f} "
                    f"th={t.theta_rad:.3f} w={t.omega_rad_s:.1f} "
                    f"vd={t.vd_v:.3f} vq={t.vq_v:.3f} "
                    f"duty={t.duty_a}/{t.duty_b}/{t.duty_c} bus={t.bus_v:.1f}V "
                    f"pwm={int(t.pwm_on)} cisr={int(t.cisr)}"
                )
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
