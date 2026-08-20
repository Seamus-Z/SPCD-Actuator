#!/usr/bin/env python3
"""Minimal CLI over xtellar binary CAN-FD protocol."""
from __future__ import annotations

import argparse
import sys
import time

from xt_proto import (
    STATUS_OK,
    Ack,
    CalTelem,
    CtrlReply,
    Info,
    Telemetry,
    cmd_id,
    pack_cal_abort,
    pack_cal_enc,
    pack_info,
    pack_query,
    pack_servo,
    pack_stop,
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
        self.last_cal = None

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
            if isinstance(msg, CalTelem):
                self.last_cal = msg
                continue
            # Query/Stop/Servo reply with CtrlReply instead of Ack.
            if isinstance(msg, (Ack, CtrlReply)):
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

    def query_ctrl(self, timeout=1.0):
        """Send Query; firmware replies with CtrlReply (replaces free-run Tel)."""
        reply, _ = self.send_cmd(pack_query(self._next_seq()), timeout=timeout)
        if not isinstance(reply, CtrlReply):
            raise RuntimeError(f"unexpected reply type: {type(reply)}")
        return reply

    def cal_enc(self, current_a=1.0, omega_elec_rad_s=40.0, timeout=90.0):
        """Encoder phase spin calibration: spins both ways, auto-persists."""
        self.last_cal = None
        self.send_cmd(pack_cal_enc(current_a, omega_elec_rad_s, self._next_seq()),
                      timeout=2.0)
        state_names = {1: "sense", 2: "fwd", 3: "rev", 4: "DONE", 5: "FAILED"}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.query_ctrl(timeout=2.0)
            cal = self.last_cal
            if cal is not None:
                name = state_names.get(cal.state, f"state{cal.state}")
                print(f"cal state={name} progress={cal.progress:.0%} "
                      f"offset={cal.offset_rad*1e3:+.1f}mrad "
                      f"residual={cal.residual_rad*1e3:.1f}mrad sign={cal.sign}")
                if cal.state == 4:
                    print(f"DONE: offset={cal.offset_rad*1e3:+.1f} mrad "
                          f"residual={cal.residual_rad*1e3:.1f} mrad "
                          f"sign={cal.sign} ok={cal.ok} "
                          f"persisted={cal.persisted} samples={cal.sample_count}")
                    return True
                if cal.state == 5:
                    print("calibration FAILED")
                    return False
            time.sleep(0.2)
        print("calibration timeout")
        return False


def main():
    p = argparse.ArgumentParser(description="xtellar binary CAN client")
    p.add_argument("--interface", default="can0")
    p.add_argument("--node", type=int, default=1)
    p.add_argument("--verbose", action="store_true")
    p.add_argument("--stop", action="store_true")
    p.add_argument("--info", action="store_true")
    p.add_argument("--servo", nargs=2, type=float, metavar=("OMEGA", "ID"),
                   help="servo velocity [rad/s] and d-axis reference [A]")
    p.add_argument("--telem", action="store_true")
    p.add_argument("--stream", action="store_true")
    p.add_argument("--hz", type=float, default=10.0)
    p.add_argument("--cal-enc", action="store_true",
                   help="run encoder phase spin calibration")
    p.add_argument("--cal-current", type=float, default=1.0,
                   help="calibration alignment current [A]")
    p.add_argument("--cal-omega", type=float, default=40.0,
                   help="calibration electrical speed [rad/s]")
    p.add_argument("--cal-abort", action="store_true",
                   help="abort any running calibration")
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
        if args.cal_abort:
            reply, _ = client.send_cmd(pack_cal_abort(client._next_seq()))
            print(f"ACK cal abort status={reply.status}")
        if args.cal_enc:
            print(f"encoder phase calibration: {args.cal_current}A, "
                  f"{args.cal_omega} rad/s elec (motor will spin ~6 turns)")
            ok = client.cal_enc(args.cal_current, args.cal_omega)
            sys.exit(0 if ok else 1)
        if args.stop:
            reply, _ = client.send_cmd(pack_stop(client._next_seq()))
            print(f"ACK stop status={reply.status}")
        if args.servo is not None:
            omega, id_a = args.servo
            reply, _ = client.send_cmd(pack_servo(omega, id_a, client._next_seq()))
            print(f"ACK servo status={reply.status}")
        if args.telem or args.stream or not any(
            [args.stop, args.servo, args.info]
        ):
            if args.stream:
                period = 1.0 / args.hz if args.hz > 0 else 0.1
                while True:
                    t = client.query_ctrl()
                    print(
                        f"mode={t.mode} st={t.status} flags=0x{t.flags:02x} "
                        f"pwm={int(t.pwm_on)} "
                        f"id={t.id_a:+.3f} iq={t.iq_a:+.3f} "
                        f"iqref={t.iqref_a:+.3f} w={t.omega_mech_rad_s:+.1f} "
                        f"wcmd={t.omega_cmd_rad_s:+.1f} th={t.theta_mech_rad:.3f} "
                        f"bus={t.bus_v:.1f}V enc={t.enc_ok} spk={t.enc_spike}"
                    )
                    time.sleep(period)
            else:
                t = client.query_ctrl()
                print(
                    f"mode={t.mode} st={t.status} flags=0x{t.flags:x} "
                    f"id={t.id_a:+.3f}A iq={t.iq_a:+.3f}A "
                    f"idref={t.idref_a:+.3f} iqref={t.iqref_a:+.3f} "
                    f"th_elec={t.theta_elec_rad:.3f} th_mech={t.theta_mech_rad:.3f} "
                    f"w={t.omega_mech_rad_s:+.1f} wcmd={t.omega_cmd_rad_s:+.1f} "
                    f"vd={t.vd_v:.3f} vq={t.vq_v:.3f} "
                    f"bus={t.bus_v:.1f}V headroom={t.voltage_headroom_v:.1f}V "
                    f"pwm={int(t.pwm_on)} enc_ok={t.enc_ok} raw={t.enc_raw}"
                )
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
