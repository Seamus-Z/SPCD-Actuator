#!/usr/bin/env python3
"""Build and upload moteus-style encoder geometric compensation (256-point).

Inertial method (no second encoder):
  1) Spin at constant velocity
  2) Measure ω(θ) / mean(ω) - 1
  3) Integrate deviation vs θ → position correction table
  4) Quantize to int8 + peak scale and upload via CMD_ENC_COMP

Examples:
  # From an existing snap (decimate=16 @ ~20 rad/s covers ~1 rev)
  python3 tools/host/compensate_encoder.py --from-snap tools/host/snap_20.json

  # Live: spin at 40 rad/s for 8 s, then upload
  python3 tools/host/compensate_encoder.py --iface can0 --omega 40 --seconds 8

  # Analyze only (no upload)
  python3 tools/host/compensate_encoder.py --from-snap tools/host/snap_200.json --analyze-only
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
from pathlib import Path

HOST_ROOT = Path(__file__).resolve().parent
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from xt_proto import (  # noqa: E402
    STATUS_OK,
    Ack,
    CtrlReply,
    ENC_COMP_CHUNK_SIZE,
    ENC_COMP_TABLE_SIZE,
    cmd_id,
    pack_enc_comp_chunk,
    pack_enc_comp_clear,
    pack_enc_comp_commit,
    pack_stop,
    pack_vel,
    parse_frame,
    tel_id,
)


def open_bus(interface: str):
    try:
        import can
    except ImportError:
        sys.exit("python-can required: pip install python-can")
    return can.Bus(interface="socketcan", channel=interface, fd=True), can


def unwrap(xs):
    out = []
    carry = 0.0
    prev = None
    for x in xs:
        if prev is None:
            out.append(x)
        else:
            d = x - prev
            if d > math.pi:
                carry -= 2.0 * math.pi
            elif d < -math.pi:
                carry += 2.0 * math.pi
            out.append(x + carry)
        prev = x
    return out


def samples_from_snap_dict(snap: dict):
    """Build (frac, omega) rows from an /api/snap payload dict."""
    series = snap.get("series") or {}
    hz = float(snap.get("sample_hz") or 1.0)
    tm = unwrap(list(series.get("theta_mech_rad") or []))
    n = len(tm)
    if n < 32:
        raise RuntimeError("snap too short")
    dt = 1.0 / hz
    rows = []
    for i in range(1, n - 1):
        w = (tm[i + 1] - tm[i - 1]) / (2.0 * dt)
        th = tm[i]
        frac = (th / (2.0 * math.pi)) % 1.0
        rows.append((frac, w))
    return rows


def samples_from_snap(path: Path):
    return samples_from_snap_dict(json.loads(Path(path).read_text()))


def samples_from_live(bus, can_mod, node: int, omega: float, seconds: float, rate_hz: float):
    mid = cmd_id(node)
    tid = tel_id(node)
    seq = 0
    period = 1.0 / rate_hz
    t0 = time.monotonic()
    next_t = t0
    rows = []

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

    try:
        while time.monotonic() - t0 < seconds:
            seq = (seq + 1) & 0xFF
            send(pack_vel(omega, 0.0, seq))
            # drain replies
            while True:
                frame = bus.recv(timeout=0.0)
                if frame is None:
                    break
                if frame.arbitration_id != tid:
                    continue
                msg = parse_frame(bytes(frame.data))
                if isinstance(msg, CtrlReply):
                    # Prefer raw encoder fraction (sensor domain).
                    frac = (float(msg.enc_raw) / 65536.0) % 1.0
                    rows.append((frac, float(msg.omega_mech_rad_s)))
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()
    finally:
        seq = (seq + 1) & 0xFF
        send(pack_stop(seq))
        time.sleep(0.002)
        seq = (seq + 1) & 0xFF
        send(pack_stop(seq))
    return rows


def build_table(rows, bins: int = ENC_COMP_TABLE_SIZE):
    if len(rows) < bins:
        raise RuntimeError(f"need more samples ({len(rows)} < {bins})")
    # Bin average ω
    acc = [0.0] * bins
    cnt = [0] * bins
    for frac, w in rows:
        i = int(frac * bins) % bins
        acc[i] += w
        cnt[i] += 1
    # Fill gaps by neighbor lerp
    omega = [0.0] * bins
    for i in range(bins):
        if cnt[i] > 0:
            omega[i] = acc[i] / cnt[i]
        else:
            # search nearest
            for d in range(1, bins):
                a = (i - d) % bins
                b = (i + d) % bins
                if cnt[a] > 0 and cnt[b] > 0:
                    omega[i] = 0.5 * (acc[a] / cnt[a] + acc[b] / cnt[b])
                    break
                if cnt[a] > 0:
                    omega[i] = acc[a] / cnt[a]
                    break
                if cnt[b] > 0:
                    omega[i] = acc[b] / cnt[b]
                    break
    mean_w = sum(omega) / bins
    if abs(mean_w) < 1e-3:
        raise RuntimeError("mean velocity ~ 0; is the motor spinning?")
    # Relative velocity deviation (moteus)
    dev = [(w / mean_w) - 1.0 for w in omega]
    # Integrate vs angle (bin width = 2π/bins) → position correction [rad]
    # ∫ (ω/ω̄ - 1) dθ = correction in radians
    dth = 2.0 * math.pi / bins
    integ = [0.0] * bins
    run = 0.0
    for i in range(bins):
        run += dev[i] * dth
        integ[i] = run
    mean_i = sum(integ) / bins
    corr = [v - mean_i for v in integ]
    peak = max(abs(v) for v in corr)
    if peak < 1e-6:
        raise RuntimeError("correction ~ 0")
    table = []
    for v in corr:
        q = int(round(127.0 * v / peak))
        q = max(-127, min(127, q))
        table.append(q)
    # Stats for report
    # implied ω harmonic reduction estimate: use peak corr angle
    rel_std = (sum(d * d for d in dev) / bins) ** 0.5
    return {
        "table": table,
        "peak_rad": peak,
        "mean_omega": mean_w,
        "vel_dev_std": rel_std,
        "filled_bins": sum(1 for c in cnt if c > 0),
        "n_samples": len(rows),
    }


def upload_with_sender(send_cmd, table, peak_rad: float, clear_first: bool = True, next_seq=None):
    """Upload table using send_cmd(payload) -> Ack-like object with .status.

    next_seq: optional callable returning 0..255; otherwise seq auto-increments from 1.
    """
    seq = 0

    def _seq():
        nonlocal seq
        if next_seq is not None:
            return int(next_seq()) & 0xFF
        seq = (seq + 1) & 0xFF
        return seq

    if clear_first:
        ack = send_cmd(pack_enc_comp_clear(_seq()))
        if getattr(ack, "status", None) != STATUS_OK:
            raise RuntimeError(f"clear failed status={getattr(ack, 'status', ack)}")
    raw = bytes(struct.pack(f"{ENC_COMP_TABLE_SIZE}b", *table))
    for chunk in range(8):
        piece = raw[chunk * ENC_COMP_CHUNK_SIZE : (chunk + 1) * ENC_COMP_CHUNK_SIZE]
        ack = send_cmd(pack_enc_comp_chunk(chunk, piece, _seq()))
        if getattr(ack, "status", None) != STATUS_OK:
            raise RuntimeError(
                f"chunk {chunk} failed status={getattr(ack, 'status', ack)}"
            )
    ack = send_cmd(pack_enc_comp_commit(peak_rad, _seq()))
    if getattr(ack, "status", None) != STATUS_OK:
        raise RuntimeError(f"commit failed status={getattr(ack, 'status', ack)}")
    time.sleep(0.2)
    return {
        "ok": True,
        "peak_rad": float(peak_rad),
        "table_len": len(table),
    }


def upload(bus, can_mod, node: int, table, peak_rad: float, clear_first=True):
    mid = cmd_id(node)
    tid = tel_id(node)
    seq = 0

    def send_cmd(payload: bytes, timeout=1.0):
        nonlocal seq
        bus.send(
            can_mod.Message(
                arbitration_id=mid,
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=True,
                data=payload,
            )
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = bus.recv(timeout=0.05)
            if frame is None or frame.arbitration_id != tid:
                continue
            msg = parse_frame(bytes(frame.data))
            if isinstance(msg, Ack):
                return msg
        raise TimeoutError("no ACK for enc_comp")

    if clear_first:
        seq = (seq + 1) & 0xFF
        ack = send_cmd(pack_enc_comp_clear(seq))
        if ack.status != STATUS_OK:
            raise RuntimeError(f"clear failed status={ack.status}")
    raw = bytes(struct.pack(f"{ENC_COMP_TABLE_SIZE}b", *table))
    for chunk in range(8):
        seq = (seq + 1) & 0xFF
        piece = raw[chunk * ENC_COMP_CHUNK_SIZE : (chunk + 1) * ENC_COMP_CHUNK_SIZE]
        ack = send_cmd(pack_enc_comp_chunk(chunk, piece, seq))
        if ack.status != STATUS_OK:
            raise RuntimeError(f"chunk {chunk} failed status={ack.status}")
    seq = (seq + 1) & 0xFF
    ack = send_cmd(pack_enc_comp_commit(peak_rad, seq))
    if ack.status != STATUS_OK:
        raise RuntimeError(f"commit failed status={ack.status}")
    # give main loop time to Persist
    time.sleep(0.2)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--from-snap", type=Path, help="build table from snap JSON")
    ap.add_argument("--omega", type=float, default=40.0, help="live spin speed [rad/s]")
    ap.add_argument("--seconds", type=float, default=8.0, help="live capture duration")
    ap.add_argument("--rate", type=float, default=200.0, help="live command/sample rate")
    ap.add_argument("--analyze-only", action="store_true", help="do not upload")
    ap.add_argument("--write-table", type=Path, help="save int8 table JSON")
    ap.add_argument("--clear", action="store_true", help="only clear compensation on device")
    args = ap.parse_args()

    if args.clear:
        bus, can_mod = open_bus(args.iface)
        try:
            mid = cmd_id(args.node)
            tid = tel_id(args.node)
            bus.send(
                can_mod.Message(
                    arbitration_id=mid,
                    is_extended_id=False,
                    is_fd=True,
                    bitrate_switch=True,
                    data=pack_enc_comp_clear(1),
                )
            )
            deadline = time.monotonic() + 1.0
            ok = False
            while time.monotonic() < deadline:
                fr = bus.recv(timeout=0.05)
                if fr is None or fr.arbitration_id != tid:
                    continue
                msg = parse_frame(bytes(fr.data))
                if isinstance(msg, Ack):
                    print(f"clear ack status={msg.status}")
                    ok = msg.status == STATUS_OK
                    break
            if not ok:
                sys.exit("clear failed")
            time.sleep(0.2)
        finally:
            bus.shutdown()
        print("encoder compensation cleared")
        return 0

    if args.from_snap:
        rows = samples_from_snap(args.from_snap)
        print(f"loaded {len(rows)} samples from {args.from_snap}")
    else:
        bus, can_mod = open_bus(args.iface)
        print(f"live capture ω={args.omega} for {args.seconds}s @ {args.rate} Hz")
        try:
            rows = samples_from_live(bus, can_mod, args.node, args.omega, args.seconds, args.rate)
        finally:
            bus.shutdown()
        print(f"captured {len(rows)} live samples")

    result = build_table(rows)
    print(
        f"mean_ω={result['mean_omega']:.3f} rad/s  "
        f"vel_dev_std={100*result['vel_dev_std']:.2f}%  "
        f"peak|Δθ|={result['peak_rad']:.4f} rad "
        f"({result['peak_rad']*180/math.pi:.2f} deg)  "
        f"bins={result['filled_bins']}/{ENC_COMP_TABLE_SIZE}"
    )
    if args.write_table:
        args.write_table.write_text(json.dumps({
            "peak_rad": result["peak_rad"],
            "table": result["table"],
            "mean_omega": result["mean_omega"],
            "vel_dev_std": result["vel_dev_std"],
        }, indent=2))
        print(f"wrote {args.write_table}")

    if args.analyze_only:
        return 0

    bus, can_mod = open_bus(args.iface)
    try:
        print("uploading 256-point table…")
        upload(bus, can_mod, args.node, result["table"], result["peak_rad"])
        print("upload+commit ok (NVS persist deferred on device)")
    finally:
        bus.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
