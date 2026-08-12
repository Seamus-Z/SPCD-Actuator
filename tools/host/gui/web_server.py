#!/usr/bin/env python3
"""xtellar local web GUI (binary CAN-FD).

  python3 tools/host/gui/web_server.py
  open http://127.0.0.1:8765
  在页面里扫描 / 选择 / 连接 CAN 口；连接后自动探测电机 Info
"""
from __future__ import annotations

import argparse
import json
import math
import mimetypes
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from collections import deque
from queue import Empty, Queue
from contextlib import redirect_stderr
from io import StringIO

GUI_ROOT = Path(__file__).resolve().parent
HOST_ROOT = GUI_ROOT.parent
STATIC_DIR = GUI_ROOT / "static"
sys.path.insert(0, str(HOST_ROOT))
import snap_analysis as _snap_an  # noqa: E402

from xt_proto import (  # noqa: E402
    CMD_CONF,
    CMD_INFO,
    CMD_QUERY,
    CMD_SNAP,
    CONF_GROUP_ALL,
    CONF_GROUP_BY_NAME,
    CONF_GROUP_ENCODER,
    CONF_GROUP_CAL,
    CONF_GROUP_FOC,
    CONF_GROUP_MOTOR,
    CONF_GROUP_SERVO,
    CONF_OP_GET,
    SNAP_CHANNEL_KEYS,
    SNAP_CHANNELS,
    SNAP_MAX_SAMPLES,
    STATUS_OK,
    STATUS_NOT_RUN,
    Ack,
    CalTelem,
    ConfReply,
    CtrlReply,
    EncTelem,
    Info,
    SnapData,
    SnapMeta,
    Telemetry,
    conf_group_id,
    pack_query,
    pack_cal_abort,
    pack_cal_bemf,
    pack_cal_enc,
    pack_cal_lock,
    pack_cal_r,
    pack_cal_l,
    pack_cal_cogging,
    pack_conf,
    pack_enc_comp_clear,
    pack_servo,
    pack_mit,
    pack_current,
    pack_info,
    pack_snap,
    pack_stop,
    parse_conf,
    parse_frame,
    cmd_id,
    tel_id,
)
import compensate_encoder as _enc_comp  # noqa: E402

# Linux ARPHRD_CAN
_ARPHRD_CAN = "280"

_CMD_NAMES = {
    0: "stop",
    4: "info",
    5: "snap",
    6: "servo",
    7: "cal",
    8: "query",
    9: "enc_comp",
    10: "conf",
}


_CAL_STATE_NAMES = {
    0: "idle",
    1: "sense",
    2: "fwd",
    3: "rev",
    4: "done",
    5: "failed",
    6: "locking",
    7: "bemf_run",
    8: "r_run",
    9: "l_run",
    10: "cogging_run",
}
_MODE_NAMES = {
    0: "stop",
    1: "servo",
    2: "cal",
    3: "current",
    4: "mit",
}


class MsgLog:
    """Ring buffer of host/device messages for the GUI console."""

    def __init__(self, maxlen: int = 500):
        self.maxlen = maxlen
        self._lock = threading.Lock()
        self._items: list[dict] = []
        self._next_id = 1
        self._last_telem_log = 0.0

    def clear(self):
        with self._lock:
            self._items.clear()

    def push(self, kind: str, text: str):
        with self._lock:
            item = {
                "id": self._next_id,
                "ts": time.time(),
                "kind": kind,
                "text": text,
            }
            self._next_id += 1
            self._items.append(item)
            if len(self._items) > self.maxlen:
                self._items = self._items[-self.maxlen :]

    def since(self, after_id: int) -> list[dict]:
        with self._lock:
            return [m for m in self._items if m["id"] > after_id]

    def maybe_telem(self, t: Telemetry, period_s: float = 0.2):
        now = time.monotonic()
        if now - self._last_telem_log < period_s:
            return
        self._last_telem_log = now
        mode = _MODE_NAMES.get(t.mode, str(t.mode))
        self.push(
            "RX",
            (
                f"TEL seq={t.seq} mode={mode} "
                f"Id={t.id_a:+.3f}/{t.idref_a:+.3f} "
                f"Iq={t.iq_a:+.3f}/{t.iqref_a:+.3f} "
                f"cisr={int(t.cisr)} pwm={int(t.pwm_on)}"
            ),
        )

    def maybe_ctrl(self, r: CtrlReply, period_s: float = 0.2):
        now = time.monotonic()
        if now - self._last_telem_log < period_s:
            return
        self._last_telem_log = now
        mode = _MODE_NAMES.get(r.mode, str(r.mode))
        cname = _CMD_NAMES.get(r.cmd, str(r.cmd))
        self.push(
            "RX",
            (
                f"REPLY cmd={cname} seq={r.seq} st={r.status} mode={mode} "
                f"Id={r.id_a:+.3f}/{r.idref_a:+.3f} "
                f"Iq={r.iq_a:+.3f}/{r.iqref_a:+.3f} "
                f"ω={r.omega_mech_rad_s:+.2f} enc={r.enc_raw} "
                f"spike={r.enc_spike} "
                f"cisr={int(r.cisr)} pwm={int(r.pwm_on)}"
            ),
        )


def scan_can_ports() -> list[dict]:
    """List SocketCAN netdevs under /sys/class/net."""
    ports: list[dict] = []
    net = Path("/sys/class/net")
    if not net.is_dir():
        return ports
    for entry in sorted(net.iterdir()):
        typ = entry / "type"
        if not typ.is_file():
            continue
        try:
            if typ.read_text().strip() != _ARPHRD_CAN:
                continue
        except OSError:
            continue
        state = "unknown"
        oper = entry / "operstate"
        if oper.is_file():
            try:
                state = oper.read_text().strip()
            except OSError:
                pass
        ports.append({"name": entry.name, "state": state})
    return ports


def _run_ip(args: list[str]) -> tuple[bool, str]:
    """Run `ip ...`; try plain, sudo -n, then pkexec (GUI auth)."""
    last_err = ""
    candidates = [
        ["ip", *args],
        ["sudo", "-n", "ip", *args],
    ]
    # Desktop password prompt when available.
    if Path("/usr/bin/pkexec").is_file():
        candidates.append(["pkexec", "ip", *args])

    for cmd in candidates:
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        except Exception as exc:  # noqa: BLE001
            last_err = str(exc)
            continue
        if r.returncode == 0:
            return True, ""
        last_err = (r.stderr or r.stdout or "").strip() or f"exit {r.returncode}"
        # sudo -n password required → try next candidate
        if "密码" in last_err or "password" in last_err.lower():
            continue
    return False, last_err


def _iface_state(interface: str) -> str:
    oper = Path(f"/sys/class/net/{interface}/operstate")
    if not oper.is_file():
        return "missing"
    try:
        return oper.read_text().strip()
    except OSError:
        return "unknown"


def bring_up_can(
    interface: str,
    bitrate: int = 1_000_000,
    dbitrate: int = 2_000_000,
) -> tuple[bool, str]:
    """Configure SocketCAN FD and set link UP."""
    iface = (interface or "").strip()
    if not iface:
        return False, "empty interface"
    if not Path(f"/sys/class/net/{iface}").is_dir():
        return False, f"{iface} 不存在（先插适配器 / 加载驱动）"

    # already up → nothing to do
    if _iface_state(iface) == "up":
        return True, ""

    # down first so type can be (re)applied
    _run_ip(["link", "set", iface, "down"])
    ok, err = _run_ip(
        [
            "link",
            "set",
            iface,
            "type",
            "can",
            "bitrate",
            str(bitrate),
            "dbitrate",
            str(dbitrate),
            "fd",
            "on",
        ]
    )
    if not ok:
        hint = (
            f"配置 {iface} 需要管理员权限。请在终端执行一次：\n"
            f"sudo ip link set {iface} down\n"
            f"sudo ip link set {iface} type can bitrate {bitrate} "
            f"dbitrate {dbitrate} fd on\n"
            f"sudo ip link set {iface} up\n"
            f"或: sudo bash tools/host/can_setup.sh {iface} up\n"
            f"详情: {err}"
        )
        return False, hint
    ok, err = _run_ip(["link", "set", iface, "up"])
    if not ok:
        hint = (
            f"拉起 {iface} 失败。请手动：\n"
            f"sudo ip link set {iface} up\n"
            f"详情: {err}"
        )
        return False, hint
    return True, ""


def bring_down_can(interface: str) -> tuple[bool, str]:
    iface = (interface or "").strip()
    if not iface:
        return False, "empty interface"
    if not Path(f"/sys/class/net/{iface}").is_dir():
        return True, ""  # already gone
    if _iface_state(iface) == "down":
        return True, ""
    ok, err = _run_ip(["link", "set", iface, "down"])
    if not ok:
        return False, (
            f"关闭 {iface} 失败（可能需要 sudo）：\n"
            f"sudo ip link set {iface} down\n"
            f"详情: {err}"
        )
    return True, ""


class CanBridge:
    def __init__(self, node_id: int = 1):
        try:
            import can
        except ImportError:
            sys.exit("python-can required: pip install python-can")
        self.can = can
        self.node_id = node_id
        self.interface: str | None = None
        self.bus = None
        self._seq = 0
        self._lock = threading.Lock()
        self.latest: Telemetry | None = None
        self.latest_enc: EncTelem | None = None
        self.latest_cal: CalTelem | None = None
        self.enc_ratio = {"d_mech": 0.0, "d_elec": 0.0, "n": 0}
        self.enc_ratio_last: tuple | None = None
        self.motor_ok = False
        self.motor_error: str | None = None
        self.info: Info | None = None
        self._acks: Queue = Queue()
        self._replies: Queue = Queue(maxsize=8)
        self._infos: Queue = Queue()
        self._confs: Queue = Queue()
        self._snap_metas: Queue = Queue()
        self._snap_datas: Queue = Queue()
        self._rx_stop = threading.Event()
        self._rx_thread: threading.Thread | None = None
        self._stream_stop = threading.Event()
        self._stream_thread: threading.Thread | None = None
        self._stream_op = "query"
        self._stream_args: dict = {}
        self._stream_hz = 50.0
        self._telem_ring: deque[dict] = deque(maxlen=256)
        self._telem_sid = 0
        self._enc_comp_capture = False
        self._enc_comp_rows: list[tuple[float, float]] = []
        self._rx_count = 0
        self._rx_window_t0 = time.monotonic()
        self._rx_hz = 0.0
        self.msglog = MsgLog()
        self.last_snap: dict | None = None

    def _motor_snapshot(self) -> dict:
        with self._lock:
            info = self.info
            motor_ok = self.motor_ok
            motor_error = self.motor_error
        return {
            "motor_ok": motor_ok,
            "motor_error": motor_error,
            "info": info.as_dict() if info is not None else None,
        }

    def status(self) -> dict:
        return {
            "ok": True,
            "connected": self.bus is not None,
            "interface": self.interface,
            "node_id": self.node_id,
            "ports": scan_can_ports(),
            **self._motor_snapshot(),
        }

    def connect(self, interface: str) -> dict:
        interface = (interface or "").strip()
        if not interface:
            return {"ok": False, "error": "empty interface", "motor_ok": False}
        self.disconnect()
        # Avoid python-can stderr spam on missing/down iface.
        err_buf = StringIO()
        try:
            with redirect_stderr(err_buf):
                bus = self.can.Bus(
                    interface="socketcan", channel=interface, fd=True
                )
        except OSError as exc:
            self.msglog.push("ERR", f"CAN open {interface} failed: {exc}")
            return {
                "ok": False,
                "motor_ok": False,
                "error": str(exc),
                "hint": (
                    f"{interface} 可能未 UP。请点「打开 CAN」，或手动："
                    f"sudo ip link set {interface} type can "
                    "bitrate 1000000 dbitrate 2000000 fd on && "
                    f"sudo ip link set {interface} up"
                ),
                "ports": scan_can_ports(),
            }
        with self._lock:
            self.bus = bus
            self.interface = interface
            self.latest = None
            self.latest_enc = None
            self.latest_cal = None
            self.enc_ratio = {"d_mech": 0.0, "d_elec": 0.0, "n": 0}
            self.enc_ratio_last = None
            self._telem_ring.clear()
            self._telem_sid = 0
            self._rx_count = 0
            self._rx_window_t0 = time.monotonic()
            self._rx_hz = 0.0
            self.motor_ok = False
            self.motor_error = None
            self.info = None
        self._drain_queues()
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self.set_stream("query")
        self._start_stream_loop()
        self.msglog.push("SYS", f"CAN bus open: {interface} node={self.node_id}")

        probe = self.probe_motor(timeout=1.5)
        return {
            "ok": True,
            "interface": interface,
            "ports": scan_can_ports(),
            **probe,
        }

    def open_can(self, interface: str) -> dict:
        """Bring SocketCAN UP (ip link) then open bus + probe motor."""
        interface = (interface or "").strip()
        if not interface:
            return {"ok": False, "error": "empty interface", "motor_ok": False}
        self.msglog.push("SYS", f"打开 CAN: bring up {interface}")
        ok, err = bring_up_can(interface)
        if not ok:
            self.msglog.push("ERR", err)
            return {
                "ok": False,
                "motor_ok": False,
                "error": err,
                "ports": scan_can_ports(),
            }
        result = self.connect(interface)
        if result.get("ok"):
            self.msglog.push("SYS", f"打开 CAN 成功: {interface}")
        return result

    def close_can(self, interface: str = "") -> dict:
        """Close bus then bring SocketCAN DOWN."""
        iface = (interface or self.interface or "").strip()
        self.disconnect()
        if not iface:
            return {
                "ok": True,
                "connected": False,
                "motor_ok": False,
                "ports": scan_can_ports(),
            }
        self.msglog.push("SYS", f"关闭 CAN: bring down {iface}")
        ok, err = bring_down_can(iface)
        if not ok:
            self.msglog.push("ERR", err)
            return {
                "ok": False,
                "connected": False,
                "motor_ok": False,
                "error": err,
                "ports": scan_can_ports(),
            }
        self.msglog.push("SYS", f"关闭 CAN 成功: {iface}")
        return {
            "ok": True,
            "connected": False,
            "motor_ok": False,
            "interface": None,
            "ports": scan_can_ports(),
        }

    def disconnect(self) -> dict:
        self._stop_stream_loop()
        self._rx_stop.set()
        thread = self._rx_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._rx_thread = None
        with self._lock:
            bus = self.bus
            self.bus = None
            self.interface = None
            self.latest = None
            self.latest_enc = None
            self.latest_cal = None
            self._telem_ring.clear()
            self._telem_sid = 0
            self._rx_count = 0
            self._rx_window_t0 = time.monotonic()
            self._rx_hz = 0.0
            self.motor_ok = False
            self.motor_error = None
            self.info = None
        self._drain_queues()
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:  # noqa: BLE001
                pass
            self.msglog.push("SYS", "CAN disconnected")
        return {
            "ok": True,
            "connected": False,
            "motor_ok": False,
            "ports": scan_can_ports(),
        }

    def _drain_queues(self):
        for q in (
            self._acks,
            self._replies,
            self._infos,
            self._confs,
            self._snap_metas,
            self._snap_datas,
        ):
            while True:
                try:
                    q.get_nowait()
                except Empty:
                    break

    def _next_seq(self) -> int:
        with self._lock:
            self._seq = (self._seq + 1) & 0xFF
            return self._seq

    def _rx_loop(self):
        want = tel_id(self.node_id)
        while not self._rx_stop.is_set():
            with self._lock:
                bus = self.bus
            if bus is None:
                break
            try:
                frame = bus.recv(timeout=0.1)
            except Exception:  # noqa: BLE001
                break
            if frame is None or frame.arbitration_id != want:
                continue
            msg = parse_frame(bytes(frame.data))
            if isinstance(msg, CtrlReply):
                telem = msg.as_telemetry()
                enc = msg.as_enc()
                with self._lock:
                    self.latest = telem
                    self.latest_enc = enc
                    self._accum_enc_ratio(enc)
                    cal = self.latest_cal
                    sample = self._live_dict_locked(
                        telem, enc, cal,
                        omega_cmd=msg.omega_cmd_rad_s,
                        omega_elec=msg.omega_elec_rad_s,
                        voltage_headroom=msg.voltage_headroom_v,
                    )
                    sample["enc_spike"] = int(msg.enc_spike)
                    self._push_live_sample(sample)
                    if self._enc_comp_capture:
                        frac = (float(msg.enc_raw) / 65536.0) % 1.0
                        self._enc_comp_rows.append(
                            (frac, float(msg.omega_mech_rad_s))
                        )
                self.msglog.maybe_ctrl(msg)
                try:
                    self._replies.put_nowait(msg)
                except Exception:  # noqa: BLE001
                    try:
                        self._replies.get_nowait()
                    except Empty:
                        pass
                    try:
                        self._replies.put_nowait(msg)
                    except Exception:  # noqa: BLE001
                        pass
            elif isinstance(msg, Telemetry):
                with self._lock:
                    self.latest = msg
                    sample = self._live_dict_locked(msg, self.latest_enc, self.latest_cal)
                    self._push_live_sample(sample)
                self.msglog.maybe_telem(msg)
            elif isinstance(msg, EncTelem):
                with self._lock:
                    self.latest_enc = msg
            elif isinstance(msg, CalTelem):
                with self._lock:
                    self.latest_cal = msg
            elif isinstance(msg, Ack):
                name = _CMD_NAMES.get(msg.cmd, str(msg.cmd))
                self.msglog.push(
                    "RX",
                    f"ACK cmd={name} status={msg.status} seq={msg.seq}",
                )
                self._acks.put(msg)
            elif isinstance(msg, Info):
                self.msglog.push(
                    "RX",
                    f"INFO {info_motor_line(msg)}",
                )
                self._infos.put(msg)
            elif isinstance(msg, ConfReply):
                self.msglog.push(
                    "RX",
                    (
                        f"CONF op={msg.op} group={msg.group_name} "
                        f"flash_valid={int(msg.flash_valid)} "
                        f"fields={msg.fields}"
                    ),
                )
                self._confs.put(msg)
            elif isinstance(msg, SnapMeta):
                self.msglog.push(
                    "RX",
                    (
                        f"SNAP meta n={msg.n_samples} hz={msg.sample_hz} "
                        f"dec={msg.decimate} dur={msg.duration_us}us"
                    ),
                )
                self._snap_metas.put(msg)
            elif isinstance(msg, SnapData):
                # TEMP debug: first snap frame samples layout check
                if getattr(self, "_snap_dump_done", False) is False:
                    self.msglog.push(
                        "SYS",
                        f"SNAPDUMP n={msg.n} samples={list(msg.samples_mA[:14])}",
                    )
                    self._snap_dump_done = True
                self._snap_datas.put(msg)

    def _send_raw(self, payload: bytes, log: bool = True):
        with self._lock:
            bus = self.bus
        if bus is None:
            raise RuntimeError("CAN not connected — pick a port and Connect")
        if log and len(payload) >= 5:
            cmd = payload[4]
            name = _CMD_NAMES.get(cmd, str(cmd))
            self.msglog.push(
                "TX",
                f"CMD {name} seq={payload[3]} len={len(payload)} hex={payload.hex()}",
            )
        bus.send(
            self.can.Message(
                arbitration_id=cmd_id(self.node_id),
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=True,
                data=payload,
            )
        )


    def set_stream(self, op: str, **args):
        with self._lock:
            self._stream_op = op
            self._stream_args = dict(args)

    def _start_stream_loop(self):
        self._stop_stream_loop()
        self._stream_stop.clear()
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()

    def _stop_stream_loop(self):
        self._stream_stop.set()
        th = self._stream_thread
        self._stream_thread = None
        if th is not None and th.is_alive():
            th.join(timeout=1.0)

    def _pack_stream_frame(self, op: str, args: dict, seq: int) -> bytes:
        if op == "query":
            return pack_query(seq)
        if op == "stop":
            return pack_stop(seq)
        if op == "servo":
            def opt_float(key):
                if key not in args or args.get(key) is None:
                    return None
                return float(args.get(key))
            return pack_servo(
                float(args.get("omega_mech", 0)),
                float(args.get("id", 0)),
                seq,
                position_rad=opt_float("position"),
                stop_position_rad=opt_float("stop_position"),
                max_torque_nm=opt_float("max_torque"),
                feedforward_nm=float(args.get("feedforward", 0) or 0),
                velocity_limit_rad_s=opt_float("velocity_limit"),
                accel_limit_rad_s2=opt_float("accel_limit"),
                kp_scale=float(args.get("kp_scale", 1.0) or 1.0),
                kd_scale=float(args.get("kd_scale", 1.0) or 1.0),
                ilimit_scale=float(args.get("ilimit_scale", 1.0) or 1.0),
            )
        if op == "current":
            return pack_current(
                float(args.get("id", 0)),
                float(args.get("iq", 0)),
                seq,
            )
        if op == "mit":
            max_t = args.get("max_torque")
            return pack_mit(
                float(args.get("position", 0)),
                float(args.get("velocity", 0)),
                float(args.get("kp", 0)),
                float(args.get("kd", 0)),
                float(args.get("feedforward", 0) or 0),
                None if max_t is None else float(max_t),
                seq,
            )
        return pack_query(seq)

    def _stream_loop(self):
        period = 1.0 / max(1.0, float(self._stream_hz))
        next_t = time.monotonic()
        while not self._stream_stop.is_set():
            with self._lock:
                bus = self.bus
                op = self._stream_op
                args = dict(self._stream_args)
            if bus is not None and op:
                try:
                    seq = self._next_seq()
                    payload = self._pack_stream_frame(op, args, seq)
                    # Avoid console spam at 50 Hz.
                    self._send_raw(payload, log=False)
                    if op == "stop":
                        # One stop is enough; fall back to query keep-alive.
                        self.set_stream("query")
                except Exception as exc:  # noqa: BLE001
                    self.msglog.push("ERR", f"stream TX failed: {exc}")
                    time.sleep(0.05)
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                # Wake early if stopped.
                self._stream_stop.wait(timeout=delay)
            else:
                next_t = time.monotonic()

    def send_ctrl(self, payload: bytes, timeout: float = 1.0) -> CtrlReply:
        """Send a CtrlReply-style command and wait for matching reply."""
        self._drain_queues()
        seq = payload[3] if len(payload) > 3 else -1
        self._send_raw(payload)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                msg = self._replies.get(timeout=0.05)
            except Empty:
                continue
            if seq < 0 or msg.seq == seq:
                return msg
        self.msglog.push("ERR", "timeout waiting for CtrlReply")
        raise TimeoutError("no CtrlReply")

    def send_cmd(self, payload: bytes, timeout: float = 1.0) -> Ack:
        self._drain_queues()
        self._send_raw(payload)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                return self._acks.get(timeout=0.05)
            except Empty:
                continue
        self.msglog.push("ERR", "timeout waiting for ACK")
        raise TimeoutError("no ACK")

    def probe_motor(self, timeout: float = 1.5) -> dict:
        """Send kCmdInfo; motor replies with Info then ACK."""
        if self.bus is None:
            return {
                "motor_ok": False,
                "motor_error": "CAN not connected",
                "info": None,
            }
        self._drain_queues()
        try:
            self._send_raw(pack_info(self._next_seq()))
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self.motor_ok = False
                self.motor_error = str(exc)
                self.info = None
            return {
                "motor_ok": False,
                "motor_error": str(exc),
                "info": None,
            }

        info: Info | None = None
        ack: Ack | None = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if info is None:
                try:
                    info = self._infos.get(timeout=0.05)
                except Empty:
                    pass
            if ack is None:
                try:
                    ack = self._acks.get(timeout=0.05)
                except Empty:
                    pass
            if info is not None and ack is not None:
                break

        if info is None or ack is None:
            err = "电机连接失败：超时无 Info/ACK（检查节点、线、固件是否含 info）"
            self.msglog.push("ERR", err)
            with self._lock:
                self.motor_ok = False
                self.motor_error = err
                self.info = None
            return {"motor_ok": False, "motor_error": err, "info": None}

        if ack.cmd != CMD_INFO or ack.status != STATUS_OK:
            err = f"电机连接失败：ACK cmd={ack.cmd} status={ack.status}"
            self.msglog.push("ERR", err)
            with self._lock:
                self.motor_ok = False
                self.motor_error = err
                self.info = None
            return {"motor_ok": False, "motor_error": err, "info": None}

        with self._lock:
            self.motor_ok = True
            self.motor_error = None
            self.info = info
        self.msglog.push("SYS", f"电机连接成功: {info_motor_line(info)}")
        return {
            "motor_ok": True,
            "motor_error": None,
            "info": info.as_dict(),
        }

    def clear_encoder_comp(self) -> dict:
        """Disable/clear moteus-style encoder geometric compensation."""
        if self.bus is None:
            return {"ok": False, "error": "CAN not connected"}
        with self._lock:
            saved_op = self._stream_op
            saved_args = dict(self._stream_args)
            self._stream_op = ""
        try:
            self._drain_queues()
            ack = self.send_cmd(pack_enc_comp_clear(self._next_seq()))
            time.sleep(0.2)
            ok = int(getattr(ack, "status", 1)) == STATUS_OK
            self.msglog.push("SYS", f"enc_comp clear status={getattr(ack, 'status', -1)}")
            return {
                "ok": ok,
                "status": int(getattr(ack, "status", -1)),
                "action": "clear",
            }
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc), "action": "clear"}
        finally:
            with self._lock:
                self._stream_op = saved_op
                self._stream_args = saved_args

    def run_encoder_comp(
        self,
        omega_mech: float = 40.0,
        seconds: float = 8.0,
        rate_hz: float = 200.0,
        from_snap: bool = False,
    ) -> dict:
        """Inertial encoder geometric compensation (256-point table)."""
        if self.bus is None:
            return {"ok": False, "error": "CAN not connected"}

        source = "snap" if from_snap else "live"
        result = None
        with self._lock:
            saved_op = self._stream_op
            saved_args = dict(self._stream_args)
            self._stream_op = ""
        try:
            if from_snap:
                snap = self.last_snap
                if not snap:
                    return {"ok": False, "error": "no snapshot yet — 先 Capture 一次"}
                rows = _enc_comp.samples_from_snap_dict(snap)
                result = _enc_comp.build_table(rows)
            else:
                omega_mech = float(omega_mech)
                seconds = max(1.0, float(seconds))
                rate_hz = max(20.0, float(rate_hz))
                if abs(omega_mech) < 5.0:
                    return {
                        "ok": False,
                        "error": "ω 太小（建议 ≥20，推荐 40~200）",
                    }
                self._drain_queues()
                with self._lock:
                    self._enc_comp_rows = []
                    self._enc_comp_capture = True
                period = 1.0 / rate_hz
                t0 = time.monotonic()
                next_t = t0
                try:
                    while time.monotonic() - t0 < seconds:
                        seq = self._next_seq()
                        self._send_raw(pack_servo(omega_mech, 0.0, seq), log=False)
                        next_t += period
                        delay = next_t - time.monotonic()
                        if delay > 0:
                            time.sleep(delay)
                        else:
                            next_t = time.monotonic()
                finally:
                    with self._lock:
                        self._enc_comp_capture = False
                        rows = list(self._enc_comp_rows)
                        self._enc_comp_rows = []
                try:
                    self.send_cmd(pack_stop(self._next_seq()), timeout=0.5)
                except Exception:
                    try:
                        self._send_raw(pack_stop(self._next_seq()), log=False)
                    except Exception:
                        pass
                if len(rows) < 256:
                    return {
                        "ok": False,
                        "error": f"样本不足 ({len(rows)} < 256)，加长秒数或提高 rate",
                        "n_samples": len(rows),
                    }
                result = _enc_comp.build_table(rows)

            self._drain_queues()

            def _send(payload: bytes):
                return self.send_cmd(payload, timeout=1.0)

            _enc_comp.upload_with_sender(
                _send,
                result["table"],
                result["peak_rad"],
                clear_first=True,
                next_seq=self._next_seq,
            )
            self.msglog.push(
                "SYS",
                f"enc_comp ok source={source} peak={result['peak_rad']:.4f}rad "
                f"dev_std={100 * result['vel_dev_std']:.2f}%",
            )
            return {
                "ok": True,
                "action": "commit",
                "source": source,
                "mean_omega": result["mean_omega"],
                "vel_dev_std": result["vel_dev_std"],
                "peak_rad": result["peak_rad"],
                "peak_deg": result["peak_rad"] * 180.0 / math.pi,
                "filled_bins": result["filled_bins"],
                "n_samples": result["n_samples"],
            }
        except Exception as exc:  # noqa: BLE001
            try:
                self._send_raw(pack_stop(self._next_seq()), log=False)
            except Exception:
                pass
            return {"ok": False, "error": str(exc), "action": "commit"}
        finally:
            with self._lock:
                self._stream_op = saved_op
                self._stream_args = saved_args

    def capture_snap(
        self,
        n_samples: int = SNAP_MAX_SAMPLES,
        decimate: int = 1,
        timeout: float = 12.0,
    ) -> dict:
        if self.bus is None:
            return {"ok": False, "error": "CAN not connected"}
        # Pause Live stream so 50 Hz vel/query TX does not crowd SnapData off
        # the bus / gs_usb RX path while the dump is in flight.
        with self._lock:
            saved_op = self._stream_op
            saved_args = dict(self._stream_args)
            self._stream_op = ""
        try:
            self._drain_queues()
            try:
                self._send_raw(
                    pack_snap(
                        n_samples=n_samples, decimate=decimate, seq=self._next_seq()
                    )
                )
            except Exception as exc:  # noqa: BLE001
                return {"ok": False, "error": str(exc)}

            deadline = time.monotonic() + timeout
            ack: Ack | None = None
            while time.monotonic() < deadline and ack is None:
                try:
                    ack = self._acks.get(timeout=0.05)
                except Empty:
                    pass
            if ack is None:
                err = "snapshot ACK timeout"
                self.msglog.push("ERR", err)
                return {"ok": False, "error": err}
            if ack.cmd != CMD_SNAP or ack.status != STATUS_OK:
                err = (
                    "snapshot rejected: "
                    f"status={ack.status}"
                    + (" (start servo/cal first)" if ack.status == STATUS_NOT_RUN else "")
                )
                self.msglog.push("ERR", err)
                return {"ok": False, "error": err, "status": ack.status}

            meta: SnapMeta | None = None
            # Capture itself may take n/hz seconds; keep remaining timeout for dump.
            while time.monotonic() < deadline and meta is None:
                try:
                    meta = self._snap_metas.get(timeout=0.05)
                except Empty:
                    pass
            if meta is None:
                err = "snapshot meta timeout (is control ISR running?)"
                self.msglog.push("ERR", err)
                return {"ok": False, "error": err}

            # Give the dump its own budget after Meta (128 FD frames for 512 pts).
            dump_deadline = time.monotonic() + max(3.0, timeout * 0.75)
            if dump_deadline > deadline:
                deadline = dump_deadline

            ch = meta.channels or SNAP_CHANNELS
            n = meta.n_samples
            rows = [[0.0] * ch for _ in range(n)]
            got = 0
            while time.monotonic() < deadline and got < n:
                try:
                    frame = self._snap_datas.get(timeout=0.05)
                except Empty:
                    continue
                for i in range(frame.n):
                    idx = frame.start_index + i
                    if idx >= n:
                        continue
                    base = i * ch
                    for c in range(ch):
                        rows[idx][c] = frame.samples_mA[base + c] / 1000.0
                    got += 1

            if got < n:
                err = f"snapshot incomplete got={got}/{n}"
                self.msglog.push("ERR", err)
                return {"ok": False, "error": err, "meta": {
                    "n_samples": n,
                    "sample_hz": meta.sample_hz,
                    "decimate": meta.decimate,
                    "duration_us": meta.duration_us,
                }}

            series = {k: [] for k in SNAP_CHANNEL_KEYS[:ch]}
            keys = SNAP_CHANNEL_KEYS[:ch]
            for row in rows:
                for c, k in enumerate(keys):
                    series[k].append(row[c])

            result = {
                "ok": True,
                "n_samples": n,
                "sample_hz": meta.sample_hz,
                "decimate": meta.decimate,
                "duration_us": meta.duration_us,
                "channels": keys,
                "series": series,
            }
            self.last_snap = result
            self.msglog.push(
                "SYS",
                f"snapshot ok n={n} hz={meta.sample_hz} dur={meta.duration_us}us",
            )
            return result
        finally:
            with self._lock:
                self._stream_op = saved_op
                self._stream_args = saved_args


    def _accum_enc_ratio(self, enc: EncTelem | None):
        """Accumulate unwrapped mech/elec angle deltas to measure the true
        electrical-periods-per-mechanical-revolution ratio (= pole pairs)."""
        if enc is None or not enc.ok:
            return
        m, e = enc.theta_mech_rad, enc.theta_elec_rad
        last = self.enc_ratio_last
        if last is not None:
            dm = m - last[0]
            de = e - last[1]
            dm = ((dm + math.pi) % (2.0 * math.pi)) - math.pi
            de = ((de + math.pi) % (2.0 * math.pi)) - math.pi
            self.enc_ratio["d_mech"] += dm
            self.enc_ratio["d_elec"] += de
            self.enc_ratio["n"] += 1
        self.enc_ratio_last = (m, e)

    def _live_dict_locked(
        self, t: Telemetry, enc: EncTelem | None, cal: CalTelem | None,
        omega_cmd: float | None = None, omega_elec: float = 0.0,
        voltage_headroom: float = 0.0,
    ) -> dict:
        enc_fields = {
            "enc_raw": enc.raw if enc else 0,
            "enc_theta_mech_rad": enc.theta_mech_rad if enc else 0.0,
            "enc_theta_elec_rad": enc.theta_elec_rad if enc else 0.0,
            "enc_sign": enc.sign if enc else 1,
            "enc_ok": bool(enc.ok) if enc else bool(getattr(t, "enc_ok", False)),
            "enc_mode": bool(getattr(t, "enc_mode", False)),
            "enc_spike": int(getattr(enc, "enc_spike", 0) or 0),
        }
        cal_fields = {
            "cal_kind": cal.kind if cal else 0,
            "cal_state": cal.state if cal else 0,
            "cal_state_name": _CAL_STATE_NAMES.get(cal.state if cal else 0, "idle"),
            "cal_progress": cal.progress if cal else 0.0,
            "cal_offset_rad": cal.offset_rad if cal else 0.0,
            "cal_residual_rad": cal.residual_rad if cal else 0.0,
            "cal_sign": cal.sign if cal else 1,
            "cal_ok": bool(cal.ok) if cal else False,
            "cal_samples": cal.sample_count if cal else 0,
            "cal_persisted": bool(cal.persisted) if cal else False,
        }
        out = {
            "seq": t.seq,
            "flags": t.flags,
            "id_a": t.id_a,
            "iq_a": t.iq_a,
            "idref_a": t.idref_a,
            "iqref_a": t.iqref_a,
            "i1_a": t.i1_a,
            "i2_a": t.i2_a,
            "i3_a": t.i3_a,
            "theta_rad": t.theta_rad,
            "omega_rad_s": t.omega_rad_s,
            "omega_cmd_rad_s": float(omega_cmd) if omega_cmd is not None else t.omega_rad_s,
            "omega_elec_rad_s": float(omega_elec),
            "voltage_headroom_v": float(voltage_headroom),
            "vd_v": t.vd_v,
            "vq_v": t.vq_v,
            "duty_a": t.duty_a,
            "duty_b": t.duty_b,
            "duty_c": t.duty_c,
            "bus_v": t.bus_v,
            "mode": t.mode,
            "pwm_on": t.pwm_on,
            "cisr": t.cisr,
            **enc_fields,
            **cal_fields,
        }
        return out

    def _push_live_sample(self, sample: dict):
        self._telem_sid += 1
        sample = dict(sample)
        sample["_sid"] = self._telem_sid
        sample["_t_mono"] = time.monotonic()
        self._telem_ring.append(sample)
        self._rx_count += 1
        now = time.monotonic()
        dt = now - self._rx_window_t0
        if dt >= 0.5:
            self._rx_hz = self._rx_count / dt
            self._rx_count = 0
            self._rx_window_t0 = now


    _CONF_SINGLE_GROUPS = (
        CONF_GROUP_MOTOR,
        CONF_GROUP_FOC,
        CONF_GROUP_SERVO,
        CONF_GROUP_ENCODER,
        CONF_GROUP_CAL,
    )

    def _pause_stream(self):
        with self._lock:
            saved_op = self._stream_op
            saved_args = dict(self._stream_args)
            self._stream_op = ""
        return saved_op, saved_args

    def _resume_stream(self, saved_op, saved_args):
        with self._lock:
            self._stream_op = saved_op
            self._stream_args = saved_args

    def _group_name(self, group_i: int) -> str:
        for name, gid in CONF_GROUP_BY_NAME.items():
            if gid == group_i:
                return name
        return str(group_i)

    def _wait_conf_ack(
        self,
        seq: int,
        timeout: float = 1.5,
        want_reply: bool = False,
        group: int | None = None,
    ) -> tuple[Ack, ConfReply | None]:
        ack: Ack | None = None
        reply: ConfReply | None = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if reply is None:
                try:
                    cand = self._confs.get(timeout=0.05)
                except Empty:
                    cand = None
                if cand is not None and (
                    group is None
                    or cand.group == group
                    or cand.group == CONF_GROUP_ALL
                ):
                    reply = cand
            if ack is None:
                try:
                    cand_ack = self._acks.get(timeout=0.05)
                except Empty:
                    cand_ack = None
                if cand_ack is not None and (
                    seq < 0 or cand_ack.seq == seq or cand_ack.cmd == CMD_CONF
                ):
                    ack = cand_ack
            if ack is not None and (reply is not None or not want_reply):
                break
        if ack is None:
            raise TimeoutError("no ACK for conf")
        if want_reply and reply is None:
            raise TimeoutError("no ConfReply")
        return ack, reply

    def conf_get(self, group: str | int = "all") -> dict:
        if self.bus is None:
            return {"ok": False, "error": "CAN not connected"}
        group_i = conf_group_id(group)
        saved = self._pause_stream()
        try:
            self._drain_queues()
            if group_i == CONF_GROUP_ALL:
                fields = {}
                flash_valid = False
                for g in self._CONF_SINGLE_GROUPS:
                    seq = self._next_seq()
                    self._send_raw(pack_conf("get", g, seq=seq))
                    ack, reply = self._wait_conf_ack(
                        seq, want_reply=True, group=g
                    )
                    if ack.status != STATUS_OK:
                        return {
                            "ok": False,
                            "error": f"conf get status={ack.status}",
                            "status": ack.status,
                            "group": "all",
                        }
                    assert reply is not None
                    fields[reply.group_name] = reply.fields
                    flash_valid = flash_valid or reply.flash_valid
                return {
                    "ok": True,
                    "op": "get",
                    "group": "all",
                    "flash_valid": flash_valid,
                    "fields": fields,
                }
            seq = self._next_seq()
            self._send_raw(pack_conf("get", group_i, seq=seq))
            ack, reply = self._wait_conf_ack(
                seq, want_reply=True, group=group_i
            )
            if ack.status != STATUS_OK:
                return {
                    "ok": False,
                    "error": f"conf get status={ack.status}",
                    "status": ack.status,
                    "group": self._group_name(group_i),
                }
            assert reply is not None
            return {
                "ok": True,
                "op": "get",
                "group": reply.group_name,
                "flash_valid": reply.flash_valid,
                "fields": reply.fields,
                "status": ack.status,
            }
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc)}
        finally:
            self._resume_stream(*saved)

    def conf_set(self, group: str | int, fields: dict | None = None) -> dict:
        if self.bus is None:
            return {"ok": False, "error": "CAN not connected"}
        group_i = conf_group_id(group)
        if group_i == CONF_GROUP_ALL:
            return {"ok": False, "error": "conf set requires a concrete group"}
        if group_i == CONF_GROUP_CAL:
            return {"ok": False, "error": "cal status is read-only"}
        saved = self._pause_stream()
        try:
            self._drain_queues()
            seq = self._next_seq()
            self._send_raw(
                pack_conf("set", group_i, fields=fields or {}, seq=seq)
            )
            ack, reply = self._wait_conf_ack(
                seq, want_reply=False, group=group_i
            )
            if ack.status != STATUS_OK:
                return {
                    "ok": False,
                    "error": f"conf set status={ack.status}",
                    "status": ack.status,
                    "group": self._group_name(group_i),
                }
            out = {
                "ok": True,
                "op": "set",
                "group": (
                    reply.group_name
                    if reply is not None
                    else self._group_name(group_i)
                ),
                "status": ack.status,
                "fields": (
                    reply.fields if reply is not None else dict(fields or {})
                ),
            }
            if reply is not None:
                out["flash_valid"] = reply.flash_valid
            return out
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc)}
        finally:
            self._resume_stream(*saved)

    def _conf_simple(self, op: str, group: str | int = "all") -> dict:
        if self.bus is None:
            return {"ok": False, "error": "CAN not connected"}
        group_i = conf_group_id(group)
        saved = self._pause_stream()
        try:
            self._drain_queues()
            seq = self._next_seq()
            self._send_raw(pack_conf(op, group_i, seq=seq))
            ack, reply = self._wait_conf_ack(seq, want_reply=False)
            if ack.status != STATUS_OK:
                err = f"conf {op} status={ack.status}"
                if op == "load" and ack.status == STATUS_FAIL:
                    err = (
                        "conf load failed: flash has no saved runtime config "
                        "(Apply groups, then Save first)"
                    )
                return {
                    "ok": False,
                    "error": err,
                    "status": ack.status,
                    "op": op,
                    "group": self._group_name(group_i),
                }
            out = {
                "ok": True,
                "op": op,
                "group": self._group_name(group_i),
                "status": ack.status,
            }
            if reply is not None:
                out["flash_valid"] = reply.flash_valid
                if reply.fields:
                    out["fields"] = reply.fields
                    out["group"] = reply.group_name
            return out
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc), "op": op}
        finally:
            self._resume_stream(*saved)

    def conf_save(self, group: str | int = "all") -> dict:
        return self._conf_simple("save", group)

    def conf_load(self, group: str | int = "all") -> dict:
        return self._conf_simple("load", group)

    def conf_defaults(self, group: str | int = "all") -> dict:
        return self._conf_simple("defaults", group)

    def telem_dict(self, after_sid: int = 0):
        motor = self._motor_snapshot()
        with self._lock:
            connected = self.bus is not None
            interface = self.interface
            t = self.latest
            enc = self.latest_enc
            cal = self.latest_cal
            rx_hz = self._rx_hz
            sid = self._telem_sid
            if after_sid > 0:
                samples = [s for s in self._telem_ring if s.get("_sid", 0) > after_sid]
            else:
                # First paint: only latest sample (avoid dumping whole ring).
                samples = [self._telem_ring[-1]] if self._telem_ring else []
        if not connected:
            return {
                "ok": False,
                "connected": False,
                "error": "not connected",
                "ports": scan_can_ports(),
                "rx_hz": 0.0,
                "sid": 0,
                "samples": [],
                **motor,
            }
        if t is None:
            return {
                "ok": False,
                "connected": True,
                "interface": interface,
                "error": "no telemetry yet",
                "rx_hz": rx_hz,
                "sid": sid,
                "samples": [],
                **motor,
            }
        latest = self._live_dict_locked(t, enc, cal)
        # Prefer last ring sample (has omega_cmd) when present.
        if samples:
            latest = {**latest, **{k: v for k, v in samples[-1].items() if not k.startswith('_')}}
        return {
            "ok": True,
            "connected": True,
            "interface": interface,
            "rx_hz": rx_hz,
            "sid": sid,
            "samples": samples,
            **latest,
            **motor,
        }

    def close(self):
        self.disconnect()


def info_motor_line(info: Info) -> str:
    # Keep logs short; identity/limits belong in Config/Flash.
    return f"fw={info.fw_version}"


def make_handler(bridge: CanBridge):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _json(self, code: int, obj):
            body = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _file(self, path: Path, content_type: str):
            data = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self):
            n = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(n)
            return json.loads(raw.decode("utf-8") if raw else "{}")

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            query = ""
            if "?" in self.path:
                query = self.path.split("?", 1)[1]
            if path.startswith("/api/telem"):
                from urllib.parse import urlparse, parse_qs
                qs = parse_qs(urlparse(self.path).query)
                after = int((qs.get("after") or ["0"])[0] or 0)
                self._json(200, bridge.telem_dict(after_sid=after))
                return
            if path.startswith("/api/messages"):
                after = 0
                for part in query.split("&"):
                    if part.startswith("after="):
                        try:
                            after = int(part.split("=", 1)[1])
                        except ValueError:
                            after = 0
                self._json(200, {"ok": True, "messages": bridge.msglog.since(after)})
                return
            if path.startswith("/api/ports") or path.startswith("/api/status"):
                self._json(200, bridge.status())
                return
            if path in ("/", "/index.html"):
                self._file(STATIC_DIR / "index.html", "text/html; charset=utf-8")
                return
            if path.startswith("/static/"):
                name = path[len("/static/") :]
                if "/" in name or name.startswith(".") or ".." in name:
                    self.send_error(404)
                    return
                file_path = STATIC_DIR / name
                if not file_path.is_file():
                    self.send_error(404)
                    return
                ctype = (
                    mimetypes.guess_type(str(file_path))[0]
                    or "application/octet-stream"
                )
                self._file(file_path, ctype)
                return
            self.send_error(404)

        def do_POST(self):
            path = self.path.split("?", 1)[0]
            try:
                req = self._read_json()
            except json.JSONDecodeError:
                self._json(400, {"ok": False, "error": "bad json"})
                return

            if path == "/api/connect":
                # Legacy: bus-only. Prefer /api/can/open.
                self._json(200, bridge.connect(str(req.get("interface", ""))))
                return
            if path == "/api/can/open":
                self._json(200, bridge.open_can(str(req.get("interface", ""))))
                return
            if path == "/api/can/close":
                self._json(
                    200,
                    bridge.close_can(str(req.get("interface", "") or "")),
                )
                return
            if path == "/api/disconnect":
                self._json(200, bridge.disconnect())
                return
            if path == "/api/probe":
                try:
                    self._json(200, bridge.probe_motor())
                except Exception as exc:  # noqa: BLE001
                    self._json(
                        500,
                        {"ok": False, "motor_ok": False, "error": str(exc)},
                    )
                return
            if path == "/api/snap":
                try:
                    self._json(
                        200,
                        bridge.capture_snap(
                            n_samples=int(req.get("n_samples", SNAP_MAX_SAMPLES)),
                            decimate=int(req.get("decimate", 1)),
                        ),
                    )
                except Exception as exc:  # noqa: BLE001
                    self._json(500, {"ok": False, "error": str(exc)})
                return
            if path == "/api/snap/export":
                try:
                    snap = bridge.last_snap
                    if not snap:
                        self._json(200, {"ok": False, "error": "no snapshot yet"})
                        return
                    w = float(req.get("omega_mech", 0) or 0)
                    name = str(req.get("filename") or "").strip()
                    if not name:
                        name = f"snap_{int(w)}.json" if w > 0 else "snap_last.json"
                    # Keep writes inside the host tools root.
                    out_dir = HOST_ROOT
                    out_path = (out_dir / Path(name).name).resolve()
                    if out_dir not in out_path.parents and out_path.parent != out_dir:
                        self._json(400, {"ok": False, "error": "bad filename"})
                        return
                    payload = dict(snap)
                    payload["omega_mech_rad_s"] = w
                    out_path.write_text(json.dumps(payload, indent=2))
                    self._json(
                        200,
                        {
                            "ok": True,
                            "path": str(out_path),
                            "n_samples": payload.get("n_samples"),
                            "sample_hz": payload.get("sample_hz"),
                        },
                    )
                except Exception as exc:  # noqa: BLE001
                    self._json(500, {"ok": False, "error": str(exc)})
                return
            if path == "/api/snap/analyze":
                try:
                    snap = bridge.last_snap
                    if not snap:
                        self._json(200, {"ok": False, "error": "no snapshot yet"})
                        return
                    pp = float(req.get("pole_pairs", 14))
                    w = float(req.get("omega_mech", 0) or 0)
                    if w <= 0.0 and getattr(bridge, "latest", None) is not None:
                        w = float(getattr(bridge.latest, "omega_rad_s", 0) or 0)
                    an = _snap_an.analyze(snap, pole_pairs=pp, omega_mech_rad_s=w)
                    an["omega_mech_used"] = w
                    self._json(200, {"ok": True, **an})
                except Exception as exc:  # noqa: BLE001
                    self._json(500, {"ok": False, "error": str(exc)})
                return
            if path == "/api/enc/ratio":
                try:
                    with bridge._lock:
                        r = dict(bridge.enc_ratio)
                    ratio = (r["d_elec"] / r["d_mech"]) if abs(r["d_mech"]) > 1e-3 else 0.0
                    self._json(200, {
                        "ok": True,
                        "d_mech_rad": round(r["d_mech"], 3),
                        "d_elec_rad": round(r["d_elec"], 3),
                        "pp_ratio": round(ratio, 3),
                        "samples": r["n"],
                        "expect": 14,
                        "note": "ratio = 电角度/机械角度；≈14 说明极对数/角度换算正确，>1.3x 或 <0.7x 则是换算 bug",
                    })
                except Exception as exc:  # noqa: BLE001
                    self._json(500, {"ok": False, "error": str(exc)})
                return
            if path == "/api/messages/clear":
                bridge.msglog.clear()
                self._json(200, {"ok": True})
                return
            if path == "/api/enc_comp":
                try:
                    action = str(req.get("action") or "run").strip()
                    if action == "clear":
                        self._json(200, bridge.clear_encoder_comp())
                    elif action in ("run", "live"):
                        self._json(
                            200,
                            bridge.run_encoder_comp(
                                omega_mech=float(req.get("omega_mech", 40)),
                                seconds=float(req.get("seconds", 8)),
                                rate_hz=float(req.get("rate", 200)),
                                from_snap=False,
                            ),
                        )
                    elif action in ("from_snap", "snap"):
                        self._json(
                            200,
                            bridge.run_encoder_comp(from_snap=True),
                        )
                    else:
                        self._json(400, {"ok": False, "error": f"bad action {action}"})
                except Exception as exc:  # noqa: BLE001
                    self._json(500, {"ok": False, "error": str(exc)})
                return
            if path == "/api/conf":
                try:
                    op = str(req.get("op") or "").strip().lower()
                    group = req.get("group", "all")
                    fields = req.get("fields")
                    if fields is not None and not isinstance(fields, dict):
                        self._json(
                            400, {"ok": False, "error": "fields must be object"}
                        )
                        return
                    if op == "get":
                        self._json(200, bridge.conf_get(group))
                    elif op == "set":
                        self._json(
                            200, bridge.conf_set(group, fields or {})
                        )
                    elif op == "save":
                        self._json(200, bridge.conf_save(group))
                    elif op == "load":
                        self._json(200, bridge.conf_load(group))
                    elif op == "defaults":
                        self._json(200, bridge.conf_defaults(group))
                    else:
                        self._json(
                            400, {"ok": False, "error": f"unknown op {op}"}
                        )
                except Exception as exc:  # noqa: BLE001
                    self._json(500, {"ok": False, "error": str(exc)})
                return
            if path != "/api/cmd":
                self.send_error(404)
                return

            op = req.get("op")
            try:
                # Streamed control: host keeps sending; board replies CtrlReply.
                if op == "stop":
                    bridge.set_stream("stop")
                    self._json(200, {"ok": True, "status": 0, "stream": "stop"})
                    return
                if op == "servo":
                    args = {
                        "omega_mech": float(req.get("omega_mech", 0)),
                        "id": float(req.get("id", 0)),
                        "feedforward": float(req.get("feedforward", 0) or 0),
                        "kp_scale": float(req.get("kp_scale", 1.0) or 1.0),
                        "kd_scale": float(req.get("kd_scale", 1.0) or 1.0),
                        "ilimit_scale": float(req.get("ilimit_scale", 1.0) or 1.0),
                    }
                    for key in (
                        "position", "stop_position", "max_torque",
                        "velocity_limit", "accel_limit",
                    ):
                        if key in req and req.get(key) is not None and str(req.get(key)) != "":
                            args[key] = float(req.get(key))
                    bridge.set_stream("servo", **args)
                    self._json(200, {"ok": True, "status": 0, "stream": "servo"})
                    return
                if op == "current":
                    bridge.set_stream(
                        "current",
                        id=float(req.get("id", 0)),
                        iq=float(req.get("iq", 0)),
                    )
                    self._json(200, {"ok": True, "status": 0, "stream": "current"})
                    return
                if op == "mit":
                    args = {
                        "position": float(req.get("position", 0)),
                        "velocity": float(req.get("velocity", 0)),
                        "kp": float(req.get("kp", 0)),
                        "kd": float(req.get("kd", 0)),
                        "feedforward": float(req.get("feedforward", 0) or 0),
                    }
                    if "max_torque" in req and req.get("max_torque") is not None and str(req.get("max_torque")) != "":
                        args["max_torque"] = float(req.get("max_torque"))
                    bridge.set_stream("mit", **args)
                    self._json(200, {"ok": True, "status": 0, "stream": "mit"})
                    return
                if op == "cal_enc":
                    bridge.set_stream("query")
                    payload = pack_cal_enc(
                        float(req.get("current", 1.0)),
                        float(req.get("omega_elec", 40)),
                        bridge._next_seq(),
                    )
                elif op == "cal_lock":
                    bridge.set_stream("query")
                    payload = pack_cal_lock(
                        float(req.get("current", 1.0)),
                        bridge._next_seq(),
                    )
                elif op == "cal_bemf":
                    bridge.set_stream("query")
                    payload = pack_cal_bemf(
                        float(req.get("max_speed", 60.0)),
                        int(req.get("n_points", 0)),
                        bridge._next_seq(),
                    )
                elif op == "cal_r":
                    bridge.set_stream("query")
                    payload = pack_cal_r(
                        float(req.get("max_current", 1.5)),
                        int(req.get("n_points", 0)),
                        bridge._next_seq(),
                    )
                elif op == "cal_l":
                    bridge.set_stream("query")
                    payload = pack_cal_l(
                        float(req.get("step_voltage", 3.0)),
                        int(req.get("n_trials", 0)),
                        bridge._next_seq(),
                    )
                elif op == "cal_cogging":
                    bridge.set_stream("query")
                    payload = pack_cal_cogging(
                        float(req.get("velocity", 1.0)),
                        float(req.get("record_revs", 0.0)),
                        bridge._next_seq(),
                    )
                elif op == "cal_abort":
                    bridge.set_stream("query")
                    payload = pack_cal_abort(bridge._next_seq())
                elif op == "info":
                    self._json(200, {"ok": True, **bridge.probe_motor()})
                    return
                else:
                    self._json(400, {"ok": False, "error": f"unknown op {op}"})
                    return
                ack = bridge.send_cmd(payload)
                self._json(
                    200,
                    {
                        "ok": True,
                        "cmd": ack.cmd,
                        "status": ack.status,
                        "seq": ack.seq,
                    },
                )
            except Exception as exc:  # noqa: BLE001
                self._json(500, {"ok": False, "error": str(exc)})

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--node", type=int, default=1)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--interface",
        default="",
        help="optional: auto-connect this SocketCAN iface on start",
    )
    args = parser.parse_args()

    bridge = CanBridge(args.node)
    if args.interface:
        result = bridge.connect(args.interface)
        if not result.get("ok"):
            print(f"auto-connect failed: {result}", file=sys.stderr)
        elif result.get("motor_ok"):
            info = result.get("info") or {}
            print(
                f"motor ok: {info.get('motor')} fw={info.get('fw_version')} "
                f"bus={info.get('bus_v')}V"
            )
        else:
            print(f"motor probe failed: {result.get('motor_error')}", file=sys.stderr)

    server = ThreadingHTTPServer((args.host, args.port), make_handler(bridge))
    print(f"xtellar GUI: http://{args.host}:{args.port}")
    print("Open the page, Scan ports, then Connect. Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        bridge.close()


if __name__ == "__main__":
    main()
