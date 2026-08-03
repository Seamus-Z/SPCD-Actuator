#!/usr/bin/env python3
"""xtellar local web GUI (binary CAN-FD).

  python3 tools/host/gui/web_server.py
  open http://127.0.0.1:8765
  在页面里扫描 / 选择 / 连接 CAN 口；连接后自动探测电机 Info
"""
from __future__ import annotations

import argparse
import json
import mimetypes
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from queue import Empty, Queue
from contextlib import redirect_stderr
from io import StringIO

GUI_ROOT = Path(__file__).resolve().parent
HOST_ROOT = GUI_ROOT.parent
STATIC_DIR = GUI_ROOT / "static"
sys.path.insert(0, str(HOST_ROOT))

from xt_proto import (  # noqa: E402
    CMD_INFO,
    CMD_SNAP,
    SNAP_CHANNEL_KEYS,
    SNAP_CHANNELS,
    SNAP_MAX_SAMPLES,
    STATUS_OK,
    STATUS_NOT_RUN,
    Ack,
    EncTelem,
    Info,
    SnapData,
    SnapMeta,
    Telemetry,
    pack_dq,
    pack_info,
    pack_snap,
    pack_stop,
    pack_vfoc,
    parse_frame,
    cmd_id,
    tel_id,
)

# Linux ARPHRD_CAN
_ARPHRD_CAN = "280"

_CMD_NAMES = {
    0: "stop",
    1: "dq",
    2: "vfoc",
    3: "raw",
    4: "info",
    5: "snap",
}
_MODE_NAMES = {
    0: "stop",
    1: "raw",
    2: "vfoc",
    3: "dq",
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
        self.motor_ok = False
        self.motor_error: str | None = None
        self.info: Info | None = None
        self._acks: Queue = Queue()
        self._infos: Queue = Queue()
        self._snap_metas: Queue = Queue()
        self._snap_datas: Queue = Queue()
        self._rx_stop = threading.Event()
        self._rx_thread: threading.Thread | None = None
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
            self.motor_ok = False
            self.motor_error = None
            self.info = None
        self._drain_queues()
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
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
        for q in (self._acks, self._infos, self._snap_metas, self._snap_datas):
            while True:
                try:
                    q.get_nowait()
                except Empty:
                    break

    def _next_seq(self) -> int:
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
            if isinstance(msg, Telemetry):
                with self._lock:
                    self.latest = msg
                self.msglog.maybe_telem(msg)
            elif isinstance(msg, EncTelem):
                with self._lock:
                    self.latest_enc = msg
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

    def capture_snap(
        self,
        n_samples: int = SNAP_MAX_SAMPLES,
        decimate: int = 1,
        timeout: float = 8.0,
    ) -> dict:
        if self.bus is None:
            return {"ok": False, "error": "CAN not connected"}
        self._drain_queues()
        try:
            self._send_raw(
                pack_snap(n_samples=n_samples, decimate=decimate, seq=self._next_seq())
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
                + (" (start dq/vfoc first)" if ack.status == STATUS_NOT_RUN else "")
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

    def telem_dict(self):
        motor = self._motor_snapshot()
        with self._lock:
            connected = self.bus is not None
            interface = self.interface
            t = self.latest
            enc = self.latest_enc
        if not connected:
            return {
                "ok": False,
                "connected": False,
                "error": "not connected",
                "ports": scan_can_ports(),
                **motor,
            }
        if t is None:
            return {
                "ok": False,
                "connected": True,
                "interface": interface,
                "error": "no telemetry yet",
                **motor,
            }
        enc_fields = {
            "enc_raw": enc.raw if enc else 0,
            "enc_theta_mech_rad": enc.theta_mech_rad if enc else 0.0,
            "enc_theta_elec_rad": enc.theta_elec_rad if enc else 0.0,
            "enc_sign": enc.sign if enc else 1,
            "enc_ok": bool(enc.ok) if enc else bool(getattr(t, "enc_ok", False)),
        }
        return {
            "ok": True,
            "connected": True,
            "interface": interface,
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
            **motor,
        }

    def close(self):
        self.disconnect()


def info_motor_line(info: Info) -> str:
    return (
        f"motor={info.motor} fw={info.fw_version} node={info.node_id} "
        f"family={info.family} pwm={info.pwm_hz}Hz bus={info.bus_v}V "
        f"Imax={info.i_max_a}A poles={info.pole_pairs} "
        f"R={info.r_ohm}ohm L={info.l_h * 1e6:.0f}uH"
    )


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
                self._json(200, bridge.telem_dict())
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
            if path == "/api/messages/clear":
                bridge.msglog.clear()
                self._json(200, {"ok": True})
                return
            if path != "/api/cmd":
                self.send_error(404)
                return

            op = req.get("op")
            try:
                if op == "stop":
                    payload = pack_stop(bridge._next_seq())
                elif op == "dq":
                    payload = pack_dq(
                        float(req.get("id", 0)),
                        float(req.get("iq", 0)),
                        float(req.get("omega", 0)),
                        bridge._next_seq(),
                    )
                elif op == "vfoc":
                    payload = pack_vfoc(
                        float(req.get("theta", 0)),
                        float(req.get("v", 0)),
                        float(req.get("omega", 0)),
                        bridge._next_seq(),
                    )
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
