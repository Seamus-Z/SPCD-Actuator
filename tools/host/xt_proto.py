# xtellar CAN-FD binary protocol (mirrors fw/protocol/inc/protocol/xt_can.h).
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional

MAGIC = 0x58
VERSION = 1

TYPE_CMD = 1
TYPE_TEL = 2
TYPE_ACK = 3
TYPE_INFO = 4
TYPE_SNAP_META = 5
TYPE_SNAP_DATA = 6
TYPE_ENC = 7
TYPE_CAL = 8
TYPE_CTRL_REPLY = 9
TYPE_CONF = 10

CMD_STOP = 0
CMD_INFO = 4
CMD_SNAP = 5
CMD_SERVO = 6
CMD_CAL = 7
CMD_QUERY = 8
CMD_ENC_COMP = 9
CMD_CONF = 10

CAL_SUB_ABORT = 0
CAL_SUB_ENC_PHASE = 1
CAL_SUB_ENC_LOCK = 2
CAL_SUB_BEMF_IDENT = 3  # closed-loop Ke sweep; see xt_can.h kCalSubBemf
CAL_SUB_RESISTANCE = 4  # Id sweep; see xt_can.h kCalSubResistance
CAL_SUB_INDUCTANCE = 5  # Vd step response; see xt_can.h kCalSubInductance
CAL_SUB_COGGING = 6     # slow spin q-current map; see xt_can.h kCalSubCogging

ENC_COMP_OP_CLEAR = 0
ENC_COMP_OP_CHUNK = 1
ENC_COMP_OP_COMMIT = 2
ENC_COMP_TABLE_SIZE = 256
ENC_COMP_CHUNK_SIZE = 32

# kCmdConf ops / groups (see xt_can.h).
CONF_OP_GET = 0
CONF_OP_SET = 1       # RAM only
CONF_OP_SAVE = 2      # RAM -> flash
CONF_OP_LOAD = 3      # flash -> RAM -> apply
CONF_OP_DEFAULTS = 4  # board defaults -> RAM -> apply
CONF_GROUP_MOTOR = 0
CONF_GROUP_FOC = 1
CONF_GROUP_SERVO = 2
CONF_GROUP_ENCODER = 3
CONF_GROUP_CAL = 4
CONF_GROUP_ALL = 0xFF
CONF_FLAG_FLASH_VALID = 1 << 0
CAL_FLAG_ENCODER = 1 << 0
CAL_FLAG_RESISTANCE = 1 << 1
CAL_FLAG_INDUCTANCE = 1 << 2
CAL_FLAG_BEMF = 1 << 3
CAL_FLAG_COGGING = 1 << 4
CAL_FLAG_ENC_COMP = 1 << 5

STATUS_OK = 0
STATUS_BAD_LEN = 1
STATUS_BAD_CMD = 2
STATUS_NOT_RUN = 3
STATUS_FAIL = 4

SNAP_CH_DEFAULT = 0x7F
SNAP_MAX_SAMPLES = 512
SNAP_CHANNELS = 7
SNAP_SAMPLES_PER_FRAME = 4
SNAP_CHANNEL_KEYS = ["id_a", "iq_a", "i1_a", "i2_a", "i3_a",
                     "theta_mech_rad", "theta_elec_rad"]

FLAG_PWM_ON = 1 << 0
FLAG_CISR = 1 << 1
FLAG_DQ_VALID = 1 << 2
FLAG_FAULT = 1 << 3
FLAG_ENC_OK = 1 << 4
FLAG_ENC_MODE = 1 << 5

MODE_STOP = 0
MODE_SERVO = 1
MODE_CAL = 2
MODE_CURRENT = 3

SERVO_CTRL_POSITION = 0
SERVO_CTRL_CURRENT = 1
SERVO_CTRL_MIT = 2
# int32 sentinel for velocity mode (moteus position=NaN)
POSITION_NAN_MRAD = -2147483648


def cmd_id(node_id: int = 1) -> int:
    return 0x100 + node_id


def tel_id(node_id: int = 1) -> int:
    return 0x180 + node_id


def pack_header(msg_type: int, seq: int) -> bytes:
    return struct.pack("<BBBB", MAGIC, VERSION, msg_type, seq & 0xFF)


def pack_stop(seq: int = 0) -> bytes:
    return pack_header(TYPE_CMD, seq) + struct.pack("<B", CMD_STOP)


def pack_query(seq: int = 0) -> bytes:
    return pack_header(TYPE_CMD, seq) + struct.pack("<B", CMD_QUERY)


def pack_info(seq: int = 0) -> bytes:
    return pack_header(TYPE_CMD, seq) + struct.pack("<B", CMD_INFO)


def pack_snap(
    n_samples: int = SNAP_MAX_SAMPLES,
    decimate: int = 1,
    channel_mask: int = SNAP_CH_DEFAULT,
    seq: int = 0,
) -> bytes:
    return pack_header(TYPE_CMD, seq) + struct.pack(
        "<BHBBH",
        CMD_SNAP,
        int(n_samples) & 0xFFFF,
        int(decimate) & 0xFF,
        0,
        int(channel_mask) & 0xFFFF,
    )


def _rad_to_mrad_or_nan(value: float | None) -> int:
    if value is None:
        return POSITION_NAN_MRAD
    return int(round(float(value) * 1000.0))


def _scale_to_milli(scale: float | None, default: int = 1000) -> int:
    if scale is None:
        return default
    return max(0, min(65535, int(round(float(scale) * 1000.0))))


# ServoRequest wire size (after cmd byte): 48
# control, reserved0, timeout_ms, 9*i32, 4*u16
_SERVO_REQ_FMT = "<BBHiiiiiiiiiHHHH"
_SERVO_CMD_FMT = "<B" + _SERVO_REQ_FMT[1:]  # cmd + request


def pack_servo(
    omega_mech_rad_s: float,
    id_a: float = 0.0,
    seq: int = 0,
    position_rad: float | None = None,
    stop_position_rad: float | None = None,
    max_torque_nm: float | None = None,
    feedforward_nm: float = 0.0,
    velocity_limit_rad_s: float | None = None,
    accel_limit_rad_s2: float | None = None,
    kp_scale: float = 1.0,
    kd_scale: float = 1.0,
    ilimit_scale: float = 1.0,
    timeout_ms: int = 0,
) -> bytes:
    """moteus-style kPosition servo via extended kCmdServo payload.

    position_rad=None → velocity mode (position=NaN).
    stop_position_rad=None → no stop clamp.
    *_limit=None → firmware board defaults.
    max_torque_nm=None/0 → firmware board default.
    """
    return pack_header(TYPE_CMD, seq) + struct.pack(
        _SERVO_CMD_FMT,
        CMD_SERVO,
        SERVO_CTRL_POSITION,
        0,  # reserved0
        int(timeout_ms) & 0xFFFF,
        _rad_to_mrad_or_nan(position_rad),
        int(round(omega_mech_rad_s * 1000.0)),
        int(round(id_a * 1000.0)),
        0,
        _rad_to_mrad_or_nan(stop_position_rad),
        0 if max_torque_nm is None else int(round(max_torque_nm * 1000.0)),
        int(round(feedforward_nm * 1000.0)),
        _rad_to_mrad_or_nan(velocity_limit_rad_s),
        _rad_to_mrad_or_nan(accel_limit_rad_s2),
        _scale_to_milli(kp_scale),
        _scale_to_milli(kd_scale),
        _scale_to_milli(ilimit_scale),
        0,
    )


def pack_current(id_a: float, iq_a: float, seq: int = 0) -> bytes:
    """Direct Id/Iq current mode via extended kCmdServo payload."""
    return pack_header(TYPE_CMD, seq) + struct.pack(
        _SERVO_CMD_FMT,
        CMD_SERVO,
        SERVO_CTRL_CURRENT,
        0,
        0,
        0,
        0,
        int(round(id_a * 1000.0)),
        int(round(iq_a * 1000.0)),
        POSITION_NAN_MRAD,
        0,
        0,
        POSITION_NAN_MRAD,
        POSITION_NAN_MRAD,
        1000,
        1000,
        1000,
        0,
    )


def pack_mit(
    position_rad: float,
    velocity_rad_s: float = 0.0,
    kp: float = 0.0,
    kd: float = 0.0,
    feedforward_nm: float = 0.0,
    max_torque_nm: float | None = None,
    seq: int = 0,
) -> bytes:
    """MIT impedance via kCmdServo: T=Kp(Pcmd-Pfdb)+Kd(Vcmd-Vfdb)+Tff.

    Position is continuous multi-turn mechanical radians (no wrap).
    Wire reuse: id_mA=Kp[mNm/rad], iq_mA=Kd[mNm/(rad/s)].
    """
    return pack_header(TYPE_CMD, seq) + struct.pack(
        _SERVO_CMD_FMT,
        CMD_SERVO,
        SERVO_CTRL_MIT,
        0,
        0,
        int(round(float(position_rad) * 1000.0)),
        int(round(float(velocity_rad_s) * 1000.0)),
        int(round(float(kp) * 1000.0)),
        int(round(float(kd) * 1000.0)),
        POSITION_NAN_MRAD,
        0 if max_torque_nm is None else int(round(float(max_torque_nm) * 1000.0)),
        int(round(float(feedforward_nm) * 1000.0)),
        POSITION_NAN_MRAD,
        POSITION_NAN_MRAD,
        1000,
        1000,
        1000,
        0,
    )


def pack_cal_enc(
    current_a: float = 1.0,
    omega_elec_rad_s: float = 40.0,
    seq: int = 0,
) -> bytes:
    """Start current-regulated encoder spin calibration (both directions)."""
    return pack_header(TYPE_CMD, seq) + struct.pack(
        "<BBBii",
        CMD_CAL,
        CAL_SUB_ENC_PHASE,
        0,
        int(round(current_a * 1000)),
        int(round(omega_elec_rad_s * 1000)),
    )


def pack_cal_lock(current_a: float = 1.0, seq: int = 0) -> bytes:
    """Start current-regulated coarse encoder lock calibration."""
    return pack_header(TYPE_CMD, seq) + struct.pack(
        "<BBBii",
        CMD_CAL,
        CAL_SUB_ENC_LOCK,
        0,
        int(round(current_a * 1000)),
        0,
    )


def pack_cal_bemf(
    max_speed_rad_s: float = 60.0, n_points: int = 0, seq: int = 0
) -> bytes:
    """Start closed-loop Ke identification (velocity sweep, Id=0).

    Requires encoder phase already calibrated (run cal_lock/cal_enc first).
    n_points: 0 => firmware default (5); else clamped to 3..8 on-device.
    """
    return pack_header(TYPE_CMD, seq) + struct.pack(
        "<BBBii",
        CMD_CAL,
        CAL_SUB_BEMF_IDENT,
        0,
        int(n_points),
        int(round(max_speed_rad_s * 1000)),
    )


def pack_cal_r(
    max_current_a: float = 1.5, n_points: int = 0, seq: int = 0
) -> bytes:
    """Start closed-loop R (phase resistance) identification (Id sweep).

    No encoder required. n_points: 0 => firmware default (5), else 3..8.
    """
    return pack_header(TYPE_CMD, seq) + struct.pack(
        "<BBBii",
        CMD_CAL,
        CAL_SUB_RESISTANCE,
        0,
        int(n_points),
        int(round(max_current_a * 1000)),
    )


def pack_cal_l(
    step_voltage_v: float = 3.0, n_trials: int = 0, seq: int = 0
) -> bytes:
    """Start locked-rotor D/Q inductance identification.

    No encoder required. Uses the currently-effective R, measures Ld and Lq,
    and stores both. Identify R first. n_trials applies to each axis.
    """
    return pack_header(TYPE_CMD, seq) + struct.pack(
        "<BBBii",
        CMD_CAL,
        CAL_SUB_INDUCTANCE,
        0,
        int(round(step_voltage_v * 1000)),
        int(n_trials),
    )


def pack_cal_abort(seq: int = 0) -> bytes:
    return pack_header(TYPE_CMD, seq) + struct.pack(
        "<BBBii",
        CMD_CAL,
        CAL_SUB_ABORT,
        0,
        0,
        0,
    )

def pack_cal_cogging(
    velocity_mech_rad_s: float = 1.0, record_revs: float = 0.0, seq: int = 0
) -> bytes:
    """Start cogging-torque compensation measurement (slow fwd/rev spin).

    Requires encoder phase already calibrated and a working velocity loop.
    velocity_mech_rad_s: constant sweep speed. record_revs: 0 => firmware
    default (1 rev/direction).
    """
    return pack_header(TYPE_CMD, seq) + struct.pack(
        "<BBBii",
        CMD_CAL,
        CAL_SUB_COGGING,
        0,
        int(round(record_revs * 100)),          # voltage_mV field reused
        int(round(velocity_mech_rad_s * 1000)),  # omega_elec_mrad_s reused
    )


def pack_enc_comp_clear(seq: int = 0) -> bytes:
    """Disable and clear the encoder geometric compensation table."""
    return pack_header(TYPE_CMD, seq) + bytes([CMD_ENC_COMP]) + struct.pack(
        "<BBHi32s",
        ENC_COMP_OP_CLEAR,
        0,
        0,
        0,
        bytes(32),
    )


def pack_enc_comp_chunk(chunk: int, data: bytes, seq: int = 0) -> bytes:
    """Upload one 32-byte chunk (chunk index 0..7) of the 256-byte table."""
    if chunk < 0 or chunk > 7:
        raise ValueError("chunk must be 0..7")
    raw = bytes(data)
    if len(raw) != ENC_COMP_CHUNK_SIZE:
        raise ValueError("chunk data must be 32 bytes")
    return pack_header(TYPE_CMD, seq) + bytes([CMD_ENC_COMP]) + struct.pack(
        "<BBHi32s",
        ENC_COMP_OP_CHUNK,
        int(chunk) & 0xFF,
        0,
        0,
        raw,
    )


def pack_enc_comp_commit(peak_corr_rad: float, seq: int = 0) -> bytes:
    """Enable table. peak_corr_rad is max |correction| in radians."""
    scale_urad = int(round(float(peak_corr_rad) * 1_000_000.0))
    return pack_header(TYPE_CMD, seq) + bytes([CMD_ENC_COMP]) + struct.pack(
        "<BBHi32s",
        ENC_COMP_OP_COMMIT,
        0,
        0,
        scale_urad,
        bytes(32),
    )



# ConfRequest wire size (after cmd byte): 4 = op, group, reserved16.
# Group payloads are native LE float blobs (Motor32 / Foc24 / Servo48 / Encoder16).
_MOTOR_CONF_FMT = "<ffffffff"
_FOC_CONF_FMT = "<ffffff"
_SERVO_CONF_FMT = "<ffffffffffff"
_ENCODER_CONF_FMT = "<ffff"
assert struct.calcsize(_MOTOR_CONF_FMT) == 32
assert struct.calcsize(_FOC_CONF_FMT) == 24
assert struct.calcsize(_SERVO_CONF_FMT) == 48
assert struct.calcsize(_ENCODER_CONF_FMT) == 16


def _pack_conf_req(op: int, group: int, seq: int = 0,
                   payload: bytes = b"") -> bytes:
    return (
        pack_header(TYPE_CMD, seq)
        + struct.pack("<BBBH", CMD_CONF, op & 0xFF, group & 0xFF, 0)
        + payload
    )


def pack_conf_get(group: int, seq: int = 0) -> bytes:
    """Request one config group (GET replies with TYPE_CONF + payload)."""
    return _pack_conf_req(CONF_OP_GET, group, seq)


def pack_conf_set(group: int, payload: bytes, seq: int = 0) -> bytes:
    """Set one config group in RAM. payload from pack_*_conf helpers."""
    return _pack_conf_req(CONF_OP_SET, group, seq, payload)


def pack_conf_save(group: int = CONF_GROUP_ALL, seq: int = 0) -> bytes:
    """Persist RAM config to flash (group=ALL or a single group)."""
    return _pack_conf_req(CONF_OP_SAVE, group, seq)


def pack_conf_load(group: int = CONF_GROUP_ALL, seq: int = 0) -> bytes:
    """Load flash -> RAM and apply (group=ALL or a single group)."""
    return _pack_conf_req(CONF_OP_LOAD, group, seq)


def pack_conf_defaults(group: int = CONF_GROUP_ALL, seq: int = 0) -> bytes:
    """Reset RAM to board defaults and apply."""
    return _pack_conf_req(CONF_OP_DEFAULTS, group, seq)


CONF_OP_BY_NAME = {
    "get": CONF_OP_GET,
    "set": CONF_OP_SET,
    "save": CONF_OP_SAVE,
    "load": CONF_OP_LOAD,
    "defaults": CONF_OP_DEFAULTS,
}
CONF_GROUP_BY_NAME = {
    "motor": CONF_GROUP_MOTOR,
    "foc": CONF_GROUP_FOC,
    "servo": CONF_GROUP_SERVO,
    "encoder": CONF_GROUP_ENCODER,
    "cal": CONF_GROUP_CAL,
    "all": CONF_GROUP_ALL,
}
CONF_GROUP_TO_NAME = {v: k for k, v in CONF_GROUP_BY_NAME.items()}

# GUI /api short names <-> wire struct fields.
_MOTOR_GUI_TO_WIRE = {
    "pole_pairs": "pole_pairs",
    "R": "resistance_ohm",
    "L": "inductance_H",
    "bemf_Vpeak_per_krpm": "bemf_Vpeak_per_krpm",
    "max_phase_current_A": "max_phase_current_A",
    "fw_speed_rad_s": "fw_speed_rad_s",
    "bus_V": "bus_V",
}
_FOC_GUI_TO_WIRE = {
    "bandwidth_hz": "bandwidth_hz",
    "bemf_ff": "bemf_feedforward",
    "current_ff": "current_feedforward",
    "cross_ff": "cross_coupling_feedforward",
    "max_current_rate": "max_current_desired_rate_A_s",
}
_SERVO_GUI_TO_WIRE = {
    "kp": "kp",
    "ki": "ki",
    "kd": "kd",
    "ilimit": "ilimit",
    "max_iq": "max_iq_A",
    "vel_threshold": "velocity_threshold",
    "slip": "max_position_slip_rad",
    "vel_err": "max_velocity_error_rad_s",
    "default_vel_limit": "default_velocity_limit_rad_s",
    "default_accel_limit": "default_accel_limit_rad_s2",
    "sign": "sign_f",
}
_ENCODER_GUI_TO_WIRE = {
    "pll_filter_hz": "pll_filter_hz",
    "spike_error_rad": "spike_error_rad",
    "filter_us": "filter_us",
}
_GUI_TO_WIRE = {
    CONF_GROUP_MOTOR: _MOTOR_GUI_TO_WIRE,
    CONF_GROUP_FOC: _FOC_GUI_TO_WIRE,
    CONF_GROUP_SERVO: _SERVO_GUI_TO_WIRE,
    CONF_GROUP_ENCODER: _ENCODER_GUI_TO_WIRE,
}
_WIRE_TO_GUI = {
    g: {w: s for s, w in m.items()} for g, m in _GUI_TO_WIRE.items()
}


def conf_op_id(op: int | str) -> int:
    if isinstance(op, str):
        key = op.strip().lower()
        if key not in CONF_OP_BY_NAME:
            raise ValueError(f"bad conf op {op!r}")
        return CONF_OP_BY_NAME[key]
    return int(op) & 0xFF


def conf_group_id(group: int | str) -> int:
    if isinstance(group, str):
        key = group.strip().lower()
        if key not in CONF_GROUP_BY_NAME:
            raise ValueError(f"bad conf group {group!r}")
        return CONF_GROUP_BY_NAME[key]
    return int(group) & 0xFF


def _finite_or_none(value: float):
    if value != value or value in (float("inf"), float("-inf")):
        return None
    return float(value)


def _gui_fields_to_wire(group: int, fields: dict | None) -> dict:
    mapping = _GUI_TO_WIRE.get(group)
    if mapping is None:
        raise ValueError(f"conf set requires concrete group, got {group}")
    src = fields or {}
    out = {}
    for gui_key, wire_key in mapping.items():
        if gui_key not in src:
            continue
        val = src[gui_key]
        if val is None or val == "":
            if group == CONF_GROUP_SERVO and gui_key == "default_accel_limit":
                out[wire_key] = None  # => NaN
            continue
        out[wire_key] = float(val)
    return out


def _wire_fields_to_gui(group: int, wire: dict) -> dict:
    mapping = _WIRE_TO_GUI.get(group, {})
    out = {}
    for wire_key, gui_key in mapping.items():
        if wire_key not in wire:
            continue
        out[gui_key] = _finite_or_none(float(wire[wire_key]))
    return out


def pack_conf_fields(group: int | str, fields: dict | None) -> bytes:
    """Pack GUI short-name fields into a group payload blob."""
    group_i = conf_group_id(group)
    if group_i == CONF_GROUP_CAL:
        src = fields or {}
        return pack_cal_status_conf(
            flags=int(src.get("flags", 0) or 0),
            resistance_ohm=src.get("R"),
            inductance_d_H=src.get("Ld"),
            inductance_q_H=src.get("Lq"),
            bemf_v_per_hz=src.get("Ke"),
        )
    wire = _gui_fields_to_wire(group_i, fields)
    if group_i == CONF_GROUP_MOTOR:
        return pack_motor_conf(
            pole_pairs=float(wire.get("pole_pairs", 0.0)),
            resistance_ohm=float(wire.get("resistance_ohm", 0.0)),
            inductance_H=float(wire.get("inductance_H", 0.0)),
            bemf_Vpeak_per_krpm=float(wire.get("bemf_Vpeak_per_krpm", 0.0)),
            max_phase_current_A=float(wire.get("max_phase_current_A", 0.0)),
            fw_speed_rad_s=float(wire.get("fw_speed_rad_s", 0.0)),
            bus_V=float(wire.get("bus_V", 0.0)),
        )
    if group_i == CONF_GROUP_FOC:
        return pack_foc_conf(
            bandwidth_hz=float(wire.get("bandwidth_hz", 0.0)),
            bemf_feedforward=float(wire.get("bemf_feedforward", 0.0)),
            current_feedforward=float(wire.get("current_feedforward", 0.0)),
            cross_coupling_feedforward=float(
                wire.get("cross_coupling_feedforward", 0.0)
            ),
            max_current_desired_rate_A_s=float(
                wire.get("max_current_desired_rate_A_s", 0.0)
            ),
        )
    if group_i == CONF_GROUP_SERVO:
        accel = wire.get("default_accel_limit_rad_s2", None)
        return pack_servo_conf(
            kp=float(wire.get("kp", 0.0)),
            ki=float(wire.get("ki", 0.0)),
            kd=float(wire.get("kd", 0.0)),
            ilimit=float(wire.get("ilimit", 0.0)),
            max_iq_A=float(wire.get("max_iq_A", 0.0)),
            velocity_threshold=float(wire.get("velocity_threshold", 0.0)),
            max_position_slip_rad=float(wire.get("max_position_slip_rad", 0.0)),
            max_velocity_error_rad_s=float(
                wire.get("max_velocity_error_rad_s", 0.0)
            ),
            default_velocity_limit_rad_s=float(
                wire.get("default_velocity_limit_rad_s", 0.0)
            ),
            default_accel_limit_rad_s2=accel,
            sign_f=float(wire.get("sign_f", 1.0)),
        )
    if group_i == CONF_GROUP_ENCODER:
        return pack_encoder_conf(
            pll_filter_hz=float(wire.get("pll_filter_hz", 0.0)),
            spike_error_rad=float(wire.get("spike_error_rad", 0.0)),
            filter_us=float(wire.get("filter_us", 0.0)),
        )
    raise ValueError(f"unsupported conf group {group_i}")


def unpack_conf_fields(group: int | str, payload: bytes) -> dict:
    """Unpack group payload into GUI short-name fields (NaN -> None)."""
    group_i = conf_group_id(group)
    if group_i == CONF_GROUP_MOTOR:
        wire = unpack_motor_conf(payload)
    elif group_i == CONF_GROUP_FOC:
        wire = unpack_foc_conf(payload)
    elif group_i == CONF_GROUP_SERVO:
        wire = unpack_servo_conf(payload)
    elif group_i == CONF_GROUP_ENCODER:
        wire = unpack_encoder_conf(payload)
    elif group_i == CONF_GROUP_CAL:
        return unpack_cal_status_conf(payload)
    else:
        return {}
    return _wire_fields_to_gui(group_i, wire)


def pack_conf(
    op: int | str,
    group: int | str = "all",
    fields: dict | None = None,
    seq: int = 0,
) -> bytes:
    """High-level kCmdConf packer used by the GUI bridge.

    op/group accept int ids or names ("get"/"motor"/...).
    SET requires a concrete group and GUI short-name fields.
    """
    op_i = conf_op_id(op)
    group_i = conf_group_id(group)
    if op_i == CONF_OP_GET:
        return pack_conf_get(group_i, seq)
    if op_i == CONF_OP_SET:
        return pack_conf_set(group_i, pack_conf_fields(group_i, fields), seq)
    if op_i == CONF_OP_SAVE:
        return pack_conf_save(group_i, seq)
    if op_i == CONF_OP_LOAD:
        return pack_conf_load(group_i, seq)
    if op_i == CONF_OP_DEFAULTS:
        return pack_conf_defaults(group_i, seq)
    raise ValueError(f"bad conf op id {op_i}")


def parse_conf(data: "bytes | ConfReply") -> dict:
    """Parse TYPE_CONF bytes or ConfReply into short-name fields dict."""
    if isinstance(data, ConfReply):
        reply = data
    else:
        reply = parse_frame(data)
        if not isinstance(reply, ConfReply):
            raise ValueError("not a ConfReply frame")
    return unpack_conf_fields(reply.group, reply.payload)


def pack_motor_conf(
    pole_pairs: float,
    resistance_ohm: float,
    inductance_H: float,
    bemf_Vpeak_per_krpm: float,
    max_phase_current_A: float,
    fw_speed_rad_s: float,
    bus_V: float,
    reserved0: float = 0.0,
) -> bytes:
    return struct.pack(
        _MOTOR_CONF_FMT,
        float(pole_pairs),
        float(resistance_ohm),
        float(inductance_H),
        float(bemf_Vpeak_per_krpm),
        float(max_phase_current_A),
        float(fw_speed_rad_s),
        float(bus_V),
        float(reserved0),
    )


def unpack_motor_conf(data: bytes) -> dict:
    if len(data) < 32:
        raise ValueError(f"MotorConf need 32 bytes, got {len(data)}")
    f = struct.unpack_from(_MOTOR_CONF_FMT, data, 0)
    return {
        "pole_pairs": f[0],
        "resistance_ohm": f[1],
        "inductance_H": f[2],
        "bemf_Vpeak_per_krpm": f[3],
        "max_phase_current_A": f[4],
        "fw_speed_rad_s": f[5],
        "bus_V": f[6],
        "reserved0": f[7],
    }


def pack_foc_conf(
    bandwidth_hz: float,
    bemf_feedforward: float,
    current_feedforward: float,
    cross_coupling_feedforward: float,
    max_current_desired_rate_A_s: float,
    reserved0: float = 0.0,
) -> bytes:
    return struct.pack(
        _FOC_CONF_FMT,
        float(bandwidth_hz),
        float(bemf_feedforward),
        float(current_feedforward),
        float(cross_coupling_feedforward),
        float(max_current_desired_rate_A_s),
        float(reserved0),
    )


def unpack_foc_conf(data: bytes) -> dict:
    if len(data) < 24:
        raise ValueError(f"FocConf need 24 bytes, got {len(data)}")
    f = struct.unpack_from(_FOC_CONF_FMT, data, 0)
    return {
        "bandwidth_hz": f[0],
        "bemf_feedforward": f[1],
        "current_feedforward": f[2],
        "cross_coupling_feedforward": f[3],
        "max_current_desired_rate_A_s": f[4],
        "reserved0": f[5],
    }


def pack_servo_conf(
    kp: float,
    ki: float,
    kd: float,
    ilimit: float,
    max_iq_A: float,
    velocity_threshold: float,
    max_position_slip_rad: float,
    max_velocity_error_rad_s: float,
    default_velocity_limit_rad_s: float,
    default_accel_limit_rad_s2: float | None = None,
    sign_f: float = 1.0,
    reserved0: float = 0.0,
) -> bytes:
    """Pack ServoConf. accel None => float NaN (unlimited); sign_f is ±1.0."""
    accel = (
        float("nan")
        if default_accel_limit_rad_s2 is None
        else float(default_accel_limit_rad_s2)
    )
    sign = 1.0 if float(sign_f) >= 0.0 else -1.0
    return struct.pack(
        _SERVO_CONF_FMT,
        float(kp),
        float(ki),
        float(kd),
        float(ilimit),
        float(max_iq_A),
        float(velocity_threshold),
        float(max_position_slip_rad),
        float(max_velocity_error_rad_s),
        float(default_velocity_limit_rad_s),
        accel,
        sign,
        float(reserved0),
    )


def unpack_servo_conf(data: bytes) -> dict:
    if len(data) < 48:
        raise ValueError(f"ServoConf need 48 bytes, got {len(data)}")
    f = struct.unpack_from(_SERVO_CONF_FMT, data, 0)
    return {
        "kp": f[0],
        "ki": f[1],
        "kd": f[2],
        "ilimit": f[3],
        "max_iq_A": f[4],
        "velocity_threshold": f[5],
        "max_position_slip_rad": f[6],
        "max_velocity_error_rad_s": f[7],
        "default_velocity_limit_rad_s": f[8],
        "default_accel_limit_rad_s2": f[9],
        "sign_f": f[10],
        "reserved0": f[11],
    }


def pack_encoder_conf(
    pll_filter_hz: float,
    spike_error_rad: float,
    filter_us: float,
    reserved0: float = 0.0,
) -> bytes:
    return struct.pack(
        _ENCODER_CONF_FMT,
        float(pll_filter_hz),
        float(spike_error_rad),
        float(filter_us),
        float(reserved0),
    )


def unpack_encoder_conf(data: bytes) -> dict:
    if len(data) < 16:
        raise ValueError(f"EncoderConf need 16 bytes, got {len(data)}")
    f = struct.unpack_from(_ENCODER_CONF_FMT, data, 0)
    return {
        "pll_filter_hz": f[0],
        "spike_error_rad": f[1],
        "filter_us": f[2],
        "reserved0": f[3],
    }


@dataclass
class Ack:
    seq: int
    cmd: int
    status: int


@dataclass
class Info:
    seq: int
    node_id: int
    fw_major: int
    fw_minor: int
    fw_patch: int
    pwm_hz: int
    bus_v: float
    i_max_a: float
    pole_pairs: int
    r_ohm: float
    l_h: float
    family: int
    motor: str

    @property
    def fw_version(self) -> str:
        return f"{self.fw_major}.{self.fw_minor}.{self.fw_patch}"

    def as_dict(self) -> dict:
        return {
            "node_id": self.node_id,
            "fw_version": self.fw_version,
            "fw_major": self.fw_major,
            "fw_minor": self.fw_minor,
            "fw_patch": self.fw_patch,
            "pwm_hz": self.pwm_hz,
            "bus_v": self.bus_v,
            "i_max_a": self.i_max_a,
            "pole_pairs": self.pole_pairs,
            "r_ohm": self.r_ohm,
            "l_uH": self.l_h * 1e6,
            "family": self.family,
            "motor": self.motor,
        }


@dataclass
class SnapMeta:
    seq: int
    n_samples: int
    sample_hz: int
    channel_mask: int
    channels: int
    decimate: int
    duration_us: int


@dataclass
class SnapData:
    seq: int
    start_index: int
    n: int
    # flat int16 mA samples: n * channels
    samples_mA: list



@dataclass
class CtrlReply:
    seq: int
    cmd: int
    status: int
    flags: int
    mode: int
    enc_ok: bool
    enc_sign: int
    enc_spike: int
    id_a: float
    iq_a: float
    idref_a: float
    iqref_a: float
    theta_elec_rad: float
    omega_mech_rad_s: float
    vd_v: float
    vq_v: float
    bus_v: float
    enc_raw: int
    theta_mech_rad: float
    omega_cmd_rad_s: float = 0.0
    omega_elec_rad_s: float = 0.0
    voltage_headroom_v: float = 0.0

    @property
    def pwm_on(self) -> bool:
        return bool(self.flags & FLAG_PWM_ON)

    @property
    def cisr(self) -> bool:
        return bool(self.flags & FLAG_CISR)

    @property
    def enc_mode(self) -> bool:
        return bool(self.flags & FLAG_ENC_MODE)

    def as_telemetry(self) -> "Telemetry":
        """Adapt to legacy Telemetry fields for GUI /api/telem."""
        return Telemetry(
            seq=self.seq,
            flags=self.flags,
            id_a=self.id_a,
            iq_a=self.iq_a,
            idref_a=self.idref_a,
            iqref_a=self.iqref_a,
            i1_a=0.0,
            i2_a=0.0,
            i3_a=0.0,
            theta_rad=self.theta_elec_rad,
            omega_rad_s=self.omega_mech_rad_s,
            vd_v=self.vd_v,
            vq_v=self.vq_v,
            duty_a=0,
            duty_b=0,
            duty_c=0,
            bus_v=self.bus_v,
            mode=self.mode,
        )

    def as_enc(self) -> "EncTelem":
        return EncTelem(
            seq=self.seq,
            raw=self.enc_raw,
            theta_mech_rad=self.theta_mech_rad,
            theta_elec_rad=self.theta_elec_rad,
            sign=self.enc_sign,
            ok=self.enc_ok,
        )



@dataclass
class ConfReply:
    seq: int
    op: int
    group: int
    flags: int
    payload: bytes

    @property
    def flash_valid(self) -> bool:
        return bool(self.flags & CONF_FLAG_FLASH_VALID)

    @property
    def group_name(self) -> str:
        return CONF_GROUP_TO_NAME.get(self.group, str(self.group))

    @property
    def fields(self) -> dict:
        try:
            return unpack_conf_fields(self.group, self.payload)
        except Exception:
            return {}

    def as_dict(self) -> dict:
        return {
            "seq": self.seq,
            "op": self.op,
            "group": self.group_name,
            "group_id": self.group,
            "flags": self.flags,
            "flash_valid": self.flash_valid,
            "fields": self.fields,
        }


@dataclass
class Telemetry:
    seq: int
    flags: int
    id_a: float
    iq_a: float
    idref_a: float
    iqref_a: float
    i1_a: float
    i2_a: float
    i3_a: float
    theta_rad: float
    omega_rad_s: float
    vd_v: float
    vq_v: float
    duty_a: int
    duty_b: int
    duty_c: int
    bus_v: float
    mode: int

    @property
    def pwm_on(self) -> bool:
        return bool(self.flags & FLAG_PWM_ON)

    @property
    def cisr(self) -> bool:
        return bool(self.flags & FLAG_CISR)

    @property
    def enc_ok(self) -> bool:
        return bool(self.flags & FLAG_ENC_OK)

    @property
    def enc_mode(self) -> bool:
        return bool(self.flags & FLAG_ENC_MODE)


@dataclass
class EncTelem:
    seq: int
    raw: int
    theta_mech_rad: float
    theta_elec_rad: float
    sign: int
    ok: bool


@dataclass
class CalTelem:
    seq: int
    kind: int
    state: int
    progress: float  # 0..1
    offset_rad: float
    residual_rad: float
    sign: int
    ok: bool
    samples: int  # low15=count, bit15=persisted flash write ok

    @property
    def sample_count(self) -> int:
        return self.samples & 0x7FFF

    @property
    def persisted(self) -> bool:
        return bool(self.samples & 0x8000)


_TELEM_FMT = "<BBBBHiiiiiiiiiiiHHHHBB"
# hdr4 + flags2 + 11*i32 + duty_a/b/c + bus + mode + reserved = 60
assert struct.calcsize(_TELEM_FMT) == 60

# hdr4 + node/fw4 + 7*u16 + motor12 = 34
_INFO_FMT = "<BBBBBBBBHHHHHHH12s"
assert struct.calcsize(_INFO_FMT) == 34

# hdr4 + n/hz/mask + ch/dec + duration = 16
_SNAP_META_FMT = "<BBBBHHHBBI"
assert struct.calcsize(_SNAP_META_FMT) == 16

# hdr4 + raw u16 + mech i32 + elec i32 + sign i8 + ok u8 + pad2 = 18
_ENC_FMT = "<BBBBHiibBxx"
assert struct.calcsize(_ENC_FMT) == 18
_CAL_FMT = "<BBBBBBHiibBH"
assert struct.calcsize(_CAL_FMT) == 20

# The first 56 bytes retain the original layout. Firmware >=0.4.10 appends
# measured electrical speed and available DQ voltage headroom (64 bytes total).
_CTRL_BASE_FMT = "<BBBBBBHBBbBiiiiiiiiHHii"
_CTRL_FMT = _CTRL_BASE_FMT + "ii"
assert struct.calcsize(_CTRL_BASE_FMT) == 56
assert struct.calcsize(_CTRL_FMT) == 64


_CAL_STATUS_CONF_FMT = "<Iffff"
assert struct.calcsize(_CAL_STATUS_CONF_FMT) == 20


def pack_cal_status_conf(
    flags: int = 0,
    resistance_ohm: float | None = None,
    inductance_d_H: float | None = None,
    inductance_q_H: float | None = None,
    bemf_v_per_hz: float | None = None,
) -> bytes:
    def _f(v):
        if v is None:
            return float("nan")
        return float(v)
    return struct.pack(
        _CAL_STATUS_CONF_FMT,
        int(flags) & 0xFFFFFFFF,
        _f(resistance_ohm),
        _f(inductance_d_H),
        _f(inductance_q_H),
        _f(bemf_v_per_hz),
    )


def unpack_cal_status_conf(payload: bytes) -> dict:
    if len(payload) < 20:
        raise ValueError(f"cal status payload too short: {len(payload)}")
    flags, r, ld, lq, ke = struct.unpack_from(_CAL_STATUS_CONF_FMT, payload, 0)
    def _n(v):
        return None if (v != v) else float(v)
    return {
        "flags": int(flags),
        "encoder": bool(flags & CAL_FLAG_ENCODER),
        "resistance": bool(flags & CAL_FLAG_RESISTANCE),
        "inductance": bool(flags & CAL_FLAG_INDUCTANCE),
        "bemf": bool(flags & CAL_FLAG_BEMF),
        "cogging": bool(flags & CAL_FLAG_COGGING),
        "enc_comp": bool(flags & CAL_FLAG_ENC_COMP),
        "R": _n(r),
        "Ld": _n(ld),
        "Lq": _n(lq),
        "Ke": _n(ke),
    }


def parse_frame(data: bytes) -> Optional[object]:
    if len(data) < 4:
        return None
    magic, ver, typ, seq = struct.unpack_from("<BBBB", data, 0)
    if magic != MAGIC or ver != VERSION:
        return None
    if typ == TYPE_ACK and len(data) >= 8:
        cmd, status, _ = struct.unpack_from("<BBH", data, 4)
        return Ack(seq=seq, cmd=cmd, status=status)
    if typ == TYPE_INFO and len(data) >= 34:
        fields = struct.unpack_from(_INFO_FMT, data, 0)
        motor = fields[15].split(b"\x00", 1)[0].decode("ascii", errors="replace")
        return Info(
            seq=fields[3],
            node_id=fields[4],
            fw_major=fields[5],
            fw_minor=fields[6],
            fw_patch=fields[7],
            pwm_hz=fields[8],
            bus_v=fields[9] / 1000.0,
            i_max_a=fields[10] / 1000.0,
            pole_pairs=fields[11],
            r_ohm=fields[12] / 1000.0,
            l_h=fields[13] * 1e-6,
            family=fields[14],
            motor=motor,
        )
    if typ == TYPE_SNAP_META and len(data) >= 16:
        fields = struct.unpack_from(_SNAP_META_FMT, data, 0)
        return SnapMeta(
            seq=fields[3],
            n_samples=fields[4],
            sample_hz=fields[5],
            channel_mask=fields[6],
            channels=fields[7],
            decimate=fields[8],
            duration_us=fields[9],
        )
    if typ == TYPE_SNAP_DATA and len(data) >= 8:
        start_index, n, _res = struct.unpack_from("<HBB", data, 4)
        ch = SNAP_CHANNELS
        need = 8 + n * ch * 2
        if len(data) < need or n > SNAP_SAMPLES_PER_FRAME:
            import sys as _sys
            _sys.stderr.write(
                f"[SNAPDUMP] REJECT len={len(data)} n={n} ch={ch} "
                f"need={need} maxn={SNAP_SAMPLES_PER_FRAME} "
                f"start={start_index} bytes={list(data[:16])}\n")
            _sys.stderr.flush()
            return None
        count = n * ch
        samples = list(struct.unpack_from("<" + "h" * count, data, 8))
        return SnapData(seq=seq, start_index=start_index, n=n, samples_mA=samples)
    if typ == TYPE_TEL and len(data) >= 60:
        fields = struct.unpack_from(_TELEM_FMT, data, 0)
        return Telemetry(
            seq=fields[3],
            flags=fields[4],
            id_a=fields[5] / 1000.0,
            iq_a=fields[6] / 1000.0,
            idref_a=fields[7] / 1000.0,
            iqref_a=fields[8] / 1000.0,
            i1_a=fields[9] / 1000.0,
            i2_a=fields[10] / 1000.0,
            i3_a=fields[11] / 1000.0,
            theta_rad=fields[12] / 1000.0,
            omega_rad_s=fields[13] / 1000.0,
            vd_v=fields[14] / 1000.0,
            vq_v=fields[15] / 1000.0,
            duty_a=fields[16],
            duty_b=fields[17],
            duty_c=fields[18],
            bus_v=fields[19] / 1000.0,
            mode=fields[20],
        )
    if typ == TYPE_ENC and len(data) >= 18:
        fields = struct.unpack_from(_ENC_FMT, data, 0)
        return EncTelem(
            seq=fields[3],
            raw=fields[4],
            theta_mech_rad=fields[5] / 1000.0,
            theta_elec_rad=fields[6] / 1000.0,
            sign=fields[7],
            ok=bool(fields[8]),
        )
    if typ == TYPE_CTRL_REPLY and len(data) >= 56:
        ctrl_fmt = _CTRL_FMT if len(data) >= 64 else _CTRL_BASE_FMT
        fields = struct.unpack_from(ctrl_fmt, data, 0)
        return CtrlReply(
            seq=fields[3],
            cmd=fields[4],
            status=fields[5],
            flags=fields[6],
            mode=fields[7],
            enc_ok=bool(fields[8]),
            enc_sign=fields[9],
            enc_spike=int(fields[10]),
            id_a=fields[11] / 1000.0,
            iq_a=fields[12] / 1000.0,
            idref_a=fields[13] / 1000.0,
            iqref_a=fields[14] / 1000.0,
            theta_elec_rad=fields[15] / 1000.0,
            omega_mech_rad_s=fields[16] / 1000.0,
            vd_v=fields[17] / 1000.0,
            vq_v=fields[18] / 1000.0,
            bus_v=fields[19] / 1000.0,
            enc_raw=fields[20],
            theta_mech_rad=fields[21] / 1000.0,
            omega_cmd_rad_s=fields[22] / 1000.0,
            omega_elec_rad_s=fields[23] / 1000.0 if len(fields) > 23 else 0.0,
            voltage_headroom_v=fields[24] / 1000.0 if len(fields) > 24 else 0.0,
        )
    if typ == TYPE_CAL and len(data) >= 20:
        fields = struct.unpack_from(_CAL_FMT, data, 0)
        kind = fields[4]
        # Field reuse per kind (see xt_can.h kCalSub* docs):
        #   Bemf: offset=Ke_dq*1e6 [V*s/rad], residual=r^2*1e6
        #   Resistance: offset=R*1e6 [uOhm], residual=r^2*1e6
        #   Inductance: offset=Ld*1e9 [nH], residual=Lq*1e9 [nH]
        #   everything else (enc phase/lock): milli-units
        # Cogging: offset=table scale (A) *1e6, residual=peak current (A) *1e6
        if kind in (CAL_SUB_BEMF_IDENT, CAL_SUB_RESISTANCE, CAL_SUB_COGGING):
            off_scale = res_scale = 1.0e6
        elif kind == CAL_SUB_INDUCTANCE:
            off_scale = res_scale = 1.0e9
        else:
            off_scale = res_scale = 1000.0
        return CalTelem(
            seq=fields[3],
            kind=kind,
            state=fields[5],
            progress=fields[6] / 1000.0,
            offset_rad=fields[7] / off_scale,
            residual_rad=fields[8] / res_scale,
            sign=fields[9],
            ok=bool(fields[10]),
            samples=fields[11],
        )
    if typ == TYPE_CONF and len(data) >= 8:
        op, group, flags = struct.unpack_from("<BBH", data, 4)
        return ConfReply(
            seq=seq,
            op=op,
            group=group,
            flags=flags,
            payload=bytes(data[8:]),
        )
    return None
