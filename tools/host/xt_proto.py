# xtellar CAN-FD binary protocol (mirrors fw/telemetry/inc/telemetry/xt_can.h).
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

CMD_STOP = 0
CMD_DQ = 1
CMD_VFOC = 2
CMD_RAW = 3
CMD_INFO = 4
CMD_SNAP = 5
CMD_VEL = 6
CMD_CAL = 7
CMD_QUERY = 8

CAL_SUB_ABORT = 0
CAL_SUB_ENC_PHASE = 1
CAL_SUB_ENC_LOCK = 2

STATUS_OK = 0
STATUS_BAD_LEN = 1
STATUS_BAD_CMD = 2
STATUS_NOT_RUN = 3
STATUS_FAIL = 4

SNAP_CH_DEFAULT = 0x1F
SNAP_MAX_SAMPLES = 512
SNAP_CHANNELS = 5
SNAP_SAMPLES_PER_FRAME = 5
SNAP_CHANNEL_KEYS = ["id_a", "iq_a", "i1_a", "i2_a", "i3_a"]

FLAG_PWM_ON = 1 << 0
FLAG_CISR = 1 << 1
FLAG_DQ_VALID = 1 << 2
FLAG_FAULT = 1 << 3
FLAG_ENC_OK = 1 << 4
FLAG_ENC_MODE = 1 << 5

MODE_STOP = 0
MODE_RAW = 1
MODE_VFOC = 2
MODE_DQ = 3
MODE_VEL = 4
MODE_CAL = 5


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


def pack_dq(id_a: float, iq_a: float, omega_rad_s: float, seq: int = 0) -> bytes:
    return pack_header(TYPE_CMD, seq) + struct.pack(
        "<Biii",
        CMD_DQ,
        int(round(id_a * 1000)),
        int(round(iq_a * 1000)),
        int(round(omega_rad_s * 1000)),
    )


def pack_vel(omega_mech_rad_s: float, id_a: float = 0.0, seq: int = 0) -> bytes:
    """moteus-style velocity: position=NaN + ω_mech [rad/s], optional Id."""
    return pack_header(TYPE_CMD, seq) + struct.pack(
        "<Bii",
        CMD_VEL,
        int(round(omega_mech_rad_s * 1000)),
        int(round(id_a * 1000)),
    )



def pack_cal_enc(
    voltage_v: float = 1.5,
    omega_elec_rad_s: float = 40.0,
    seq: int = 0,
) -> bytes:
    """Start encoder spin calibration (both directions)."""
    return pack_header(TYPE_CMD, seq) + struct.pack(
        "<BBBii",
        CMD_CAL,
        CAL_SUB_ENC_PHASE,
        0,
        int(round(voltage_v * 1000)),
        int(round(omega_elec_rad_s * 1000)),
    )


def pack_cal_lock(voltage_v: float = 1.5, seq: int = 0) -> bytes:
    """Start encoder lock calibration (hold Vd, omega=0) — recommended."""
    return pack_header(TYPE_CMD, seq) + struct.pack(
        "<BBBii",
        CMD_CAL,
        CAL_SUB_ENC_LOCK,
        0,
        int(round(voltage_v * 1000)),
        0,
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


def pack_vfoc(theta_rad: float, v: float, omega_rad_s: float, seq: int = 0) -> bytes:
    return pack_header(TYPE_CMD, seq) + struct.pack(
        "<Biii",
        CMD_VFOC,
        int(round(theta_rad * 1000)),
        int(round(v * 1000)),
        int(round(omega_rad_s * 1000)),
    )


def pack_raw(a: int, b: int, c: int, seq: int = 0) -> bytes:
    return pack_header(TYPE_CMD, seq) + struct.pack("<BHHH", CMD_RAW, a, b, c)


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

# hdr4 + cmd/status + flags + mode/enc + 4*i32 I + 4*i32 ang/V + bus/raw + mech + pad = 56
_CTRL_FMT = "<BBBBBBHBBbBiiiiiiiiHHii"
assert struct.calcsize(_CTRL_FMT) == 56


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
        fields = struct.unpack_from(_CTRL_FMT, data, 0)
        return CtrlReply(
            seq=fields[3],
            cmd=fields[4],
            status=fields[5],
            flags=fields[6],
            mode=fields[7],
            enc_ok=bool(fields[8]),
            enc_sign=fields[9],
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
        )
    if typ == TYPE_CAL and len(data) >= 20:
        fields = struct.unpack_from(_CAL_FMT, data, 0)
        return CalTelem(
            seq=fields[3],
            kind=fields[4],
            state=fields[5],
            progress=fields[6] / 1000.0,
            offset_rad=fields[7] / 1000.0,
            residual_rad=fields[8] / 1000.0,
            sign=fields[9],
            ok=bool(fields[10]),
            samples=fields[11],
        )
    return None
