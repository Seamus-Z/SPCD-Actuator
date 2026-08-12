// GUI / Monitor telemetry protocol (host <-> APP). Little-endian, packed.
// Lives under telemetry/; fw/protocol/* are thin shims for include jump.
// Future 8-byte real-time control protocol will live under fw/control/.
#pragma once

#include <cstdint>

namespace telemetry
{
namespace xt_can
{

inline constexpr uint8_t kMagic = 0x58;  // 'X'
inline constexpr uint8_t kVersion = 1;

inline constexpr uint8_t kTypeCmd = 1;
inline constexpr uint8_t kTypeTel = 2;
inline constexpr uint8_t kTypeAck = 3;
inline constexpr uint8_t kTypeInfo = 4;
inline constexpr uint8_t kTypeSnapMeta = 5;
inline constexpr uint8_t kTypeSnapData = 6;
inline constexpr uint8_t kTypeEnc = 7;
inline constexpr uint8_t kTypeCal = 8;
// moteus-style control reply (cmd/query → one merged Live frame).
inline constexpr uint8_t kTypeCtrlReply = 9;
// Reply payload for kCmdConf GET (and optional SET echo).
inline constexpr uint8_t kTypeConf = 10;

inline constexpr uint8_t kCmdStop = 0;
inline constexpr uint8_t kCmdInfo = 4;
inline constexpr uint8_t kCmdSnap = 5;
// Servo command: extended payload selects position/velocity or current.
// See ServoRequest. FOC voltage/PWM modes remain internal (cal only).
inline constexpr uint8_t kCmdServo = 6;
// Calibration: subcmd + args (see CalRequest).
inline constexpr uint8_t kCmdCal = 7;
// Idle poll: no mode change, reply with CtrlReply.
inline constexpr uint8_t kCmdQuery = 8;
// Upload moteus-style encoder geometric compensation table.
inline constexpr uint8_t kCmdEncComp = 9;
// Runtime config get/set/save (motor/foc/servo/encoder).
inline constexpr uint8_t kCmdConf = 10;

// APP firmware semver reported by kCmdInfo (bump when shipping).
inline constexpr uint8_t kFwMajor = 0;
inline constexpr uint8_t kFwMinor = 6;
inline constexpr uint8_t kFwPatch = 5;

// Commands that answer with CtrlReply instead of ACK.
inline constexpr bool UsesCtrlReply(uint8_t cmd)
{
  return cmd == kCmdStop || cmd == kCmdServo || cmd == kCmdQuery;
}

inline constexpr uint8_t kCalSubAbort = 0;
inline constexpr uint8_t kCalSubEncPhase = 1;  // bidirectional 64-bin mapping
inline constexpr uint8_t kCalSubEncLock = 2;   // coarse global offset only
// Encoder lock/spin reuse CalRequest voltage_mV as alignment current [mA].
// The field name is retained to preserve the packed wire ABI.
// Closed-loop Ke (back-EMF constant) identification: velocity sweep +
// Vq/Iq regression. Requires encoder phase already calibrated. CalRequest
// reuses omega_elec_mrad_s as the sweep's top mech speed [mrad/s];
// voltage_mV as sweep point count (0 => firmware default, else 3..8).
// CalTelem reuses offset_mrad for Ke*1e6 [µV·s/rad] and residual_mrad
// for fit r²*1e6 (see BuildCalTelem()).
inline constexpr uint8_t kCalSubBemf = 3;
// Closed-loop R (phase resistance) identification: Id sweep at fixed
// theta=0, no encoder needed. CalRequest reuses omega_elec_mrad_s as the
// sweep's top Id current [mA]; voltage_mV as sweep point count (0 =>
// firmware default, else 3..8). CalTelem reuses offset_mrad for R*1e6
// [µΩ] and residual_mrad for fit r²*1e6.
inline constexpr uint8_t kCalSubResistance = 4;
// Locked-rotor D/Q inductance identification. Requires R already known.
// CalRequest reuses voltage_mV as the step voltage [mV] (0 => firmware
// default); omega_elec_mrad_s as per-axis trial count (0 => firmware default,
// else 3..8). CalTelem reuses offset_mrad for Ld*1e9 [nH] and
// residual_mrad for Lq*1e9 [nH].
inline constexpr uint8_t kCalSubInductance = 5;
// Cogging-torque compensation measurement: slow constant-velocity forward +
// reverse spin, records the torque current per rotor position and stores a
// moteus-style int8 feed-forward table. Requires encoder phase already
// calibrated. CalRequest reuses omega_elec_mrad_s as the sweep speed
// [mech mrad/s] (0 => firmware default); voltage_mV as record revolutions
// x100 (0 => firmware default). CalTelem reuses offset_mrad for the table
// scale (peak A) *1e6 and residual_mrad for peak current *1e6.
inline constexpr uint8_t kCalSubCogging = 6;

// kCmdEncComp payload ops (after cmd byte).
inline constexpr uint8_t kEncCompOpClear = 0;   // disable + zero table, persist
inline constexpr uint8_t kEncCompOpChunk = 1;   // write 32-byte chunk
inline constexpr uint8_t kEncCompOpCommit = 2;  // enable with scale, persist

// kCmdConf ops / groups.
inline constexpr uint8_t kConfOpGet = 0;
inline constexpr uint8_t kConfOpSet = 1;       // RAM only
inline constexpr uint8_t kConfOpSave = 2;      // RAM -> flash
inline constexpr uint8_t kConfOpLoad = 3;      // flash -> RAM -> apply
inline constexpr uint8_t kConfOpDefaults = 4;  // board defaults -> RAM -> apply
inline constexpr uint8_t kConfGroupMotor = 0;
inline constexpr uint8_t kConfGroupFoc = 1;
inline constexpr uint8_t kConfGroupServo = 2;
inline constexpr uint8_t kConfGroupEncoder = 3;
inline constexpr uint8_t kConfGroupCal = 4;  // read-only calibration flash status
inline constexpr uint8_t kConfGroupAll = 0xFF;
inline constexpr uint32_t kCalFlagEncoder = 1u << 0;
inline constexpr uint32_t kCalFlagResistance = 1u << 1;
inline constexpr uint32_t kCalFlagInductance = 1u << 2;
inline constexpr uint32_t kCalFlagBemf = 1u << 3;
inline constexpr uint32_t kCalFlagCogging = 1u << 4;
inline constexpr uint32_t kCalFlagEncComp = 1u << 5;
inline constexpr uint16_t kConfFlagFlashValid = 1u << 0;

inline constexpr uint8_t kStatusOk = 0;
inline constexpr uint8_t kStatusBadLen = 1;
inline constexpr uint8_t kStatusBadCmd = 2;
inline constexpr uint8_t kStatusNotRun = 3;
inline constexpr uint8_t kStatusFail = 4;

// Standard 11-bit IDs (node_id default = 1).
inline constexpr uint16_t CmdId(uint8_t node_id)
{
  return static_cast<uint16_t>(0x100u + node_id);
}
inline constexpr uint16_t TelId(uint8_t node_id)
{
  return static_cast<uint16_t>(0x180u + node_id);
}

inline constexpr uint16_t kFlagPwmOn = 1u << 0;
inline constexpr uint16_t kFlagCisr = 1u << 1;
inline constexpr uint16_t kFlagDqValid = 1u << 2;
inline constexpr uint16_t kFlagFault = 1u << 3;
inline constexpr uint16_t kFlagEncOk = 1u << 4;
inline constexpr uint16_t kFlagEncMode = 1u << 5;

inline constexpr uint8_t kModeStop = 0;
inline constexpr uint8_t kModeServo = 1;     // position or velocity (outer PID)
inline constexpr uint8_t kModeCal = 2;
inline constexpr uint8_t kModeCurrent = 3;   // direct Id/Iq current mode

// kCmdServo payload.control
inline constexpr uint8_t kServoCtrlPosition = 0;  // position/velocity PID → Iq
inline constexpr uint8_t kServoCtrlCurrent = 1;   // direct Id/Iq
// position_mrad sentinel: velocity tracking (moteus position=NaN).
inline constexpr int32_t kPositionNanMrad = static_cast<int32_t>(0x80000000u);

inline constexpr uint8_t kCalStateIdle = 0;
inline constexpr uint8_t kCalStateSense = 1;
inline constexpr uint8_t kCalStateFwd = 2;
inline constexpr uint8_t kCalStateRev = 3;
inline constexpr uint8_t kCalStateDone = 4;
inline constexpr uint8_t kCalStateFailed = 5;
inline constexpr uint8_t kCalStateLocking = 6;
inline constexpr uint8_t kCalStateBemfRun = 7;  // kCalSubBemf: sweeping/sampling
inline constexpr uint8_t kCalStateRRun = 8;     // kCalSubResistance: sweeping
inline constexpr uint8_t kCalStateLRun = 9;     // kCalSubInductance: stepping
inline constexpr uint8_t kCalStateCoggingRun = 10;  // kCalSubCogging: spinning

// Snapshot (PWM-rate burst). Channels packed as int16 mA / mrad in order:
// Id, Iq, I1, I2, I3, theta_mech_mrad, theta_elec_mrad.
// theta_* are milli-radians (int16 wraps every ~6.28 rad mech; the host
// unwraps by per-channel delta accumulation, same as before for currents).
inline constexpr uint16_t kSnapChId = 1u << 0;
inline constexpr uint16_t kSnapChIq = 1u << 1;
inline constexpr uint16_t kSnapChI1 = 1u << 2;
inline constexpr uint16_t kSnapChI2 = 1u << 3;
inline constexpr uint16_t kSnapChI3 = 1u << 4;
inline constexpr uint16_t kSnapChThetaMech = 1u << 5;
inline constexpr uint16_t kSnapChThetaElec = 1u << 6;
inline constexpr uint16_t kSnapChDefault =
    kSnapChId | kSnapChIq | kSnapChI1 | kSnapChI2 | kSnapChI3 |
    kSnapChThetaMech | kSnapChThetaElec;
inline constexpr uint16_t kSnapMaxSamples = 512;
inline constexpr uint8_t kSnapChannelCount = 7;
// 7 ch * 2 B = 14 B/sample; 4 samples/frame = 56 B fits a 64 B FD frame.
inline constexpr uint8_t kSnapSamplesPerFrame = 4;

#pragma pack(push, 1)

struct Header
{
  uint8_t magic;
  uint8_t ver;
  uint8_t type;
  uint8_t seq;
};

struct Ack
{
  Header hdr;
  uint8_t cmd;
  uint8_t status;
  uint16_t reserved;
};

// Response to kCmdInfo (sent on TelId before ACK).
struct Info
{
  Header hdr;
  uint8_t node_id;
  uint8_t fw_major;
  uint8_t fw_minor;
  uint8_t fw_patch;
  uint16_t pwm_hz;
  uint16_t bus_mV;       // configured nominal bus
  uint16_t i_max_mA;     // soft current-loop cap
  uint16_t pole_pairs;
  uint16_t r_mohm;       // phase R
  uint16_t l_uH;         // phase L
  uint16_t family;       // board family (moteus-x1 = 3)
  char motor[12];        // e.g. "DM4310"
} __attribute__((packed));

struct Telemetry
{
  Header hdr;
  uint16_t flags;
  int32_t id_mA;
  int32_t iq_mA;
  int32_t idref_mA;
  int32_t iqref_mA;
  int32_t i1_mA;
  int32_t i2_mA;
  int32_t i3_mA;
  int32_t theta_mrad;
  int32_t omega_mrad_s;
  int32_t vd_mV;
  int32_t vq_mV;
  uint16_t duty_a;
  uint16_t duty_b;
  uint16_t duty_c;
  uint16_t bus_mV;
  uint8_t mode;
  uint8_t reserved;
} __attribute__((packed));

// Compact encoder status (AUX2 MA600). Sent on TelId alongside Telemetry.
struct EncTelem
{
  Header hdr;
  uint16_t raw;  // 0..65535
  int32_t theta_mech_mrad;
  int32_t theta_elec_mrad;
  int8_t sign;
  uint8_t ok;
  uint8_t reserved[2];
} __attribute__((packed));

// Device -> host reply to Query/Stop/Servo (replaces free-running Tel).
// seq echoes the command seq. Live Id/Iq + encoder in one FD frame.
struct CtrlReply
{
  Header hdr;
  uint8_t cmd;
  uint8_t status;
  uint16_t flags;
  uint8_t mode;
  uint8_t enc_ok;
  int8_t enc_sign;
  // Sticky encoder/PLL glitch count, saturating at 255. Climbs when the PLL
  // sees a large one-sample angle jump or hits its absolute velocity clamp.
  uint8_t enc_spike;
  int32_t id_mA;
  int32_t iq_mA;
  int32_t idref_mA;
  int32_t iqref_mA;
  int32_t theta_elec_mrad;
  int32_t omega_mech_mrad_s;
  int32_t vd_mV;
  int32_t vq_mV;
  uint16_t bus_mV;
  uint16_t enc_raw;
  int32_t theta_mech_mrad;
  int32_t omega_cmd_mrad_s;
  int32_t omega_elec_mrad_s;
  int32_t voltage_headroom_mV;
} __attribute__((packed));

// Host -> device: after cmd byte for kCmdServo.
// moteus-style kPosition fields (plus control selector for current mode).
// control=kServoCtrlPosition:
//   position_mrad = kPositionNanMrad → no absolute target (velocity / stop).
//   Finite position/stop are single-turn [0, 2π); firmware shortest-path maps.
//   velocity_mrad_s = trajectory / hold velocity command.
//   stop_position_mrad = kPositionNanMrad → none; else clamp trajectory.
//   max_torque_mNm = 0 → board default.
//   *_limit_mrad* = kPositionNanMrad → board default; negative → unlimited.
//   *_scale_milli = 0 → 1000 (1.0).
// control=kServoCtrlCurrent:
//   id_mA / iq_mA are the FOC references; position fields ignored.
struct ServoRequest
{
  uint8_t control;     // kServoCtrl*
  uint8_t reserved0;
  uint16_t timeout_ms; // reserved for future position-timeout policy
  int32_t position_mrad;
  int32_t velocity_mrad_s;
  int32_t id_mA;
  int32_t iq_mA;
  int32_t stop_position_mrad;
  int32_t max_torque_mNm;
  int32_t feedforward_mNm;
  int32_t velocity_limit_mrad_s;
  int32_t accel_limit_mrad_s2;
  uint16_t kp_scale_milli;
  uint16_t kd_scale_milli;
  uint16_t ilimit_scale_milli;
  uint16_t reserved1;
} __attribute__((packed));

// Host -> device: after cmd byte for kCmdEncComp.
// Chunk write: op=Chunk, chunk=0..7, data[32]
// Commit: op=Commit, scale_urad = peak|corr| in microradians (scale = peak/127)
// Clear: op=Clear
struct EncCompRequest
{
  uint8_t op;
  uint8_t chunk;
  uint8_t reserved[2];
  int32_t scale_urad;
  int8_t data[32];
} __attribute__((packed));

// Host -> device: after cmd byte for kCmdCal.
struct CalRequest
{
  uint8_t subcmd;        // kCalSub*
  uint8_t reserved;
  int32_t voltage_mV;    // subcommand-dependent value; see kCalSub* comments
  int32_t omega_elec_mrad_s;
} __attribute__((packed));

// Device -> host calibration status / result.
struct CalTelem
{
  Header hdr;
  uint8_t kind;          // kCalSub*
  uint8_t state;         // kCalState*
  uint16_t progress_pm;  // 0..1000
  int32_t offset_mrad;   // mechanical
  int32_t residual_mrad;
  int8_t sign;
  uint8_t ok;
  uint16_t samples;
} __attribute__((packed));

// Host -> device: after cmd byte.
// n_samples (0 => 512), decimate (>=1), reserved, channel_mask (v1: ignore, use default).
struct SnapRequest
{
  uint16_t n_samples;
  uint8_t decimate;
  uint8_t reserved;
  uint16_t channel_mask;
} __attribute__((packed));

// Device -> host before data frames.
struct SnapMeta
{
  Header hdr;
  uint16_t n_samples;
  uint16_t sample_hz;
  uint16_t channel_mask;
  uint8_t channels;     // bytes per sample / sizeof(int16) count
  uint8_t decimate;
  uint32_t duration_us;
} __attribute__((packed));

// Device -> host sample payload. Fixed v1 layout: 5 x int16 mA.
struct SnapData
{
  Header hdr;
  uint16_t start_index;
  uint8_t n;  // samples in this frame (1..kSnapSamplesPerFrame)
  uint8_t reserved;
  int16_t samples[kSnapSamplesPerFrame * kSnapChannelCount];
} __attribute__((packed));

#pragma pack(pop)

static_assert(sizeof(Header) == 4, "Header size");
static_assert(sizeof(Ack) == 8, "Ack size");
// hdr4 + node/fw4 + 7*u16 + motor12 = 34

// Host <-> device runtime config groups (kCmdConf). Sizes fit one CAN-FD frame
// together with ConfReply header (GET/SET are single-group only).
struct MotorConf
{
  float pole_pairs;
  float resistance_ohm;
  float inductance_H;
  float bemf_Vpeak_per_krpm;
  float max_phase_current_A;
  float fw_speed_rad_s;
  float bus_V;
  float reserved0;
} __attribute__((packed));

struct FocConf
{
  float bandwidth_hz;
  float bemf_feedforward;
  float current_feedforward;
  float cross_coupling_feedforward;
  float max_current_desired_rate_A_s;
  float reserved0;
} __attribute__((packed));

struct ServoConf
{
  float kp;
  float ki;
  float kd;
  float ilimit;
  float max_iq_A;
  float velocity_threshold;
  float max_position_slip_rad;
  float max_velocity_error_rad_s;
  float default_velocity_limit_rad_s;
  float default_accel_limit_rad_s2;  // NaN => unlimited
  float sign_f;                      // -1 or +1
  float reserved0;
} __attribute__((packed));

struct EncoderConf
{
  float pll_filter_hz;
  float spike_error_rad;
  float filter_us;
  float reserved0;
} __attribute__((packed));

// Read-only snapshot of calibration flash validity + effective values.
// Not stored in RuntimeConfigStore; sourced from CalibrationManager / FOC.
struct CalStatusConf
{
  uint32_t flags;          // kCalFlag*
  float resistance_ohm;    // NaN if not persisted
  float inductance_d_H;    // NaN if not persisted
  float inductance_q_H;    // NaN if not persisted
  float bemf_v_per_hz;     // dq Ke; NaN if not persisted
} __attribute__((packed));

// Host -> device: after cmd byte for kCmdConf.
struct ConfRequest
{
  uint8_t op;       // kConfOp*
  uint8_t group;    // kConfGroup*
  uint16_t reserved;
  // Optional group payload follows for SET.
} __attribute__((packed));

// Device -> host: TelId before ACK for GET (optional SET echo).
struct ConfReply
{
  Header hdr;       // type = kTypeConf
  uint8_t op;
  uint8_t group;
  uint16_t flags;   // kConfFlagFlashValid
  // Group payload follows.
} __attribute__((packed));

static_assert(sizeof(Info) == 34, "Info size");
// hdr4 + flags2 + 11*i32 + 3*u16 duty + bus_u16 + mode + reserved = 60
static_assert(sizeof(Telemetry) == 60, "Telemetry size");
// hdr4 + raw2 + mech4 + elec4 + sign1 + ok1 + pad2 = 18
static_assert(sizeof(EncTelem) == 18, "EncTelem size");
// Original 56-byte reply plus command ω, electrical ω, and voltage headroom.
static_assert(sizeof(CtrlReply) == 64, "CtrlReply size");
static_assert(sizeof(ServoRequest) == 48, "ServoRequest size");
static_assert(sizeof(CalRequest) == 10, "CalRequest size");
// hdr4 + kind1 + state1 + pm2 + offset4 + resid4 + sign1 + ok1 + samples2 = 20
static_assert(sizeof(CalTelem) == 20, "CalTelem size");
static_assert(sizeof(SnapRequest) == 6, "SnapRequest size");
static_assert(sizeof(SnapMeta) == 16, "SnapMeta size");
// hdr4 + idx2 + n1 + r1 + 4*7*i16 = 8 + 56 = 64
static_assert(sizeof(SnapData) == 64, "SnapData size");
static_assert(sizeof(MotorConf) == 32, "MotorConf size");
static_assert(sizeof(FocConf) == 24, "FocConf size");
static_assert(sizeof(ServoConf) == 48, "ServoConf size");
static_assert(sizeof(EncoderConf) == 16, "EncoderConf size");
static_assert(sizeof(CalStatusConf) == 20, "CalStatusConf size");
static_assert(sizeof(ConfRequest) == 4, "ConfRequest size");
static_assert(sizeof(ConfReply) == 8, "ConfReply size");

}  // namespace xt_can
}  // namespace telemetry
