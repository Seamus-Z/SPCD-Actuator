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

inline constexpr uint8_t kCmdStop = 0;
inline constexpr uint8_t kCmdDq = 1;
inline constexpr uint8_t kCmdVfoc = 2;
inline constexpr uint8_t kCmdRaw = 3;
inline constexpr uint8_t kCmdInfo = 4;
inline constexpr uint8_t kCmdSnap = 5;

// APP firmware semver reported by kCmdInfo (bump when shipping).
inline constexpr uint8_t kFwMajor = 0;
inline constexpr uint8_t kFwMinor = 2;
inline constexpr uint8_t kFwPatch = 0;

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

inline constexpr uint8_t kModeStop = 0;
inline constexpr uint8_t kModeRaw = 1;
inline constexpr uint8_t kModeVfoc = 2;
inline constexpr uint8_t kModeDq = 3;

// Snapshot (PWM-rate burst). Channels packed as int16 mA in order:
// Id, Iq, I1, I2, I3 when mask == kSnapChDefault.
inline constexpr uint16_t kSnapChId = 1u << 0;
inline constexpr uint16_t kSnapChIq = 1u << 1;
inline constexpr uint16_t kSnapChI1 = 1u << 2;
inline constexpr uint16_t kSnapChI2 = 1u << 3;
inline constexpr uint16_t kSnapChI3 = 1u << 4;
inline constexpr uint16_t kSnapChDefault =
    kSnapChId | kSnapChIq | kSnapChI1 | kSnapChI2 | kSnapChI3;
inline constexpr uint16_t kSnapMaxSamples = 512;
inline constexpr uint8_t kSnapChannelCount = 5;
inline constexpr uint8_t kSnapSamplesPerFrame = 5;  // 5*5*i16 = 50 B payload

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
static_assert(sizeof(Info) == 34, "Info size");
// hdr4 + flags2 + 11*i32 + 3*u16 duty + bus_u16 + mode + reserved = 60
static_assert(sizeof(Telemetry) == 60, "Telemetry size");
static_assert(sizeof(SnapRequest) == 6, "SnapRequest size");
static_assert(sizeof(SnapMeta) == 16, "SnapMeta size");
// hdr4 + idx2 + n1 + r1 + 5*5*i16 = 8 + 50 = 58
static_assert(sizeof(SnapData) == 58, "SnapData size");

}  // namespace xt_can
}  // namespace telemetry
