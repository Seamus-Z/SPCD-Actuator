#include "middleware/communication/can_command_adapter.h"

#include <cstring>

#include "math/foc/pi.h"
#include "protocol/xt_can.h"

namespace middleware::communication
{
namespace
{

constexpr uint32_t kBootRequestId = 0x7Eu;
constexpr char kBootRequestPayload[] = "BOOT";

float MilliToAmps(int32_t mA)
{
  return static_cast<float>(mA) * 0.001f;
}

bool ReadI32(const uint8_t* p, size_t len, size_t off, int32_t* out)
{
  if (out == nullptr || p == nullptr || off + 4 > len)
  {
    return false;
  }
  std::memcpy(out, p + off, 4);
  return true;
}

bool NotReadI32(const uint8_t* p, size_t len, size_t off, int32_t* out)
{
  return ReadI32(p, len, off, out) == false;
}

uint8_t ToProtocolStatus(middleware::control::Result result)
{
  switch (result)
  {
    case middleware::control::Result::Ok:
      return protocol::xt_can::kStatusOk;
    case middleware::control::Result::NotReady:
      return protocol::xt_can::kStatusNotRun;
    case middleware::control::Result::InvalidArgument:
      return protocol::xt_can::kStatusBadCmd;
    case middleware::control::Result::Failed:
      return protocol::xt_can::kStatusFail;
  }
  return protocol::xt_can::kStatusFail;
}

}  // namespace

CanCommandAdapter::CanCommandAdapter(const Dependencies& dependencies)
    : can_(dependencies.can),
      link_(dependencies.link),
      motor_control_(dependencies.motor_control),
      encoder_(dependencies.encoder),
      calibration_(dependencies.calibration),
      snapshot_(dependencies.snapshot),
      runtime_config_(dependencies.runtime_config),
      telemetry_(dependencies.telemetry),
      motor_name_(dependencies.motor_name),
      nominal_pwm_hz_(dependencies.nominal_pwm_hz)
{
  if (link_ != nullptr)
  {
    link_->SetCommandHandler(&CanCommandAdapter::CommandThunk, this);
  }
}

app::CommandPollResult CanCommandAdapter::Poll(
    const app::CommandPollContext& context)
{
  control_allowed_ = context.control_allowed;
  driver_fault_ = context.driver_fault;
  app::CommandPollResult result{};
  if (can_ == nullptr || link_ == nullptr)
  {
    return result;
  }

  const auto status = can_->status();
  if (status.BusOff)
  {
    can_->RecoverBusOff();
    return result;
  }

  for (int i = 0; i < 8; ++i)
  {
    FDCAN_RxHeaderTypeDef header{};
    uint8_t data[64]{};
    size_t size = 0;
    if (!can_->Poll(&header, data, sizeof(data), &size))
    {
      break;
    }
    if (header.IdType == FDCAN_STANDARD_ID &&
        header.Identifier == kBootRequestId &&
        header.RxFrameType == FDCAN_DATA_FRAME &&
        size == sizeof(kBootRequestPayload) - 1u &&
        std::memcmp(data, kBootRequestPayload,
                    sizeof(kBootRequestPayload) - 1u) == 0)
    {
      result.received_frame = true;
      result.bootloader_requested = true;
      return result;
    }
    if (link_->HandleFrame(header, data, size))
    {
      result.received_frame = true;
    }
  }
  return result;
}

uint8_t CanCommandAdapter::StartServo(
    const math::servo_mode::ServoMode::Command& command)
{
  return control_allowed_
             ? ToProtocolStatus(motor_control_->StartServo(command))
             : protocol::xt_can::kStatusNotRun;
}

uint8_t CanCommandAdapter::StartCurrent(float id_A, float iq_A)
{
  return control_allowed_
             ? ToProtocolStatus(motor_control_->StartCurrent(id_A, iq_A))
             : protocol::xt_can::kStatusNotRun;
}

uint8_t CanCommandAdapter::StartMit(
    const math::servo_mode::MitMode::Command& command)
{
  return control_allowed_
             ? ToProtocolStatus(motor_control_->StartMit(command))
             : protocol::xt_can::kStatusNotRun;
}

uint8_t CanCommandAdapter::CommandThunk(void* context, uint8_t cmd, uint8_t seq,
                              const uint8_t* payload, size_t payload_len)
{
  if (context == nullptr)
  {
    return protocol::xt_can::kStatusFail;
  }
  return static_cast<CanCommandAdapter*>(context)->HandleCommand(cmd, seq, payload,
                                                              payload_len);
}

uint8_t CanCommandAdapter::HandleCommand(uint8_t cmd, uint8_t seq, const uint8_t* payload,
                               size_t payload_len)
{
  uint8_t status = protocol::xt_can::kStatusBadCmd;
  switch (cmd)
  {
    case protocol::xt_can::kCmdStop:
      status = HandleStop();
      break;
    case protocol::xt_can::kCmdQuery:
      status = HandleQuery();
      break;
    case protocol::xt_can::kCmdServo:
      status = HandleServo(payload, payload_len);
      break;
    case protocol::xt_can::kCmdCal:
      status = HandleCalibration(payload, payload_len);
      break;
    case protocol::xt_can::kCmdInfo:
      status = HandleInfo(seq);
      break;
    case protocol::xt_can::kCmdSnap:
      status = HandleSnapshot(seq, payload, payload_len);
      break;
    case protocol::xt_can::kCmdEncComp:
      status = HandleEncoderCompensation(payload, payload_len);
      break;
    case protocol::xt_can::kCmdConf:
      status = HandleConfiguration(seq, payload, payload_len);
      break;
    default:
      status = protocol::xt_can::kStatusBadCmd;
      break;
  }
  if (protocol::xt_can::UsesCtrlReply(cmd))
  {
    telemetry_->ReplyControl(cmd, seq, status, driver_fault_);
  }
  return status;
}

uint8_t CanCommandAdapter::HandleStop()
{
  motor_control_->Stop();
  return protocol::xt_can::kStatusOk;
}

uint8_t CanCommandAdapter::HandleQuery()
{
  // Idle poll / keep-alive while streaming. Encoder sampled in ReplyCtrl.
  return protocol::xt_can::kStatusOk;
}

uint8_t CanCommandAdapter::HandleServo(const uint8_t* payload, size_t payload_len)
{
  auto MilliScale = [](uint16_t milli) -> float {
    return (milli == 0) ? 1.0f : (static_cast<float>(milli) * 0.001f);
  };
  auto DecodeMaybeRad = [](int32_t mrad) -> float {
    return (mrad == protocol::xt_can::kPositionNanMrad)
               ? math::foc::QuietNan()
               : static_cast<float>(mrad) * 0.001f;
  };

  // Legacy 8-byte payload: velocity_mrad_s + id_mA → velocity mode.
  if (payload_len == 8)
  {
    int32_t velocity_mrad_s = 0;
    int32_t id_mA = 0;
    if (NotReadI32(payload, payload_len, 0, &velocity_mrad_s) ||
        NotReadI32(payload, payload_len, 4, &id_mA))
    {
      return protocol::xt_can::kStatusBadLen;
    }
    math::servo_mode::ServoMode::Command command{};
    command.position_rad = math::foc::QuietNan();
    command.velocity_rad_s = static_cast<float>(velocity_mrad_s) * 0.001f;
    command.id_ref_A = MilliToAmps(id_mA);
    return this->StartServo(command);
  }

  // Legacy 20-byte ServoRequest (control + pad3 + pos/vel/id/iq).
  if (payload_len == 20)
  {
    if (payload == nullptr)
    {
      return protocol::xt_can::kStatusBadLen;
    }
    const uint8_t control = payload[0];
    int32_t position_mrad = 0;
    int32_t velocity_mrad_s = 0;
    int32_t id_mA = 0;
    int32_t iq_mA = 0;
    if (NotReadI32(payload, payload_len, 4, &position_mrad) ||
        NotReadI32(payload, payload_len, 8, &velocity_mrad_s) ||
        NotReadI32(payload, payload_len, 12, &id_mA) ||
        NotReadI32(payload, payload_len, 16, &iq_mA))
    {
      return protocol::xt_can::kStatusBadLen;
    }
    if (control == protocol::xt_can::kServoCtrlCurrent)
    {
      return this->StartCurrent(MilliToAmps(id_mA), MilliToAmps(iq_mA));
    }
    if (control != protocol::xt_can::kServoCtrlPosition)
    {
      return protocol::xt_can::kStatusBadCmd;
    }
    math::servo_mode::ServoMode::Command command{};
    command.position_rad = DecodeMaybeRad(position_mrad);
    command.velocity_rad_s = static_cast<float>(velocity_mrad_s) * 0.001f;
    command.id_ref_A = MilliToAmps(id_mA);
    return this->StartServo(command);
  }

  if (payload == nullptr ||
      payload_len < sizeof(protocol::xt_can::ServoRequest))
  {
    return protocol::xt_can::kStatusBadLen;
  }

  protocol::xt_can::ServoRequest req{};
  std::memcpy(&req, payload, sizeof(req));
  if (req.control == protocol::xt_can::kServoCtrlCurrent)
  {
    return this->StartCurrent(MilliToAmps(req.id_mA), MilliToAmps(req.iq_mA));
  }
  if (req.control == protocol::xt_can::kServoCtrlMit)
  {
    math::servo_mode::MitMode::Command command{};
    // Multi-turn continuous: position_mrad is absolute, never NaN-mapped here.
    command.position_rad = static_cast<float>(req.position_mrad) * 0.001f;
    command.velocity_rad_s = static_cast<float>(req.velocity_mrad_s) * 0.001f;
    // id_mA/iq_mA reused as Kp/Kd in milli-Nm units.
    command.kp_nm_per_rad = static_cast<float>(req.id_mA) * 0.001f;
    command.kd_nm_s_per_rad = static_cast<float>(req.iq_mA) * 0.001f;
    command.feedforward_Nm = static_cast<float>(req.feedforward_mNm) * 0.001f;
    command.max_torque_Nm = (req.max_torque_mNm <= 0)
                                ? math::foc::QuietNan()
                                : static_cast<float>(req.max_torque_mNm) * 0.001f;
    return this->StartMit(command);
  }
  if (req.control != protocol::xt_can::kServoCtrlPosition)
  {
    return protocol::xt_can::kStatusBadCmd;
  }

  math::servo_mode::ServoMode::Command command{};
  command.position_rad = DecodeMaybeRad(req.position_mrad);
  command.velocity_rad_s = static_cast<float>(req.velocity_mrad_s) * 0.001f;
  command.stop_position_rad = DecodeMaybeRad(req.stop_position_mrad);
  command.max_torque_Nm = (req.max_torque_mNm <= 0)
                              ? math::foc::QuietNan()
                              : static_cast<float>(req.max_torque_mNm) * 0.001f;
  command.feedforward_Nm = static_cast<float>(req.feedforward_mNm) * 0.001f;
  command.velocity_limit_rad_s = DecodeMaybeRad(req.velocity_limit_mrad_s);
  command.accel_limit_rad_s2 = DecodeMaybeRad(req.accel_limit_mrad_s2);
  command.kp_scale = MilliScale(req.kp_scale_milli);
  command.kd_scale = MilliScale(req.kd_scale_milli);
  command.ilimit_scale = MilliScale(req.ilimit_scale_milli);
  command.id_ref_A = MilliToAmps(req.id_mA);
  return this->StartServo(command);
}

uint8_t CanCommandAdapter::HandleEncoderCompensation(const uint8_t* payload,
                                        size_t payload_len)
{
  if (!control_allowed_)
  {
    return protocol::xt_can::kStatusNotRun;
  }
  if (payload == nullptr ||
      payload_len < sizeof(protocol::xt_can::EncCompRequest))
  {
    return protocol::xt_can::kStatusBadLen;
  }
  protocol::xt_can::EncCompRequest req{};
  std::memcpy(&req, payload, sizeof(req));
  if (req.op == protocol::xt_can::kEncCompOpClear)
  {
    calibration_->ClearCompensation();
    return protocol::xt_can::kStatusOk;
  }
  if (req.op == protocol::xt_can::kEncCompOpChunk)
  {
    if (req.chunk >= 8u)
    {
      return protocol::xt_can::kStatusBadCmd;
    }
    if (!calibration_->SetCompensationChunk(req.chunk, req.data,
                                               sizeof(req.data)))
    {
      return protocol::xt_can::kStatusBadCmd;
    }
    return protocol::xt_can::kStatusOk;
  }
  if (req.op == protocol::xt_can::kEncCompOpCommit)
  {
    const float peak_rad = static_cast<float>(req.scale_urad) * 1.0e-6f;
    if (peak_rad <= 0.0f)
    {
      return protocol::xt_can::kStatusBadCmd;
    }
    // Host sends peak |correction| [rad]; store rad-per-LSB = peak/127.
    const float scale = peak_rad / 127.0f;
    if (!calibration_->CommitCompensation(scale))
    {
      return protocol::xt_can::kStatusFail;
    }
    return protocol::xt_can::kStatusOk;
  }
  return protocol::xt_can::kStatusBadCmd;
}

uint8_t CanCommandAdapter::HandleCalibration(const uint8_t* payload, size_t payload_len)
{
  if (!control_allowed_)
  {
    return protocol::xt_can::kStatusNotRun;
  }
  if (payload == nullptr ||
      payload_len < sizeof(protocol::xt_can::CalRequest))
  {
    return protocol::xt_can::kStatusBadLen;
  }

  protocol::xt_can::CalRequest request{};
  std::memcpy(&request, payload, sizeof(request));
  middleware::control::CalibrationCommand command{};
  const float rate = static_cast<float>(request.omega_elec_mrad_s) * 0.001f;
  switch (request.subcmd)
  {
    case protocol::xt_can::kCalSubAbort:
      command.kind = middleware::control::CalibrationKind::Abort;
      break;
    case protocol::xt_can::kCalSubEncPhase:
    case protocol::xt_can::kCalSubEncLock:
      command.kind = request.subcmd == protocol::xt_can::kCalSubEncLock
                         ? middleware::control::CalibrationKind::EncoderLock
                         : middleware::control::CalibrationKind::EncoderPhase;
      command.encoder_current_A =
          static_cast<float>(request.voltage_mV) * 0.001f;
      command.encoder_electrical_speed_rad_s = rate;
      command.pole_pairs = runtime_config_->config().motor.pole_pairs;
      break;
    case protocol::xt_can::kCalSubBemf:
      command.kind = middleware::control::CalibrationKind::Bemf;
      command.bemf_max_speed_rad_s = rate;
      if (request.voltage_mV > 0)
      {
        command.bemf_points = static_cast<uint8_t>(
            request.voltage_mV > 255 ? 255 : request.voltage_mV);
      }
      break;
    case protocol::xt_can::kCalSubResistance:
      command.kind = middleware::control::CalibrationKind::Resistance;
      command.resistance_max_current_A = rate;
      if (request.voltage_mV > 0)
      {
        command.resistance_points = static_cast<uint8_t>(
            request.voltage_mV > 255 ? 255 : request.voltage_mV);
      }
      break;
    case protocol::xt_can::kCalSubInductance:
      command.kind = middleware::control::CalibrationKind::Inductance;
      command.inductance_step_voltage_V =
          static_cast<float>(request.voltage_mV) * 0.001f;
      if (request.omega_elec_mrad_s > 0)
      {
        command.inductance_trials = static_cast<uint8_t>(
            request.omega_elec_mrad_s > 255 ? 255
                                            : request.omega_elec_mrad_s);
      }
      break;
    case protocol::xt_can::kCalSubCogging:
      command.kind = middleware::control::CalibrationKind::Cogging;
      command.cogging_velocity_rad_s = rate;
      command.cogging_record_revs =
          static_cast<float>(request.voltage_mV) * 0.01f;
      break;
    default:
      return protocol::xt_can::kStatusBadCmd;
  }
  return ToProtocolStatus(motor_control_->StartCalibration(command));
}

bool CanCommandAdapter::SendConfigurationGroup(uint8_t seq, uint8_t op, uint8_t group)
{
  const auto& config = runtime_config_->config();
  uint8_t buf[8 + 48]{};
  protocol::xt_can::ConfReply* reply =
      reinterpret_cast<protocol::xt_can::ConfReply*>(buf);
  reply->hdr.magic = protocol::xt_can::kMagic;
  reply->hdr.ver = protocol::xt_can::kVersion;
  reply->hdr.type = protocol::xt_can::kTypeConf;
  reply->hdr.seq = seq;
  reply->op = op;
  reply->group = group;
  reply->flags = runtime_config_->flash_valid()
                     ? protocol::xt_can::kConfFlagFlashValid
                     : 0;
  uint8_t* payload = buf + sizeof(protocol::xt_can::ConfReply);
  size_t payload_len = 0;
  switch (group)
  {
    case protocol::xt_can::kConfGroupMotor:
      std::memcpy(payload, &config.motor, sizeof(middleware::config::MotorConf));
      payload_len = sizeof(middleware::config::MotorConf);
      break;
    case protocol::xt_can::kConfGroupFoc:
      std::memcpy(payload, &config.foc, sizeof(middleware::config::FocConf));
      payload_len = sizeof(middleware::config::FocConf);
      break;
    case protocol::xt_can::kConfGroupServo:
      std::memcpy(payload, &config.servo, sizeof(middleware::config::ServoConf));
      payload_len = sizeof(middleware::config::ServoConf);
      break;
    case protocol::xt_can::kConfGroupEncoder:
      std::memcpy(payload, &config.encoder,
                  sizeof(middleware::config::EncoderConf));
      payload_len = sizeof(middleware::config::EncoderConf);
      break;
    case protocol::xt_can::kConfGroupCal:
    {
      protocol::xt_can::CalStatusConf cal{};
      uint32_t flags = 0;
      if (calibration_->encoder_persisted())
      {
        flags |= protocol::xt_can::kCalFlagEncoder;
      }
      if (calibration_->resistance_persisted())
      {
        flags |= protocol::xt_can::kCalFlagResistance;
      }
      if (calibration_->inductance_persisted())
      {
        flags |= protocol::xt_can::kCalFlagInductance;
      }
      if (calibration_->bemf_persisted())
      {
        flags |= protocol::xt_can::kCalFlagBemf;
      }
      if (calibration_->cogging_persisted())
      {
        flags |= protocol::xt_can::kCalFlagCogging;
      }
      if (calibration_->compensation_persisted())
      {
        flags |= protocol::xt_can::kCalFlagEncComp;
      }
      cal.flags = flags;
      cal.resistance_ohm = math::foc::QuietNan();
      cal.inductance_d_H = math::foc::QuietNan();
      cal.inductance_q_H = math::foc::QuietNan();
      cal.bemf_v_per_hz = math::foc::QuietNan();
      if (motor_control_->foc() != nullptr)
      {
        if (calibration_->resistance_persisted())
        {
          cal.resistance_ohm = motor_control_->foc()->resistance_ohm();
        }
        if (calibration_->inductance_persisted())
        {
          cal.inductance_d_H = motor_control_->foc()->inductance_d_H();
          cal.inductance_q_H = motor_control_->foc()->inductance_q_H();
        }
        if (calibration_->bemf_persisted())
        {
          cal.bemf_v_per_hz = motor_control_->foc()->options().v_per_hz;
        }
      }
      std::memcpy(payload, &cal, sizeof(cal));
      payload_len = sizeof(cal);
      break;
    }
    default:
      return false;
  }
  return telemetry_->SendConfig(buf, sizeof(protocol::xt_can::ConfReply) +
                                               payload_len);
}

uint8_t CanCommandAdapter::HandleConfiguration(uint8_t seq, const uint8_t* payload,
                                size_t payload_len)
{
  if (payload == nullptr ||
      payload_len < sizeof(protocol::xt_can::ConfRequest))
  {
    return protocol::xt_can::kStatusBadLen;
  }
  protocol::xt_can::ConfRequest req{};
  std::memcpy(&req, payload, sizeof(req));
  const uint8_t* body = payload + sizeof(req);
  const size_t body_len = payload_len - sizeof(req);
  auto& mutable_config = runtime_config_->mutable_config();

  auto GroupSize = [](uint8_t group) -> size_t {
    switch (group)
    {
      case protocol::xt_can::kConfGroupMotor:
        return sizeof(middleware::config::MotorConf);
      case protocol::xt_can::kConfGroupFoc:
        return sizeof(middleware::config::FocConf);
      case protocol::xt_can::kConfGroupServo:
        return sizeof(middleware::config::ServoConf);
      case protocol::xt_can::kConfGroupEncoder:
        return sizeof(middleware::config::EncoderConf);
      case protocol::xt_can::kConfGroupCal:
        return sizeof(protocol::xt_can::CalStatusConf);
      default:
        return 0;
    }
  };

  switch (req.op)
  {
    case protocol::xt_can::kConfOpGet:
    {
      if (req.group == protocol::xt_can::kConfGroupAll ||
          GroupSize(req.group) == 0)
      {
        return protocol::xt_can::kStatusBadCmd;
      }
      if (!SendConfigurationGroup(seq, req.op, req.group))
      {
        return protocol::xt_can::kStatusFail;
      }
      return protocol::xt_can::kStatusOk;
    }
    case protocol::xt_can::kConfOpSet:
    {
      if (req.group == protocol::xt_can::kConfGroupCal)
      {
        return protocol::xt_can::kStatusBadCmd;  // cal is read-only here
      }
      const size_t need = GroupSize(req.group);
      if (need == 0 || body_len < need)
      {
        return protocol::xt_can::kStatusBadLen;
      }
      switch (req.group)
      {
        case protocol::xt_can::kConfGroupMotor:
          std::memcpy(&mutable_config.motor, body, need);
          break;
        case protocol::xt_can::kConfGroupFoc:
          std::memcpy(&mutable_config.foc, body, need);
          break;
        case protocol::xt_can::kConfGroupServo:
          std::memcpy(&mutable_config.servo, body, need);
          break;
        case protocol::xt_can::kConfGroupEncoder:
          std::memcpy(&mutable_config.encoder, body, need);
          break;
        default:
          return protocol::xt_can::kStatusBadCmd;
      }
      runtime_config_->Apply();
      // Persistent calibration remains authoritative for calibrated values.
      (void)calibration_->LoadPersistentConfig();
      if (!SendConfigurationGroup(seq, req.op, req.group))
      {
        return protocol::xt_can::kStatusFail;
      }
      return protocol::xt_can::kStatusOk;
    }
    case protocol::xt_can::kConfOpSave:
    {
      if (req.group != protocol::xt_can::kConfGroupAll &&
          GroupSize(req.group) == 0)
      {
        return protocol::xt_can::kStatusBadCmd;
      }
      return runtime_config_->Save() ? protocol::xt_can::kStatusOk
                                    : protocol::xt_can::kStatusFail;
    }
    case protocol::xt_can::kConfOpLoad:
    {
      return runtime_config_->Load() ? protocol::xt_can::kStatusOk
                                    : protocol::xt_can::kStatusFail;
    }
    case protocol::xt_can::kConfOpDefaults:
    {
      runtime_config_->RestoreDefaults();
      return protocol::xt_can::kStatusOk;
    }
    default:
      return protocol::xt_can::kStatusBadCmd;
  }
}

uint8_t CanCommandAdapter::HandleInfo(uint8_t seq)
{
  const auto& motor = runtime_config_->config().motor;

  protocol::xt_can::Info info{};
  info.hdr.seq = seq;
  info.node_id = link_->node_id();
  info.fw_major = protocol::xt_can::kFwMajor;
  info.fw_minor = protocol::xt_can::kFwMinor;
  info.fw_patch = protocol::xt_can::kFwPatch;
  info.pwm_hz = nominal_pwm_hz_;
  info.bus_mV =
      static_cast<uint16_t>(motor.bus_V * 1000.0f + 0.5f);
  info.i_max_mA =
      static_cast<uint16_t>(motor.max_phase_current_A * 1000.0f + 0.5f);
  info.pole_pairs = static_cast<uint16_t>(motor.pole_pairs + 0.5f);
  info.r_mohm =
      static_cast<uint16_t>(motor.resistance_ohm * 1000.0f + 0.5f);
  info.l_uH =
      static_cast<uint16_t>(motor.inductance_H * 1.0e6f + 0.5f);
  info.family = 3;  // moteus-x1 / family 3
  std::memset(info.motor, 0, sizeof(info.motor));
  std::strncpy(info.motor, motor_name_ != nullptr ? motor_name_ : "",
               sizeof(info.motor) - 1);

  telemetry_->SendInfo(info);
  return protocol::xt_can::kStatusOk;
}

uint8_t CanCommandAdapter::HandleSnapshot(uint8_t seq, const uint8_t* payload,
                                   size_t payload_len)
{
  if (!motor_control_->isr_enabled())
  {
    return protocol::xt_can::kStatusNotRun;
  }
  if (snapshot_->busy())
  {
    return protocol::xt_can::kStatusFail;
  }

  uint16_t n_samples = protocol::xt_can::kSnapMaxSamples;
  uint8_t decimate = 1;
  if (payload != nullptr &&
      payload_len >= sizeof(protocol::xt_can::SnapRequest))
  {
    protocol::xt_can::SnapRequest req{};
    std::memcpy(&req, payload, sizeof(req));
    n_samples = req.n_samples;
    decimate = req.decimate;
  }
  else if (payload_len != 0 &&
           payload_len < sizeof(protocol::xt_can::SnapRequest))
  {
    return protocol::xt_can::kStatusBadLen;
  }

  const uint16_t pwm_hz =
      nominal_pwm_hz_;
  if (!snapshot_->Arm(seq, n_samples, decimate, pwm_hz))
  {
    return protocol::xt_can::kStatusFail;
  }
  return protocol::xt_can::kStatusOk;
}

}  // namespace middleware::communication
