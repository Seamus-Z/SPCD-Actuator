#include "binary_commands.h"

#include <cstring>

#include "application.h"
#include "board_config.h"
#include "device/motor.h"
#include "telemetry/xt_can.h"

namespace app
{
namespace
{

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

bool ReadU16(const uint8_t* p, size_t len, size_t off, uint16_t* out)
{
  if (out == nullptr || p == nullptr || off + 2 > len)
  {
    return false;
  }
  std::memcpy(out, p + off, 2);
  return true;
}

}  // namespace

uint8_t BinaryCommands::Thunk(void* context, uint8_t cmd, uint8_t seq,
                              const uint8_t* payload, size_t payload_len)
{
  if (context == nullptr)
  {
    return telemetry::xt_can::kStatusFail;
  }
  return static_cast<BinaryCommands*>(context)->Handle(cmd, seq, payload,
                                                       payload_len);
}

uint8_t BinaryCommands::Handle(uint8_t cmd, uint8_t seq, const uint8_t* payload,
                               size_t payload_len)
{
  switch (cmd)
  {
    case telemetry::xt_can::kCmdStop:
      return HandleStop();
    case telemetry::xt_can::kCmdDq:
      return HandleDq(payload, payload_len);
    case telemetry::xt_can::kCmdVfoc:
      return HandleVfoc(payload, payload_len);
    case telemetry::xt_can::kCmdRaw:
      return HandleRaw(payload, payload_len);
    case telemetry::xt_can::kCmdInfo:
      return HandleInfo(seq);
    case telemetry::xt_can::kCmdSnap:
      return HandleSnap(seq, payload, payload_len);
    default:
      return telemetry::xt_can::kStatusBadCmd;
  }
}

uint8_t BinaryCommands::HandleStop()
{
  app_->StopOutput();
  return telemetry::xt_can::kStatusOk;
}

uint8_t BinaryCommands::HandleDq(const uint8_t* payload, size_t payload_len)
{
  if (app_->state_ != State::RUN || !app_->phase_pwm_->init_ok())
  {
    return telemetry::xt_can::kStatusNotRun;
  }
  int32_t id_mA = 0;
  int32_t iq_mA = 0;
  int32_t omega = 0;
  if (!ReadI32(payload, payload_len, 0, &id_mA) ||
      !ReadI32(payload, payload_len, 4, &iq_mA) ||
      !ReadI32(payload, payload_len, 8, &omega))
  {
    return telemetry::xt_can::kStatusBadLen;
  }

  app_->voltage_foc_->Stop();
  app_->phase_pwm_->DisableControlIsr();

  foc_ctrl::CurrentLoop::Command cmd;
  cmd.theta_rad = 0.0f;
  cmd.id_A = MilliToAmps(id_mA);
  cmd.iq_A = MilliToAmps(iq_mA);
  cmd.theta_rate_rad_s = static_cast<float>(omega) * 0.001f;
  app_->current_loop_->Start(cmd);

  app_->last_current_ = app_->current_adc_->ReadLatest();
  foc_ctrl::CurrentLoop::Duties duties;
  if (!app_->current_loop_->Step(0.0f, app_->last_current_.i1_A,
                                 app_->last_current_.i2_A,
                                 app_->last_current_.i3_A, &duties))
  {
    app_->current_loop_->Stop();
    return telemetry::xt_can::kStatusFail;
  }
  app_->id_A_ = app_->current_loop_->id_A();
  app_->iq_A_ = app_->current_loop_->iq_A();
  app_->dq_valid_ = app_->last_current_.ok;
  app_->phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
  app_->gate_driver_->PowerOn();
  app_->pwm_output_on_ = true;
  app_->mode_ = telemetry::xt_can::kModeDq;
  app_->StartControlIsr();
  return telemetry::xt_can::kStatusOk;
}

uint8_t BinaryCommands::HandleVfoc(const uint8_t* payload, size_t payload_len)
{
  if (app_->state_ != State::RUN || !app_->phase_pwm_->init_ok())
  {
    return telemetry::xt_can::kStatusNotRun;
  }
  int32_t theta_mrad = 0;
  int32_t v_mV = 0;
  int32_t omega = 0;
  if (!ReadI32(payload, payload_len, 0, &theta_mrad) ||
      !ReadI32(payload, payload_len, 4, &v_mV) ||
      !ReadI32(payload, payload_len, 8, &omega))
  {
    return telemetry::xt_can::kStatusBadLen;
  }

  app_->current_loop_->Stop();
  app_->phase_pwm_->DisableControlIsr();

  foc_ctrl::VoltageFoc::Command cmd;
  cmd.theta_rad = static_cast<float>(theta_mrad) * 0.001f;
  cmd.voltage_V = static_cast<float>(v_mV) * 0.001f;
  cmd.theta_rate_rad_s = static_cast<float>(omega) * 0.001f;
  app_->voltage_foc_->Start(cmd);

  foc_ctrl::VoltageFoc::Duties duties;
  if (!app_->voltage_foc_->Step(0.0f, &duties))
  {
    app_->voltage_foc_->Stop();
    return telemetry::xt_can::kStatusFail;
  }
  app_->phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
  app_->gate_driver_->PowerOn();
  app_->pwm_output_on_ = true;
  app_->last_current_ = app_->current_adc_->ReadLatest();
  app_->ObserveDqFromSample(app_->voltage_foc_->theta_rad(),
                             app_->last_current_);
  app_->mode_ = telemetry::xt_can::kModeVfoc;
  app_->StartControlIsr();
  return telemetry::xt_can::kStatusOk;
}

uint8_t BinaryCommands::HandleRaw(const uint8_t* payload, size_t payload_len)
{
  if (app_->state_ != State::RUN || !app_->phase_pwm_->init_ok())
  {
    return telemetry::xt_can::kStatusNotRun;
  }
  uint16_t a = 0;
  uint16_t b = 0;
  uint16_t c = 0;
  if (!ReadU16(payload, payload_len, 0, &a) ||
      !ReadU16(payload, payload_len, 2, &b) ||
      !ReadU16(payload, payload_len, 4, &c))
  {
    return telemetry::xt_can::kStatusBadLen;
  }
  if (a > hal::PhasePwm::kDutyMax || b > hal::PhasePwm::kDutyMax ||
      c > hal::PhasePwm::kDutyMax)
  {
    return telemetry::xt_can::kStatusFail;
  }

  app_->voltage_foc_->Stop();
  app_->current_loop_->Stop();
  app_->phase_pwm_->DisableControlIsr();
  app_->dq_valid_ = false;

  app_->phase_pwm_->SetDuty(a, b, c);
  app_->gate_driver_->PowerOn();
  app_->pwm_output_on_ = true;
  app_->mode_ = telemetry::xt_can::kModeRaw;
  return telemetry::xt_can::kStatusOk;
}

uint8_t BinaryCommands::HandleInfo(uint8_t seq)
{
  const auto& motor = board::MotorParams();
  const auto cl = board::CurrentLoopOptions();

  telemetry::xt_can::Info info{};
  info.hdr.seq = seq;
  info.node_id = app_->binary_link_->node_id();
  info.fw_major = telemetry::xt_can::kFwMajor;
  info.fw_minor = telemetry::xt_can::kFwMinor;
  info.fw_patch = telemetry::xt_can::kFwPatch;
  info.pwm_hz = static_cast<uint16_t>(board::PhasePwmOptions().rate_hz);
  info.bus_mV =
      static_cast<uint16_t>(board::kBusVoltage_V * 1000.0f + 0.5f);
  info.i_max_mA = static_cast<uint16_t>(cl.max_current_A * 1000.0f + 0.5f);
  info.pole_pairs = static_cast<uint16_t>(motor.pole_pairs + 0.5f);
  info.r_mohm =
      static_cast<uint16_t>(motor.phase_resistance_ohm * 1000.0f + 0.5f);
  info.l_uH =
      static_cast<uint16_t>(motor.phase_inductance_H * 1.0e6f + 0.5f);
  info.family = 3;  // moteus-x1 / family 3
  std::memset(info.motor, 0, sizeof(info.motor));
  std::strncpy(info.motor, device::motor::kActiveName, sizeof(info.motor) - 1);

  app_->binary_link_->SendInfo(info);
  return telemetry::xt_can::kStatusOk;
}

uint8_t BinaryCommands::HandleSnap(uint8_t seq, const uint8_t* payload,
                                   size_t payload_len)
{
  if (app_->phase_pwm_.get() == nullptr || !app_->phase_pwm_->control_isr_on())
  {
    return telemetry::xt_can::kStatusNotRun;
  }
  if (app_->snapshot_.busy())
  {
    return telemetry::xt_can::kStatusFail;
  }

  uint16_t n_samples = telemetry::xt_can::kSnapMaxSamples;
  uint8_t decimate = 1;
  if (payload != nullptr &&
      payload_len >= sizeof(telemetry::xt_can::SnapRequest))
  {
    telemetry::xt_can::SnapRequest req{};
    std::memcpy(&req, payload, sizeof(req));
    n_samples = req.n_samples;
    decimate = req.decimate;
  }
  else if (payload_len != 0 &&
           payload_len < sizeof(telemetry::xt_can::SnapRequest))
  {
    return telemetry::xt_can::kStatusBadLen;
  }

  const uint16_t pwm_hz =
      static_cast<uint16_t>(board::PhasePwmOptions().rate_hz);
  if (!app_->snapshot_.Arm(n_samples, decimate, pwm_hz))
  {
    return telemetry::xt_can::kStatusFail;
  }
  app_->snap_seq_ = seq;
  app_->snap_meta_sent_ = false;
  return telemetry::xt_can::kStatusOk;
}

}  // namespace app
