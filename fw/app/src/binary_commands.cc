#include <cstring>

#include "application.h"
#include "board_config.h"
#include "device/motor.h"
#include "math/foc/pi.h"
#include "math/commutation.h"
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

bool NotReadI32(const uint8_t* p, size_t len, size_t off, int32_t* out)
{
  return ReadI32(p, len, off, out) == false;
}

}  // namespace

uint8_t Application::CommandThunk(void* context, uint8_t cmd, uint8_t seq,
                              const uint8_t* payload, size_t payload_len)
{
  if (context == nullptr)
  {
    return telemetry::xt_can::kStatusFail;
  }
  return static_cast<Application*>(context)->HandleCommand(cmd, seq, payload,
                                                              payload_len);
}

uint8_t Application::HandleCommand(uint8_t cmd, uint8_t seq, const uint8_t* payload,
                               size_t payload_len)
{
  uint8_t status = telemetry::xt_can::kStatusBadCmd;
  switch (cmd)
  {
    case telemetry::xt_can::kCmdStop:
      status = HandleStop();
      break;
    case telemetry::xt_can::kCmdQuery:
      status = HandleQuery();
      break;
    case telemetry::xt_can::kCmdServo:
      status = HandleServo(payload, payload_len);
      break;
    case telemetry::xt_can::kCmdCal:
      status = HandleCal(payload, payload_len);
      break;
    case telemetry::xt_can::kCmdInfo:
      status = HandleInfo(seq);
      break;
    case telemetry::xt_can::kCmdSnap:
      status = HandleSnap(seq, payload, payload_len);
      break;
    case telemetry::xt_can::kCmdEncComp:
      status = HandleEncComp(payload, payload_len);
      break;
    default:
      status = telemetry::xt_can::kStatusBadCmd;
      break;
  }
  if (telemetry::xt_can::UsesCtrlReply(cmd))
  {
    this->ReplyCtrl(cmd, seq, status);
  }
  return status;
}

uint8_t Application::HandleStop()
{
  this->StopOutput();
  return telemetry::xt_can::kStatusOk;
}

uint8_t Application::HandleQuery()
{
  // Idle poll / keep-alive while streaming. Encoder sampled in ReplyCtrl.
  return telemetry::xt_can::kStatusOk;
}

uint8_t Application::HandleServo(const uint8_t* payload, size_t payload_len)
{
  int32_t velocity_mrad_s = 0;
  int32_t id_mA = 0;
  if (NotReadI32(payload, payload_len, 0, &velocity_mrad_s) ||
      NotReadI32(payload, payload_len, 4, &id_mA))
  {
    return telemetry::xt_can::kStatusBadLen;
  }
  return this->StartServo(static_cast<float>(velocity_mrad_s) * 0.001f,
                           MilliToAmps(id_mA));
}

uint8_t Application::HandleEncComp(const uint8_t* payload,
                                        size_t payload_len)
{
  if (this->state_ != State::RUN)
  {
    return telemetry::xt_can::kStatusNotRun;
  }
  if (payload == nullptr ||
      payload_len < sizeof(telemetry::xt_can::EncCompRequest))
  {
    return telemetry::xt_can::kStatusBadLen;
  }
  telemetry::xt_can::EncCompRequest req{};
  std::memcpy(&req, payload, sizeof(req));
  if (req.op == telemetry::xt_can::kEncCompOpClear)
  {
    this->calibration_.ClearCompensation();
    return telemetry::xt_can::kStatusOk;
  }
  if (req.op == telemetry::xt_can::kEncCompOpChunk)
  {
    if (req.chunk >= 8u)
    {
      return telemetry::xt_can::kStatusBadCmd;
    }
    if (!this->calibration_.SetCompensationChunk(req.chunk, req.data,
                                               sizeof(req.data)))
    {
      return telemetry::xt_can::kStatusBadCmd;
    }
    return telemetry::xt_can::kStatusOk;
  }
  if (req.op == telemetry::xt_can::kEncCompOpCommit)
  {
    const float peak_rad = static_cast<float>(req.scale_urad) * 1.0e-6f;
    if (peak_rad <= 0.0f)
    {
      return telemetry::xt_can::kStatusBadCmd;
    }
    // Host sends peak |correction| [rad]; store rad-per-LSB = peak/127.
    const float scale = peak_rad / 127.0f;
    if (!this->calibration_.CommitCompensation(scale))
    {
      return telemetry::xt_can::kStatusFail;
    }
    return telemetry::xt_can::kStatusOk;
  }
  return telemetry::xt_can::kStatusBadCmd;
}

uint8_t Application::HandleCal(const uint8_t* payload, size_t payload_len)
{
  if (this->state_ != State::RUN || this->phase_pwm_->init_ok() == false)
  {
    return telemetry::xt_can::kStatusNotRun;
  }
  if (payload == nullptr ||
      payload_len < sizeof(telemetry::xt_can::CalRequest))
  {
    return telemetry::xt_can::kStatusBadLen;
  }

  telemetry::xt_can::CalRequest req{};
  std::memcpy(&req, payload, sizeof(req));

  if (req.subcmd == telemetry::xt_can::kCalSubAbort)
  {
    this->StopOutput();
    return telemetry::xt_can::kStatusOk;
  }
  const bool is_spin = req.subcmd == telemetry::xt_can::kCalSubEncPhase;
  const bool is_lock = req.subcmd == telemetry::xt_can::kCalSubEncLock;
  const bool is_bemf = req.subcmd == telemetry::xt_can::kCalSubBemf;
  const bool is_r = req.subcmd == telemetry::xt_can::kCalSubResistance;
  const bool is_l = req.subcmd == telemetry::xt_can::kCalSubInductance;
  const bool is_cogging = req.subcmd == telemetry::xt_can::kCalSubCogging;
  if (!is_spin && !is_lock && !is_bemf && !is_r && !is_l && !is_cogging)
  {
    return telemetry::xt_can::kStatusBadCmd;
  }
  if (is_r)
  {
    if (this->foc_.get() == nullptr)
    {
      return telemetry::xt_can::kStatusFail;
    }
    this->foc_->Stop();
    if (this->servo_mode_.get() != nullptr)
    {
      this->servo_mode_->Stop();
    }
    if (this->dq_modulator_.get() != nullptr)
    {
      this->dq_modulator_->Stop();
    }
    this->phase_pwm_->DisableControlIsr();

    math::calibration::RIdentCal::Options ropts;
    const float max_current =
        static_cast<float>(req.omega_elec_mrad_s) * 0.001f;
    ropts.max_current_A = (max_current > 0.05f) ? max_current : 1.5f;
    if (req.voltage_mV > 0)
    {
      ropts.n_points = static_cast<uint8_t>(
          req.voltage_mV > 255 ? 255 : req.voltage_mV);
    }
    this->calibration_.SetLastKind(telemetry::xt_can::kCalSubResistance);
    this->calibration_.resistance().Start(ropts);

    math::foc::FocController::Command cmd;
    cmd.theta_rad = 0.0f;
    cmd.id_A = this->calibration_.resistance().id_cmd_A();
    cmd.iq_A = 0.0f;
    cmd.theta_rate_rad_s = 0.0f;
    this->foc_->Start(cmd);

    this->last_current_ = this->current_adc_->ReadLatest();
    math::foc::FocController::Duties duties;
    if (this->foc_->Step(0.0f, this->last_current_.i1_A,
                                  this->last_current_.i2_A,
                                  this->last_current_.i3_A, &duties,
                                  nullptr) == false)
    {
      this->foc_->Stop();
      this->calibration_.resistance().Stop();
      return telemetry::xt_can::kStatusFail;
    }
    this->id_A_ = this->foc_->id_A();
    this->iq_A_ = this->foc_->iq_A();
    this->dq_valid_ = this->last_current_.ok;
    this->phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    this->gate_driver_->PowerOn();
    this->pwm_output_on_ = true;
    this->mode_ = telemetry::xt_can::kModeCal;
    this->StartControlIsr();
    return telemetry::xt_can::kStatusOk;
  }
  if (is_l)
  {
    if (this->dq_modulator_.get() == nullptr ||
        this->foc_.get() == nullptr)
    {
      return telemetry::xt_can::kStatusFail;
    }
    this->foc_->Stop();
    if (this->servo_mode_.get() != nullptr)
    {
      this->servo_mode_->Stop();
    }
    this->dq_modulator_->Stop();
    this->phase_pwm_->DisableControlIsr();

    math::calibration::LIdentCal::Options lopts;
    lopts.resistance_ohm = this->foc_->resistance_ohm();
    if (req.voltage_mV > 0)
    {
      lopts.step_voltage_V = static_cast<float>(req.voltage_mV) * 0.001f;
    }
    if (req.omega_elec_mrad_s > 0)
    {
      lopts.n_trials = static_cast<uint8_t>(
          req.omega_elec_mrad_s > 255 ? 255 : req.omega_elec_mrad_s);
    }
    this->calibration_.SetLastKind(telemetry::xt_can::kCalSubInductance);
    this->calibration_.inductance().Start(lopts);

    math::foc::DqModulator::Command cmd;
    cmd.theta_rad = 0.0f;
    cmd.voltage_V = this->calibration_.inductance().voltage_d_cmd_V();
    cmd.q_voltage_V = this->calibration_.inductance().voltage_q_cmd_V();
    cmd.theta_rate_rad_s = 0.0f;
    this->dq_modulator_->Start(cmd);

    math::foc::DqModulator::Duties duties;
    if (this->dq_modulator_->Step(0.0f, &duties) == false)
    {
      this->dq_modulator_->Stop();
      this->calibration_.inductance().Stop();
      return telemetry::xt_can::kStatusFail;
    }
    this->last_current_ = this->current_adc_->ReadLatest();
    this->ObserveDqFromSample(this->dq_modulator_->theta_rad(),
                              this->last_current_);
    this->phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    this->gate_driver_->PowerOn();
    this->pwm_output_on_ = true;
    this->mode_ = telemetry::xt_can::kModeCal;
    this->StartControlIsr();
    return telemetry::xt_can::kStatusOk;
  }
  if (is_bemf)
  {
    if (!this->encoder_.valid() ||
        this->servo_mode_.get() == nullptr ||
        this->foc_.get() == nullptr ||
        !this->encoder_.pll().theta_valid())
    {
      return telemetry::xt_can::kStatusFail;
    }
    this->foc_->Stop();
    this->servo_mode_->Stop();
    if (this->dq_modulator_.get() != nullptr)
    {
      this->dq_modulator_->Stop();
    }
    this->phase_pwm_->DisableControlIsr();

    math::calibration::BemfIdentCal::Options bopts;
    bopts.resistance_ohm = this->foc_->resistance_ohm();
    const float max_speed =
        static_cast<float>(req.omega_elec_mrad_s) * 0.001f;
    bopts.max_speed_rad_s = (max_speed > 1.0f) ? max_speed : 60.0f;
    if (req.voltage_mV > 0)
    {
      bopts.n_points = static_cast<uint8_t>(
          req.voltage_mV > 255 ? 255 : req.voltage_mV);
    }
    this->calibration_.SetLastKind(telemetry::xt_can::kCalSubBemf);
    this->calibration_.bemf().Start(bopts);

    // moteus velocity mode cold start (mirrors HandleServo), Id forced to 0.
    this->encoder_.SampleBlocking();
    this->encoder_.ResetPll();
    this->servo_mode_->Start(math::foc::QuietNan(),
                                this->calibration_.bemf().velocity_cmd_mech_rad_s(),
                                0.0f);

    const float theta_elec = this->encoder_.sample().electrical_rad;
    math::foc::FocController::Command cmd;
    cmd.theta_rad = theta_elec;
    cmd.id_A = 0.0f;
    cmd.iq_A = 0.0f;
    cmd.theta_rate_rad_s = 0.0f;
    this->foc_->Start(cmd);

    this->last_current_ = this->current_adc_->ReadLatest();
    math::foc::FocController::Duties duties;
    if (this->foc_->Step(0.0f, this->last_current_.i1_A,
                                  this->last_current_.i2_A,
                                  this->last_current_.i3_A, &duties,
                                  &theta_elec) == false)
    {
      this->foc_->Stop();
      this->servo_mode_->Stop();
      this->calibration_.bemf().Stop();
      return telemetry::xt_can::kStatusFail;
    }
    this->id_A_ = this->foc_->id_A();
    this->iq_A_ = this->foc_->iq_A();
    this->dq_valid_ = this->last_current_.ok;
    this->phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    this->gate_driver_->PowerOn();
    this->pwm_output_on_ = true;
    this->mode_ = telemetry::xt_can::kModeCal;
    this->StartControlIsr();
    return telemetry::xt_can::kStatusOk;
  }
  if (is_cogging)
  {
    if (!this->encoder_.valid() ||
        this->servo_mode_.get() == nullptr ||
        this->foc_.get() == nullptr ||
        !this->encoder_.pll().theta_valid())
    {
      return telemetry::xt_can::kStatusFail;
    }
    this->foc_->Stop();
    this->servo_mode_->Stop();
    if (this->dq_modulator_.get() != nullptr)
    {
      this->dq_modulator_->Stop();
    }
    this->phase_pwm_->DisableControlIsr();

    math::calibration::CoggingCal::Options copts;
    const float vel = static_cast<float>(req.omega_elec_mrad_s) * 0.001f;
    copts.velocity_mech_rad_s = (vel > 0.05f) ? vel : 5.0f;
    if (req.voltage_mV > 0)
    {
      copts.record_revs = static_cast<float>(req.voltage_mV) * 0.01f;
    }
    this->calibration_.SetLastKind(telemetry::xt_can::kCalSubCogging);
    this->calibration_.cogging().Start(copts);

    // moteus velocity mode cold start (mirrors HandleServo), Id forced to 0.
    this->encoder_.SampleBlocking();
    this->encoder_.ResetPll();
    this->servo_mode_->Start(math::foc::QuietNan(),
                                this->calibration_.cogging().velocity_cmd_mech_rad_s(),
                                0.0f);

    const float theta_elec = this->encoder_.sample().electrical_rad;
    math::foc::FocController::Command cmd;
    cmd.theta_rad = theta_elec;
    cmd.id_A = 0.0f;
    cmd.iq_A = 0.0f;
    cmd.theta_rate_rad_s = 0.0f;
    this->foc_->Start(cmd);

    this->last_current_ = this->current_adc_->ReadLatest();
    math::foc::FocController::Duties duties;
    if (this->foc_->Step(0.0f, this->last_current_.i1_A,
                                  this->last_current_.i2_A,
                                  this->last_current_.i3_A, &duties,
                                  &theta_elec) == false)
    {
      this->foc_->Stop();
      this->servo_mode_->Stop();
      this->calibration_.cogging().Stop();
      return telemetry::xt_can::kStatusFail;
    }
    this->id_A_ = this->foc_->id_A();
    this->iq_A_ = this->foc_->iq_A();
    this->dq_valid_ = this->last_current_.ok;
    this->phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    this->gate_driver_->PowerOn();
    this->pwm_output_on_ = true;
    this->mode_ = telemetry::xt_can::kModeCal;
    this->StartControlIsr();
    return telemetry::xt_can::kStatusOk;
  }
  if (!this->encoder_.valid() ||
      this->foc_.get() == nullptr)
  {
    return telemetry::xt_can::kStatusFail;
  }

  if (this->foc_.get() != nullptr)
  {
    this->foc_->Stop();
  }
  if (this->servo_mode_.get() != nullptr)
  {
    this->servo_mode_->Stop();
  }
  if (this->dq_modulator_.get() != nullptr)
  {
    this->dq_modulator_->Stop();
  }
  this->phase_pwm_->DisableControlIsr();

  math::calibration::EncoderPhaseCal::Options opts;
  opts.method = is_lock ? math::calibration::EncoderPhaseCal::Method::Lock
                        : math::calibration::EncoderPhaseCal::Method::Spin;
  // CalRequest voltage_mV is reused as alignment current in mA for encoder
  // lock/spin calibration.  The wire layout remains backward compatible.
  opts.current_A = static_cast<float>(req.voltage_mV) * 0.001f;
  opts.omega_elec_rad_s = static_cast<float>(req.omega_elec_mrad_s) * 0.001f;
  if (opts.omega_elec_rad_s < 0.0f)
  {
    opts.omega_elec_rad_s = -opts.omega_elec_rad_s;
  }
  opts.pole_pairs = board::MotorParams().pole_pairs;
  // More revs → denser commutation table; reduces one-sided high-speed bias.
  opts.mech_revs_each_way = 3.0f;
  if (is_lock)
  {
    opts.current_A = (opts.current_A > 0.05f) ? opts.current_A : 1.0f;
    opts.omega_elec_rad_s = 0.0f;
  }
  else
  {
    opts.current_A = (opts.current_A > 0.05f) ? opts.current_A : 1.0f;
    opts.omega_elec_rad_s =
        (opts.omega_elec_rad_s > 1.0f) ? opts.omega_elec_rad_s : 40.0f;
    if (opts.omega_elec_rad_s > 80.0f)
    {
      opts.omega_elec_rad_s = 80.0f;
    }
  }
  if (opts.current_A > 2.0f)
  {
    opts.current_A = 2.0f;
  }

  // The encoder service snapshots the installed calibration and presents raw
  // sensor coordinates to the phase-calibration algorithm.
  this->calibration_.SetLastKind(req.subcmd);
  if (!this->calibration_.StartEncoderPhase(opts))
  {
    return telemetry::xt_can::kStatusFail;
  }
  const float phase_cal_omega = this->calibration_.encoder_phase().omega_cmd_elec();

  math::foc::FocController::Command cmd;
  cmd.theta_rad = 0.0f;
  cmd.id_A = opts.current_A;
  cmd.iq_A = 0.0f;
  cmd.theta_rate_rad_s = phase_cal_omega;
  this->foc_->Start(cmd);
  this->foc_->SetOmega(0.0f, 0.0f);

  this->last_current_ = this->current_adc_->ReadLatest();
  math::foc::FocController::Duties duties;
  if (this->foc_->Step(
          0.0f, this->last_current_.i1_A, this->last_current_.i2_A,
          this->last_current_.i3_A, &duties) == false)
  {
    this->foc_->Stop();
    this->calibration_.encoder_phase().Stop();
    this->encoder_.RestoreCalibration();
    return telemetry::xt_can::kStatusFail;
  }
  this->phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
  this->gate_driver_->PowerOn();
  this->pwm_output_on_ = true;
  this->mode_ = telemetry::xt_can::kModeCal;
  this->StartControlIsr();
  return telemetry::xt_can::kStatusOk;
}

uint8_t Application::HandleInfo(uint8_t seq)
{
  const auto& motor = board::MotorParams();
  const auto cl = board::FocControllerOptions();

  telemetry::xt_can::Info info{};
  info.hdr.seq = seq;
  info.node_id = this->binary_link_->node_id();
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

  this->binary_link_->SendInfo(info);
  return telemetry::xt_can::kStatusOk;
}

uint8_t Application::HandleSnap(uint8_t seq, const uint8_t* payload,
                                   size_t payload_len)
{
  if (this->phase_pwm_.get() == nullptr ||
      this->phase_pwm_->control_isr_on() == false)
  {
    return telemetry::xt_can::kStatusNotRun;
  }
  if (this->snapshot_.busy())
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
  if (this->snapshot_.Arm(n_samples, decimate, pwm_hz) == false)
  {
    return telemetry::xt_can::kStatusFail;
  }
  this->snap_seq_ = seq;
  this->snap_meta_sent_ = false;
  return telemetry::xt_can::kStatusOk;
}

}  // namespace app
