#include "binary_commands.h"

#include <cstring>

#include "application.h"
#include "board_config.h"
#include "device/motor.h"
#include "foc_ctrl/simple_pi.h"
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

bool ReadU16(const uint8_t* p, size_t len, size_t off, uint16_t* out)
{
  if (out == nullptr || p == nullptr || off + 2 > len)
  {
    return false;
  }
  std::memcpy(out, p + off, 2);
  return true;
}

bool NotReadI32(const uint8_t* p, size_t len, size_t off, int32_t* out)
{
  return ReadI32(p, len, off, out) == false;
}

bool NotReadU16(const uint8_t* p, size_t len, size_t off, uint16_t* out)
{
  return ReadU16(p, len, off, out) == false;
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
  uint8_t status = telemetry::xt_can::kStatusBadCmd;
  switch (cmd)
  {
    case telemetry::xt_can::kCmdStop:
      status = HandleStop();
      break;
    case telemetry::xt_can::kCmdQuery:
      status = HandleQuery();
      break;
    case telemetry::xt_can::kCmdDq:
      status = HandleDq(payload, payload_len);
      break;
    case telemetry::xt_can::kCmdVel:
      status = HandleVel(payload, payload_len);
      break;
    case telemetry::xt_can::kCmdCal:
      status = HandleCal(payload, payload_len);
      break;
    case telemetry::xt_can::kCmdVfoc:
      status = HandleVfoc(payload, payload_len);
      break;
    case telemetry::xt_can::kCmdRaw:
      status = HandleRaw(payload, payload_len);
      break;
    case telemetry::xt_can::kCmdInfo:
      status = HandleInfo(seq);
      break;
    case telemetry::xt_can::kCmdSnap:
      status = HandleSnap(seq, payload, payload_len);
      break;
    default:
      status = telemetry::xt_can::kStatusBadCmd;
      break;
  }
  if (telemetry::xt_can::UsesCtrlReply(cmd))
  {
    app_->ReplyCtrl(cmd, seq, status);
  }
  return status;
}

uint8_t BinaryCommands::HandleStop()
{
  app_->StopOutput();
  return telemetry::xt_can::kStatusOk;
}

uint8_t BinaryCommands::HandleQuery()
{
  // Idle poll / keep-alive while streaming. Encoder sampled in ReplyCtrl.
  return telemetry::xt_can::kStatusOk;
}

uint8_t BinaryCommands::HandleDq(const uint8_t* payload, size_t payload_len)
{
  if (app_->state_ != State::RUN || app_->phase_pwm_->init_ok() == false)
  {
    return telemetry::xt_can::kStatusNotRun;
  }
  int32_t id_mA = 0;
  int32_t iq_mA = 0;
  int32_t omega = 0;
  if (NotReadI32(payload, payload_len, 0, &id_mA) ||
      NotReadI32(payload, payload_len, 4, &iq_mA) ||
      NotReadI32(payload, payload_len, 8, &omega))
  {
    return telemetry::xt_can::kStatusBadLen;
  }

  const float id_ref = MilliToAmps(id_mA);
  const float iq_ref = MilliToAmps(iq_mA);
  const float theta_rate = static_cast<float>(omega) * 0.001f;
  if (app_->mode_ == telemetry::xt_can::kModeDq &&
      app_->current_loop_.get() != nullptr && app_->current_loop_->active())
  {
    app_->current_loop_->SetRefs(id_ref, iq_ref);
    const bool use_enc =
        app_->encoder_ok_ && app_->ma600_.get() != nullptr;
    if (use_enc == false)
    {
      app_->current_loop_->SetThetaRate(theta_rate);
    }
    return telemetry::xt_can::kStatusOk;
  }

  app_->voltage_foc_->Stop();
  if (app_->position_loop_.get() != nullptr)
  {
    app_->position_loop_->Stop();
  }
  app_->phase_pwm_->DisableControlIsr();

  const bool use_enc = app_->encoder_ok_ && app_->ma600_.get() != nullptr;
  float theta_elec = 0.0f;
  const float* theta_override = nullptr;
  if (use_enc)
  {
    app_->SampleEncoder();
    app_->encoder_pll_.Reset(app_->enc_theta_mech_rad_);
    app_->enc_theta_elec_rad_ = app_->encoder_pll_.electrical_theta();
    theta_elec = app_->enc_theta_elec_rad_;
    theta_override = &theta_elec;
  }

  foc_ctrl::CurrentLoop::Command cmd;
  cmd.theta_rad = theta_elec;
  cmd.id_A = id_ref;
  cmd.iq_A = iq_ref;
  cmd.theta_rate_rad_s = use_enc ? 0.0f : theta_rate;
  app_->current_loop_->Start(cmd);

  app_->last_current_ = app_->current_adc_->ReadLatest();
  foc_ctrl::CurrentLoop::Duties duties;
  if (app_->current_loop_->Step(0.0f, app_->last_current_.i1_A,
                                app_->last_current_.i2_A,
                                app_->last_current_.i3_A, &duties,
                                theta_override) == false)
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

uint8_t BinaryCommands::HandleVel(const uint8_t* payload, size_t payload_len)
{
  if (app_->state_ != State::RUN || app_->phase_pwm_->init_ok() == false)
  {
    return telemetry::xt_can::kStatusNotRun;
  }
  if (app_->encoder_ok_ == false || app_->ma600_.get() == nullptr ||
      app_->position_loop_.get() == nullptr)
  {
    return telemetry::xt_can::kStatusFail;
  }

  int32_t omega_mech_mrad_s = 0;
  int32_t id_mA = 0;
  if (NotReadI32(payload, payload_len, 0, &omega_mech_mrad_s) ||
      NotReadI32(payload, payload_len, 4, &id_mA))
  {
    return telemetry::xt_can::kStatusBadLen;
  }

  const float omega_ref = static_cast<float>(omega_mech_mrad_s) * 0.001f;
  const float id_ref = MilliToAmps(id_mA);
  if (app_->mode_ == telemetry::xt_can::kModeVel &&
      app_->position_loop_.get() != nullptr && app_->position_loop_->active() &&
      app_->current_loop_.get() != nullptr && app_->current_loop_->active())
  {
    app_->position_loop_->SetVelocity(omega_ref);
    app_->position_loop_->SetIdRef(id_ref);
    app_->current_loop_->SetRefs(id_ref, app_->current_loop_->iq_ref_A());
    const float omega_control = app_->position_loop_->control_velocity();
    app_->current_loop_->SetOmega(
        omega_control * board::MotorParams().pole_pairs, omega_control);
    return telemetry::xt_can::kStatusOk;
  }

  app_->voltage_foc_->Stop();
  app_->phase_pwm_->DisableControlIsr();

  // moteus velocity mode: position=NaN + velocity command.
  app_->SampleEncoder();
  app_->encoder_pll_.Reset(app_->enc_theta_mech_rad_);
  app_->enc_theta_elec_rad_ = app_->encoder_pll_.electrical_theta();
  app_->position_loop_->Start(foc_ctrl::QuietNan(), omega_ref, id_ref);

  const float theta_elec = app_->enc_theta_elec_rad_;
  foc_ctrl::CurrentLoop::Command cmd;
  cmd.theta_rad = theta_elec;
  cmd.id_A = id_ref;
  cmd.iq_A = 0.0f;
  cmd.theta_rate_rad_s = 0.0f;
  app_->current_loop_->Start(cmd);
  const float omega_control = app_->position_loop_->control_velocity();
  app_->current_loop_->SetOmega(
      omega_control * board::MotorParams().pole_pairs, omega_control);

  app_->last_current_ = app_->current_adc_->ReadLatest();
  foc_ctrl::CurrentLoop::Duties duties;
  if (app_->current_loop_->Step(0.0f, app_->last_current_.i1_A,
                                app_->last_current_.i2_A,
                                app_->last_current_.i3_A, &duties,
                                &theta_elec) == false)
  {
    app_->current_loop_->Stop();
    app_->position_loop_->Stop();
    return telemetry::xt_can::kStatusFail;
  }
  app_->id_A_ = app_->current_loop_->id_A();
  app_->iq_A_ = app_->current_loop_->iq_A();
  app_->dq_valid_ = app_->last_current_.ok;
  app_->phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
  app_->gate_driver_->PowerOn();
  app_->pwm_output_on_ = true;
  app_->mode_ = telemetry::xt_can::kModeVel;
  app_->StartControlIsr();
  return telemetry::xt_can::kStatusOk;
}

uint8_t BinaryCommands::HandleVfoc(const uint8_t* payload, size_t payload_len)
{
  if (app_->state_ != State::RUN || app_->phase_pwm_->init_ok() == false)
  {
    return telemetry::xt_can::kStatusNotRun;
  }
  int32_t theta_mrad = 0;
  int32_t v_mV = 0;
  int32_t omega = 0;
  if (NotReadI32(payload, payload_len, 0, &theta_mrad) ||
      NotReadI32(payload, payload_len, 4, &v_mV) ||
      NotReadI32(payload, payload_len, 8, &omega))
  {
    return telemetry::xt_can::kStatusBadLen;
  }

  const float v_ref = static_cast<float>(v_mV) * 0.001f;
  const float omega_ref = static_cast<float>(omega) * 0.001f;
  if (app_->mode_ == telemetry::xt_can::kModeVfoc &&
      app_->voltage_foc_.get() != nullptr && app_->voltage_foc_->active())
  {
    app_->voltage_foc_->SetVoltage(v_ref);
    app_->voltage_foc_->SetThetaRate(omega_ref);
    return telemetry::xt_can::kStatusOk;
  }

  app_->current_loop_->Stop();
  if (app_->position_loop_.get() != nullptr)
  {
    app_->position_loop_->Stop();
  }
  app_->phase_pwm_->DisableControlIsr();

  foc_ctrl::VoltageFoc::Command cmd;
  cmd.theta_rad = static_cast<float>(theta_mrad) * 0.001f;
  cmd.voltage_V = v_ref;
  cmd.theta_rate_rad_s = omega_ref;
  app_->voltage_foc_->Start(cmd);

  foc_ctrl::VoltageFoc::Duties duties;
  if (app_->voltage_foc_->Step(0.0f, &duties) == false)
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

uint8_t BinaryCommands::HandleCal(const uint8_t* payload, size_t payload_len)
{
  if (app_->state_ != State::RUN || app_->phase_pwm_->init_ok() == false)
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
    app_->encoder_cal_.Stop();
    app_->StopOutput();
    app_->RestoreEncoderCalBackup();
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
    if (app_->current_loop_.get() == nullptr)
    {
      return telemetry::xt_can::kStatusFail;
    }
    app_->current_loop_->Stop();
    if (app_->position_loop_.get() != nullptr)
    {
      app_->position_loop_->Stop();
    }
    if (app_->voltage_foc_.get() != nullptr)
    {
      app_->voltage_foc_->Stop();
    }
    app_->phase_pwm_->DisableControlIsr();

    calibration::RIdentCal::Options ropts;
    const float max_current =
        static_cast<float>(req.omega_elec_mrad_s) * 0.001f;
    ropts.max_current_A = (max_current > 0.05f) ? max_current : 1.5f;
    if (req.voltage_mV > 0)
    {
      ropts.n_points = static_cast<uint8_t>(
          req.voltage_mV > 255 ? 255 : req.voltage_mV);
    }
    app_->last_cal_kind_ = telemetry::xt_can::kCalSubResistance;
    app_->r_cal_.Start(ropts);

    foc_ctrl::CurrentLoop::Command cmd;
    cmd.theta_rad = 0.0f;
    cmd.id_A = app_->r_cal_.id_cmd_A();
    cmd.iq_A = 0.0f;
    cmd.theta_rate_rad_s = 0.0f;
    app_->current_loop_->Start(cmd);

    app_->last_current_ = app_->current_adc_->ReadLatest();
    foc_ctrl::CurrentLoop::Duties duties;
    if (app_->current_loop_->Step(0.0f, app_->last_current_.i1_A,
                                  app_->last_current_.i2_A,
                                  app_->last_current_.i3_A, &duties,
                                  nullptr) == false)
    {
      app_->current_loop_->Stop();
      app_->r_cal_.Stop();
      return telemetry::xt_can::kStatusFail;
    }
    app_->id_A_ = app_->current_loop_->id_A();
    app_->iq_A_ = app_->current_loop_->iq_A();
    app_->dq_valid_ = app_->last_current_.ok;
    app_->phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    app_->gate_driver_->PowerOn();
    app_->pwm_output_on_ = true;
    app_->mode_ = telemetry::xt_can::kModeCal;
    app_->StartControlIsr();
    return telemetry::xt_can::kStatusOk;
  }
  if (is_l)
  {
    if (app_->voltage_foc_.get() == nullptr ||
        app_->current_loop_.get() == nullptr)
    {
      return telemetry::xt_can::kStatusFail;
    }
    app_->current_loop_->Stop();
    if (app_->position_loop_.get() != nullptr)
    {
      app_->position_loop_->Stop();
    }
    app_->voltage_foc_->Stop();
    app_->phase_pwm_->DisableControlIsr();

    calibration::LIdentCal::Options lopts;
    lopts.resistance_ohm = app_->current_loop_->resistance_ohm();
    if (req.voltage_mV > 0)
    {
      lopts.step_voltage_V = static_cast<float>(req.voltage_mV) * 0.001f;
    }
    if (req.omega_elec_mrad_s > 0)
    {
      lopts.n_trials = static_cast<uint8_t>(
          req.omega_elec_mrad_s > 255 ? 255 : req.omega_elec_mrad_s);
    }
    app_->last_cal_kind_ = telemetry::xt_can::kCalSubInductance;
    app_->l_cal_.Start(lopts);

    foc_ctrl::VoltageFoc::Command cmd;
    cmd.theta_rad = 0.0f;
    cmd.voltage_V = app_->l_cal_.voltage_d_cmd_V();
    cmd.q_voltage_V = app_->l_cal_.voltage_q_cmd_V();
    cmd.theta_rate_rad_s = 0.0f;
    app_->voltage_foc_->Start(cmd);

    foc_ctrl::VoltageFoc::Duties duties;
    if (app_->voltage_foc_->Step(0.0f, &duties) == false)
    {
      app_->voltage_foc_->Stop();
      app_->l_cal_.Stop();
      return telemetry::xt_can::kStatusFail;
    }
    app_->last_current_ = app_->current_adc_->ReadLatest();
    app_->ObserveDqFromSample(app_->voltage_foc_->theta_rad(),
                              app_->last_current_);
    app_->phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    app_->gate_driver_->PowerOn();
    app_->pwm_output_on_ = true;
    app_->mode_ = telemetry::xt_can::kModeCal;
    app_->StartControlIsr();
    return telemetry::xt_can::kStatusOk;
  }
  if (is_bemf)
  {
    if (app_->encoder_ok_ == false || app_->ma600_.get() == nullptr ||
        app_->position_loop_.get() == nullptr ||
        app_->current_loop_.get() == nullptr ||
        !app_->encoder_pll_.theta_valid())
    {
      return telemetry::xt_can::kStatusFail;
    }
    app_->current_loop_->Stop();
    app_->position_loop_->Stop();
    if (app_->voltage_foc_.get() != nullptr)
    {
      app_->voltage_foc_->Stop();
    }
    app_->phase_pwm_->DisableControlIsr();

    calibration::BemfIdentCal::Options bopts;
    bopts.resistance_ohm = app_->current_loop_->resistance_ohm();
    const float max_speed =
        static_cast<float>(req.omega_elec_mrad_s) * 0.001f;
    bopts.max_speed_rad_s = (max_speed > 1.0f) ? max_speed : 60.0f;
    if (req.voltage_mV > 0)
    {
      bopts.n_points = static_cast<uint8_t>(
          req.voltage_mV > 255 ? 255 : req.voltage_mV);
    }
    app_->last_cal_kind_ = telemetry::xt_can::kCalSubBemf;
    app_->bemf_cal_.Start(bopts);

    // moteus velocity mode cold start (mirrors HandleVel), Id forced to 0.
    app_->SampleEncoder();
    app_->encoder_pll_.Reset(app_->enc_theta_mech_rad_);
    app_->enc_theta_elec_rad_ = app_->encoder_pll_.electrical_theta();
    app_->position_loop_->Start(foc_ctrl::QuietNan(),
                                app_->bemf_cal_.velocity_cmd_mech_rad_s(),
                                0.0f);

    const float theta_elec = app_->enc_theta_elec_rad_;
    foc_ctrl::CurrentLoop::Command cmd;
    cmd.theta_rad = theta_elec;
    cmd.id_A = 0.0f;
    cmd.iq_A = 0.0f;
    cmd.theta_rate_rad_s = 0.0f;
    app_->current_loop_->Start(cmd);

    app_->last_current_ = app_->current_adc_->ReadLatest();
    foc_ctrl::CurrentLoop::Duties duties;
    if (app_->current_loop_->Step(0.0f, app_->last_current_.i1_A,
                                  app_->last_current_.i2_A,
                                  app_->last_current_.i3_A, &duties,
                                  &theta_elec) == false)
    {
      app_->current_loop_->Stop();
      app_->position_loop_->Stop();
      app_->bemf_cal_.Stop();
      return telemetry::xt_can::kStatusFail;
    }
    app_->id_A_ = app_->current_loop_->id_A();
    app_->iq_A_ = app_->current_loop_->iq_A();
    app_->dq_valid_ = app_->last_current_.ok;
    app_->phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    app_->gate_driver_->PowerOn();
    app_->pwm_output_on_ = true;
    app_->mode_ = telemetry::xt_can::kModeCal;
    app_->StartControlIsr();
    return telemetry::xt_can::kStatusOk;
  }
  if (is_cogging)
  {
    if (app_->encoder_ok_ == false || app_->ma600_.get() == nullptr ||
        app_->position_loop_.get() == nullptr ||
        app_->current_loop_.get() == nullptr ||
        !app_->encoder_pll_.theta_valid())
    {
      return telemetry::xt_can::kStatusFail;
    }
    app_->current_loop_->Stop();
    app_->position_loop_->Stop();
    if (app_->voltage_foc_.get() != nullptr)
    {
      app_->voltage_foc_->Stop();
    }
    app_->phase_pwm_->DisableControlIsr();

    calibration::CoggingCal::Options copts;
    const float vel = static_cast<float>(req.omega_elec_mrad_s) * 0.001f;
    copts.velocity_mech_rad_s = (vel > 0.05f) ? vel : 5.0f;
    if (req.voltage_mV > 0)
    {
      copts.record_revs = static_cast<float>(req.voltage_mV) * 0.01f;
    }
    app_->last_cal_kind_ = telemetry::xt_can::kCalSubCogging;
    app_->cogging_cal_.Start(copts);

    // moteus velocity mode cold start (mirrors HandleVel), Id forced to 0.
    app_->SampleEncoder();
    app_->encoder_pll_.Reset(app_->enc_theta_mech_rad_);
    app_->enc_theta_elec_rad_ = app_->encoder_pll_.electrical_theta();
    app_->position_loop_->Start(foc_ctrl::QuietNan(),
                                app_->cogging_cal_.velocity_cmd_mech_rad_s(),
                                0.0f);

    const float theta_elec = app_->enc_theta_elec_rad_;
    foc_ctrl::CurrentLoop::Command cmd;
    cmd.theta_rad = theta_elec;
    cmd.id_A = 0.0f;
    cmd.iq_A = 0.0f;
    cmd.theta_rate_rad_s = 0.0f;
    app_->current_loop_->Start(cmd);

    app_->last_current_ = app_->current_adc_->ReadLatest();
    foc_ctrl::CurrentLoop::Duties duties;
    if (app_->current_loop_->Step(0.0f, app_->last_current_.i1_A,
                                  app_->last_current_.i2_A,
                                  app_->last_current_.i3_A, &duties,
                                  &theta_elec) == false)
    {
      app_->current_loop_->Stop();
      app_->position_loop_->Stop();
      app_->cogging_cal_.Stop();
      return telemetry::xt_can::kStatusFail;
    }
    app_->id_A_ = app_->current_loop_->id_A();
    app_->iq_A_ = app_->current_loop_->iq_A();
    app_->dq_valid_ = app_->last_current_.ok;
    app_->phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    app_->gate_driver_->PowerOn();
    app_->pwm_output_on_ = true;
    app_->mode_ = telemetry::xt_can::kModeCal;
    app_->StartControlIsr();
    return telemetry::xt_can::kStatusOk;
  }
  if (app_->encoder_ok_ == false || app_->ma600_.get() == nullptr ||
      app_->current_loop_.get() == nullptr)
  {
    return telemetry::xt_can::kStatusFail;
  }

  if (app_->current_loop_.get() != nullptr)
  {
    app_->current_loop_->Stop();
  }
  if (app_->position_loop_.get() != nullptr)
  {
    app_->position_loop_->Stop();
  }
  if (app_->voltage_foc_.get() != nullptr)
  {
    app_->voltage_foc_->Stop();
  }
  app_->phase_pwm_->DisableControlIsr();

  calibration::EncoderPhaseCal::Options opts;
  opts.method = is_lock ? calibration::EncoderPhaseCal::Method::Lock
                        : calibration::EncoderPhaseCal::Method::Spin;
  // CalRequest voltage_mV is reused as alignment current in mA for encoder
  // lock/spin calibration.  The wire layout remains backward compatible.
  opts.current_A = static_cast<float>(req.voltage_mV) * 0.001f;
  opts.omega_elec_rad_s = static_cast<float>(req.omega_elec_mrad_s) * 0.001f;
  if (opts.omega_elec_rad_s < 0.0f)
  {
    opts.omega_elec_rad_s = -opts.omega_elec_rad_s;
  }
  opts.pole_pairs = board::MotorParams().pole_pairs;
  opts.mech_revs_each_way = 2.0f;
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

  app_->encoder_cal_backup_ = app_->ma600_->options();
  app_->encoder_cal_backup_valid_ = true;
  // Clear prior offset so calibration sees raw counts.
  app_->ma600_->SetOffsetRad(0.0f);
  app_->ma600_->SetSign(1.0f);
  app_->ma600_->SetCommutationOffsets(math::CommutationTable{}, false);
  app_->encoder_cal_persisted_ = false;
  app_->last_cal_kind_ = req.subcmd;
  app_->encoder_cal_.Start(opts);
  app_->cal_last_omega_cmd_ = app_->encoder_cal_.omega_cmd_elec();

  foc_ctrl::CurrentLoop::Command cmd;
  cmd.theta_rad = 0.0f;
  cmd.id_A = opts.current_A;
  cmd.iq_A = 0.0f;
  cmd.theta_rate_rad_s = app_->cal_last_omega_cmd_;
  app_->current_loop_->Start(cmd);
  app_->current_loop_->SetOmega(0.0f, 0.0f);

  app_->last_current_ = app_->current_adc_->ReadLatest();
  foc_ctrl::CurrentLoop::Duties duties;
  if (app_->current_loop_->Step(
          0.0f, app_->last_current_.i1_A, app_->last_current_.i2_A,
          app_->last_current_.i3_A, &duties) == false)
  {
    app_->current_loop_->Stop();
    app_->encoder_cal_.Stop();
    app_->RestoreEncoderCalBackup();
    return telemetry::xt_can::kStatusFail;
  }
  app_->phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
  app_->gate_driver_->PowerOn();
  app_->pwm_output_on_ = true;
  app_->mode_ = telemetry::xt_can::kModeCal;
  app_->StartControlIsr();
  return telemetry::xt_can::kStatusOk;
}

uint8_t BinaryCommands::HandleRaw(const uint8_t* payload, size_t payload_len)
{
  if (app_->state_ != State::RUN || app_->phase_pwm_->init_ok() == false)
  {
    return telemetry::xt_can::kStatusNotRun;
  }
  uint16_t a = 0;
  uint16_t b = 0;
  uint16_t c = 0;
  if (NotReadU16(payload, payload_len, 0, &a) ||
      NotReadU16(payload, payload_len, 2, &b) ||
      NotReadU16(payload, payload_len, 4, &c))
  {
    return telemetry::xt_can::kStatusBadLen;
  }
  if (a > hal::PhasePwm::kDutyMax || b > hal::PhasePwm::kDutyMax ||
      c > hal::PhasePwm::kDutyMax)
  {
    return telemetry::xt_can::kStatusFail;
  }

  if (app_->mode_ == telemetry::xt_can::kModeRaw && app_->pwm_output_on_)
  {
    app_->phase_pwm_->SetDuty(a, b, c);
    return telemetry::xt_can::kStatusOk;
  }

  app_->voltage_foc_->Stop();
  app_->current_loop_->Stop();
  if (app_->position_loop_.get() != nullptr)
  {
    app_->position_loop_->Stop();
  }
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
  if (app_->phase_pwm_.get() == nullptr ||
      app_->phase_pwm_->control_isr_on() == false)
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
  if (app_->snapshot_.Arm(n_samples, decimate, pwm_hz) == false)
  {
    return telemetry::xt_can::kStatusFail;
  }
  app_->snap_seq_ = seq;
  app_->snap_meta_sent_ = false;
  return telemetry::xt_can::kStatusOk;
}

}  // namespace app
