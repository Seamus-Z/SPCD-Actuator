#include "middleware/communication/telemetry_publisher.h"

namespace middleware::communication
{
namespace
{

int32_t AmpsToMilli(float amps)
{
  return static_cast<int32_t>(amps * 1000.0f);
}

uint8_t ModeId(control::Mode mode)
{
  switch (mode)
  {
    case control::Mode::Calibration:
      return protocol::xt_can::kModeCal;
    case control::Mode::Servo:
      return protocol::xt_can::kModeServo;
    case control::Mode::Current:
      return protocol::xt_can::kModeCurrent;
    case control::Mode::Mit:
      return protocol::xt_can::kModeMit;
    case control::Mode::Stopped:
      return protocol::xt_can::kModeStop;
  }
  return protocol::xt_can::kModeStop;
}

}  // namespace

TelemetryPublisher::TelemetryPublisher(const Dependencies& dependencies)
    : link_(dependencies.link),
      motor_control_(dependencies.motor_control),
      encoder_(dependencies.encoder),
      calibration_(dependencies.calibration),
      snapshot_(dependencies.snapshot),
      runtime_config_(dependencies.runtime_config)
{
}

void TelemetryPublisher::ReplyControl(uint8_t command, uint8_t sequence,
                                      uint8_t status, bool driver_fault)
{
  if (link_ == nullptr || motor_control_ == nullptr || encoder_ == nullptr ||
      calibration_ == nullptr || snapshot_ == nullptr)
  {
    return;
  }
  // Snapshot TX owns the bus; skip Live reply until the burst finishes.
  if (snapshot_->busy())
  {
    return;
  }
  if (!motor_control_->isr_enabled())
  {
    encoder_->SampleBlocking();
    motor_control_->SampleSlowTelemetry();
  }
  link_->SendCtrlReply(
      BuildControlReply(command, sequence, status, driver_fault));
  if (CalibrationActive())
  {
    link_->SendCalTelem(BuildCalibrationTelemetry());
  }
}

void TelemetryPublisher::SendInfo(const protocol::xt_can::Info& info)
{
  if (link_ != nullptr)
  {
    link_->SendInfo(info);
  }
}

bool TelemetryPublisher::SendConfig(const void* data, size_t size)
{
  return link_ != nullptr && link_->SendConf(data, size);
}

void TelemetryPublisher::Poll()
{
  if (snapshot_ != nullptr)
  {
    snapshot_->PollSend(link_);
  }
}

bool TelemetryPublisher::CalibrationActive() const
{
  return calibration_->encoder_phase().state() !=
             math::calibration::EncoderPhaseCal::State::Idle ||
         calibration_->bemf().state() !=
             math::calibration::BemfIdentCal::State::Idle ||
         calibration_->resistance().state() !=
             math::calibration::RIdentCal::State::Idle ||
         calibration_->inductance().state() !=
             math::calibration::LIdentCal::State::Idle ||
         calibration_->cogging().state() !=
             math::calibration::CoggingCal::State::Idle;
}

protocol::xt_can::CalTelem
TelemetryPublisher::BuildCalibrationTelemetry() const
{
  if (calibration_->last_kind() == protocol::xt_can::kCalSubBemf)
  {
    protocol::xt_can::CalTelem result{};
    result.kind = protocol::xt_can::kCalSubBemf;
    switch (calibration_->bemf().state())
    {
      case math::calibration::BemfIdentCal::State::Running:
        result.state = protocol::xt_can::kCalStateBemfRun;
        break;
      case math::calibration::BemfIdentCal::State::Done:
        result.state = protocol::xt_can::kCalStateDone;
        break;
      case math::calibration::BemfIdentCal::State::Failed:
        result.state = protocol::xt_can::kCalStateFailed;
        break;
      default:
        result.state = protocol::xt_can::kCalStateIdle;
        break;
    }
    result.progress_pm = calibration_->bemf().progress_permille();
    const auto& calibration_result = calibration_->bemf().result();
    result.offset_mrad =
        static_cast<int32_t>(calibration_result.ke_v_s_per_rad * 1.0e6f);
    result.residual_mrad = static_cast<int32_t>(calibration_result.r2 * 1.0e6f);
    result.sign = 0;
    result.ok = calibration_result.ok ? 1 : 0;
    uint16_t samples = calibration_result.points_used;
    if (calibration_->bemf_persisted() && samples < 0x8000u)
    {
      samples = static_cast<uint16_t>(samples | 0x8000u);
    }
    result.samples = samples;
    return result;
  }

  if (calibration_->last_kind() == protocol::xt_can::kCalSubResistance)
  {
    protocol::xt_can::CalTelem result{};
    result.kind = protocol::xt_can::kCalSubResistance;
    switch (calibration_->resistance().state())
    {
      case math::calibration::RIdentCal::State::Running:
        result.state = protocol::xt_can::kCalStateRRun;
        break;
      case math::calibration::RIdentCal::State::Done:
        result.state = protocol::xt_can::kCalStateDone;
        break;
      case math::calibration::RIdentCal::State::Failed:
        result.state = protocol::xt_can::kCalStateFailed;
        break;
      default:
        result.state = protocol::xt_can::kCalStateIdle;
        break;
    }
    result.progress_pm = calibration_->resistance().progress_permille();
    const auto& calibration_result = calibration_->resistance().result();
    result.offset_mrad =
        static_cast<int32_t>(calibration_result.resistance_ohm * 1.0e6f);
    result.residual_mrad = static_cast<int32_t>(calibration_result.r2 * 1.0e6f);
    result.sign = 0;
    result.ok = calibration_result.ok ? 1 : 0;
    uint16_t samples = calibration_result.points_used;
    if (calibration_->resistance_persisted() && samples < 0x8000u)
    {
      samples = static_cast<uint16_t>(samples | 0x8000u);
    }
    result.samples = samples;
    return result;
  }

  if (calibration_->last_kind() == protocol::xt_can::kCalSubInductance)
  {
    protocol::xt_can::CalTelem result{};
    result.kind = protocol::xt_can::kCalSubInductance;
    switch (calibration_->inductance().state())
    {
      case math::calibration::LIdentCal::State::Running:
        result.state = protocol::xt_can::kCalStateLRun;
        break;
      case math::calibration::LIdentCal::State::Done:
        result.state = protocol::xt_can::kCalStateDone;
        break;
      case math::calibration::LIdentCal::State::Failed:
        result.state = protocol::xt_can::kCalStateFailed;
        break;
      default:
        result.state = protocol::xt_can::kCalStateIdle;
        break;
    }
    result.progress_pm = calibration_->inductance().progress_permille();
    const auto& calibration_result = calibration_->inductance().result();
    result.offset_mrad =
        static_cast<int32_t>(calibration_result.inductance_d_H * 1.0e9f);
    result.residual_mrad =
        static_cast<int32_t>(calibration_result.inductance_q_H * 1.0e9f);
    result.sign = 0;
    result.ok = calibration_result.ok ? 1 : 0;
    uint16_t samples = static_cast<uint16_t>(
        calibration_result.trials_d_used + calibration_result.trials_q_used);
    if (calibration_->inductance_persisted() && samples < 0x8000u)
    {
      samples = static_cast<uint16_t>(samples | 0x8000u);
    }
    result.samples = samples;
    return result;
  }

  if (calibration_->last_kind() == protocol::xt_can::kCalSubCogging)
  {
    protocol::xt_can::CalTelem result{};
    result.kind = protocol::xt_can::kCalSubCogging;
    switch (calibration_->cogging().state())
    {
      case math::calibration::CoggingCal::State::Running:
        result.state = protocol::xt_can::kCalStateCoggingRun;
        break;
      case math::calibration::CoggingCal::State::Done:
        result.state = protocol::xt_can::kCalStateDone;
        break;
      case math::calibration::CoggingCal::State::Failed:
        result.state = protocol::xt_can::kCalStateFailed;
        break;
      default:
        result.state = protocol::xt_can::kCalStateIdle;
        break;
    }
    result.progress_pm = calibration_->cogging().progress_permille();
    const auto& calibration_result = calibration_->cogging().result();
    result.offset_mrad = static_cast<int32_t>(calibration_result.scale * 1.0e6f);
    result.residual_mrad =
        static_cast<int32_t>(calibration_result.peak_A * 1.0e6f);
    result.sign = 0;
    result.ok = calibration_result.ok ? 1 : 0;
    result.samples = calibration_->cogging_persisted() ? 0x8001u : 1u;
    return result;
  }

  protocol::xt_can::CalTelem result{};
  result.kind = static_cast<uint8_t>(
      calibration_->encoder_phase().method() ==
              math::calibration::EncoderPhaseCal::Method::Lock
          ? protocol::xt_can::kCalSubEncLock
          : protocol::xt_can::kCalSubEncPhase);
  result.state = static_cast<uint8_t>(calibration_->encoder_phase().state());
  result.progress_pm = calibration_->encoder_phase().progress_permille();
  const auto& calibration_result = calibration_->encoder_phase().result();
  result.offset_mrad =
      static_cast<int32_t>(calibration_result.offset_rad * 1000.0f);
  result.residual_mrad =
      static_cast<int32_t>(calibration_result.residual_rad_rms * 1000.0f);
  result.sign = calibration_result.sign >= 0.0f ? 1 : -1;
  result.ok = calibration_result.ok ? 1 : 0;
  uint16_t samples = calibration_result.samples;
  if (calibration_->encoder_persisted() && samples < 0x8000u)
  {
    samples = static_cast<uint16_t>(samples | 0x8000u);
  }
  result.samples = samples;
  return result;
}

protocol::xt_can::CtrlReply TelemetryPublisher::BuildControlReply(
    uint8_t command, uint8_t sequence, uint8_t status,
    bool driver_fault) const
{
  protocol::xt_can::CtrlReply reply{};
  reply.hdr.seq = sequence;
  reply.cmd = command;
  reply.status = status;
  reply.flags = 0;
  if (motor_control_->output_enabled())
  {
    reply.flags = static_cast<uint16_t>(reply.flags |
                                        protocol::xt_can::kFlagPwmOn);
  }
  if (motor_control_->isr_enabled())
  {
    reply.flags = static_cast<uint16_t>(reply.flags |
                                        protocol::xt_can::kFlagCisr);
  }
  if (motor_control_->dq_valid())
  {
    reply.flags = static_cast<uint16_t>(reply.flags |
                                        protocol::xt_can::kFlagDqValid);
  }
  if (driver_fault || motor_control_->protection_tripped())
  {
    reply.flags = static_cast<uint16_t>(reply.flags |
                                        protocol::xt_can::kFlagFault);
  }
  if (encoder_->valid())
  {
    reply.flags = static_cast<uint16_t>(reply.flags |
                                        protocol::xt_can::kFlagEncOk);
  }
  const uint8_t mode = ModeId(motor_control_->mode());
  if (encoder_->valid() && encoder_->pll().theta_valid() &&
      (mode == protocol::xt_can::kModeServo ||
       mode == protocol::xt_can::kModeCurrent ||
       mode == protocol::xt_can::kModeMit))
  {
    reply.flags = static_cast<uint16_t>(reply.flags |
                                        protocol::xt_can::kFlagEncMode);
  }

  reply.mode = mode;
  reply.enc_ok = encoder_->valid() ? 1 : 0;
  reply.enc_sign = 1;
  const uint32_t spikes = encoder_->pll().spike_count();
  reply.enc_spike = spikes > 255u ? 255u : static_cast<uint8_t>(spikes);
  reply.id_mA = AmpsToMilli(motor_control_->id_A());
  reply.iq_mA = AmpsToMilli(motor_control_->iq_A());
  {
    const float bus_V = motor_control_->measured_bus_V() > 1.0f
                            ? motor_control_->measured_bus_V()
                            : runtime_config_->config().motor.bus_V;
    reply.bus_mV = static_cast<uint16_t>(bus_V * 1000.0f + 0.5f);
  }
  reply.theta_mech_mrad = static_cast<int32_t>(
      ((mode == protocol::xt_can::kModeMit && encoder_->pll().theta_valid())
           ? encoder_->pll().position_rad()
           : encoder_->sample().mechanical_rad) *
      1000.0f);

  if (encoder_->valid())
  {
    reply.enc_raw = encoder_->sample().raw;
    reply.enc_sign = encoder_->calibration().sign >= 0.0f ? 1 : -1;
    reply.theta_elec_mrad = static_cast<int32_t>(
        encoder_->sample().electrical_rad * 1000.0f);
  }

  const auto* foc = motor_control_->foc();
  const auto* dq_modulator = motor_control_->dq_modulator();
  if (foc != nullptr && foc->active())
  {
    reply.idref_mA = AmpsToMilli(foc->id_ref_A());
    reply.iqref_mA = AmpsToMilli(foc->iq_ref_A());
    reply.vd_mV = static_cast<int32_t>(foc->vd_V() * 1000.0f);
    reply.vq_mV = static_cast<int32_t>(foc->vq_V() * 1000.0f);
    reply.bus_mV = static_cast<uint16_t>(foc->bus_V() * 1000.0f + 0.5f);
    {
      const float headroom_cV = foc->voltage_headroom_V() * 100.0f;
      if (headroom_cV > 32767.0f)
      {
        reply.voltage_headroom_cV = 32767;
      }
      else if (headroom_cV < -32768.0f)
      {
        reply.voltage_headroom_cV = -32768;
      }
      else
      {
        reply.voltage_headroom_cV = static_cast<int16_t>(headroom_cV);
      }
    }
    reply.theta_elec_mrad = static_cast<int32_t>(
        (encoder_->valid() && encoder_->pll().theta_valid()
             ? encoder_->pll().electrical_theta()
             : foc->theta_rad()) *
        1000.0f);
  }
  else if (dq_modulator != nullptr && dq_modulator->active())
  {
    reply.theta_elec_mrad =
        static_cast<int32_t>(dq_modulator->theta_rad() * 1000.0f);
    reply.vd_mV =
        static_cast<int32_t>(dq_modulator->voltage_V() * 1000.0f);
    reply.bus_mV =
        static_cast<uint16_t>(dq_modulator->bus_V() * 1000.0f + 0.5f);
  }

  const auto* servo = motor_control_->servo();
  const auto* mit = motor_control_->mit();
  const float pole_pairs = runtime_config_->config().motor.pole_pairs;
  if (servo != nullptr && servo->active())
  {
    reply.omega_cmd_mrad_s =
        static_cast<int32_t>(servo->velocity_cmd() * 1000.0f);
  }
  else if (mit != nullptr && mit->active())
  {
    reply.omega_cmd_mrad_s =
        static_cast<int32_t>(mit->velocity_cmd_rad_s() * 1000.0f);
  }
  else if (foc != nullptr && foc->active())
  {
    reply.omega_cmd_mrad_s = static_cast<int32_t>(
        foc->theta_rate_rad_s() / pole_pairs * 1000.0f);
  }
  else if (dq_modulator != nullptr && dq_modulator->active())
  {
    reply.omega_cmd_mrad_s = static_cast<int32_t>(
        dq_modulator->theta_rate_rad_s() / pole_pairs * 1000.0f);
  }

  if (encoder_->valid() && encoder_->pll().theta_valid())
  {
    reply.omega_mech_mrad_s =
        static_cast<int32_t>(encoder_->pll().velocity_mech() * 1000.0f);
    reply.omega_elec_mrad_s =
        static_cast<int32_t>(encoder_->pll().omega_elec() * 1000.0f);
  }
  reply.fet_temp_dC = static_cast<int16_t>(-32768);
  if (motor_control_->fet_temp_ok())
  {
    const float dC = motor_control_->fet_temp_C() * 10.0f;
    if (dC > 32767.0f)
    {
      reply.fet_temp_dC = 32767;
    }
    else if (dC < -32767.0f)
    {
      reply.fet_temp_dC = static_cast<int16_t>(-32767);
    }
    else
    {
      reply.fet_temp_dC = static_cast<int16_t>(dC);
    }
  }

  return reply;
}

}  // namespace middleware::communication
