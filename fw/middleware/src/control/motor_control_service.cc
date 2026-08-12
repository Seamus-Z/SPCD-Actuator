#include "middleware/control/motor_control_service.h"

#include "math/constants.h"
#include "math/foc/transform.h"
#include "protocol/xt_can.h"

namespace middleware::control
{
namespace
{

uint8_t CalibrationKindId(CalibrationKind kind)
{
  switch (kind)
  {
    case CalibrationKind::EncoderPhase:
      return protocol::xt_can::kCalSubEncPhase;
    case CalibrationKind::EncoderLock:
      return protocol::xt_can::kCalSubEncLock;
    case CalibrationKind::Bemf:
      return protocol::xt_can::kCalSubBemf;
    case CalibrationKind::Resistance:
      return protocol::xt_can::kCalSubResistance;
    case CalibrationKind::Inductance:
      return protocol::xt_can::kCalSubInductance;
    case CalibrationKind::Cogging:
      return protocol::xt_can::kCalSubCogging;
    case CalibrationKind::Abort:
      return protocol::xt_can::kCalSubAbort;
  }
  return protocol::xt_can::kCalSubAbort;
}

}  // namespace

MotorControlService::MotorControlService(const Dependencies& dependencies)
    : timer_(dependencies.timer),
      gate_driver_(dependencies.gate_driver),
      current_adc_(dependencies.current_adc),
      phase_pwm_(dependencies.phase_pwm),
      dq_modulator_(dependencies.dq_modulator),
      foc_(dependencies.foc),
      servo_(dependencies.servo),
      mit_(dependencies.mit),
      encoder_(dependencies.encoder),
      calibration_(dependencies.calibration),
      snapshot_(dependencies.snapshot)
{
}

Result MotorControlService::PrimeFoc(float theta_rad, float id_A, float iq_A,
                                     const float* theta_override)
{
  if (foc_ == nullptr || current_adc_ == nullptr || phase_pwm_ == nullptr)
  {
    return Result::Failed;
  }

  math::foc::FocController::Command command;
  command.theta_rad = theta_rad;
  command.id_A = id_A;
  command.iq_A = iq_A;
  command.theta_rate_rad_s = 0.0f;
  foc_->Start(command);
  if (encoder_ != nullptr)
  {
    foc_->SetOmega(encoder_->pll().omega_elec(),
                   encoder_->pll().velocity_mech());
  }

  last_current_ = current_adc_->ReadLatest();
  math::foc::FocController::Duties duties;
  if (!foc_->Step(0.0f, last_current_.i1_A, last_current_.i2_A,
                  last_current_.i3_A, &duties, theta_override))
  {
    foc_->Stop();
    return Result::Failed;
  }
  id_A_ = foc_->id_A();
  iq_A_ = foc_->iq_A();
  dq_valid_ = last_current_.ok;
  phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
  return Result::Ok;
}

Result MotorControlService::StartServo(
    const math::servo_mode::ServoMode::Command& command_in)
{
  if (phase_pwm_ == nullptr || !phase_pwm_->init_ok())
  {
    return Result::NotReady;
  }
  if (encoder_ == nullptr || !encoder_->valid() || servo_ == nullptr ||
      foc_ == nullptr || gate_driver_ == nullptr || dq_modulator_ == nullptr)
  {
    return Result::Failed;
  }

  auto MapSingleTurnCommand = [this](float command_rad) -> float {
    if (!math::foc::IsFinite(command_rad))
    {
      return command_rad;
    }
    const float command = math::WrapZeroToTwoPi(command_rad);
    const float measured = encoder_->pll().position_rad();
    return measured + math::WrapNegPiToPi(
                          command - math::WrapZeroToTwoPi(measured));
  };

  math::servo_mode::ServoMode::Command command = command_in;
  command.position_rad = MapSingleTurnCommand(command.position_rad);
  command.stop_position_rad = MapSingleTurnCommand(command.stop_position_rad);

  if (mode_ == Mode::Servo && servo_->active() && foc_->active())
  {
    servo_->SetCommand(command);
    foc_->SetRefs(command.id_ref_A, foc_->iq_ref_A());
    foc_->SetOmega(encoder_->pll().omega_elec(),
                   encoder_->pll().velocity_mech());
    return Result::Ok;
  }

  Stop();
  encoder_->SampleBlocking();
  encoder_->ResetPll();
  command.position_rad = MapSingleTurnCommand(command_in.position_rad);
  command.stop_position_rad = MapSingleTurnCommand(command_in.stop_position_rad);
  servo_->Start(command);

  const float theta_electrical = encoder_->sample().electrical_rad;
  if (PrimeFoc(theta_electrical, command.id_ref_A, 0.0f,
               &theta_electrical) != Result::Ok)
  {
    servo_->Stop();
    return Result::Failed;
  }

  gate_driver_->PowerOn();
  output_enabled_ = true;
  mode_ = Mode::Servo;
  StartIsr();
  return Result::Ok;
}

Result MotorControlService::StartCurrent(float id_A, float iq_A)
{
  if (phase_pwm_ == nullptr || !phase_pwm_->init_ok())
  {
    return Result::NotReady;
  }
  if (encoder_ == nullptr || !encoder_->valid() || foc_ == nullptr ||
      gate_driver_ == nullptr)
  {
    return Result::Failed;
  }

  if (mode_ == Mode::Current && foc_->active())
  {
    foc_->SetRefs(id_A, iq_A);
    foc_->SetOmega(encoder_->pll().omega_elec(),
                   encoder_->pll().velocity_mech());
    return Result::Ok;
  }

  Stop();
  encoder_->SampleBlocking();
  encoder_->ResetPll();
  const float theta_electrical = encoder_->sample().electrical_rad;
  if (PrimeFoc(theta_electrical, id_A, iq_A, &theta_electrical) != Result::Ok)
  {
    return Result::Failed;
  }

  gate_driver_->PowerOn();
  output_enabled_ = true;
  mode_ = Mode::Current;
  StartIsr();
  return Result::Ok;
}

Result MotorControlService::StartMit(
    const math::servo_mode::MitMode::Command& command)
{
  if (phase_pwm_ == nullptr || !phase_pwm_->init_ok())
  {
    return Result::NotReady;
  }
  if (encoder_ == nullptr || !encoder_->valid() || mit_ == nullptr ||
      foc_ == nullptr || gate_driver_ == nullptr)
  {
    return Result::Failed;
  }

  if (mode_ == Mode::Mit && mit_->active() && foc_->active())
  {
    mit_->SetCommand(command);
    foc_->SetRefs(0.0f, foc_->iq_ref_A());
    foc_->SetOmega(encoder_->pll().omega_elec(),
                   encoder_->pll().velocity_mech());
    return Result::Ok;
  }

  Stop();
  encoder_->SampleBlocking();
  encoder_->ResetPll();
  mit_->Start(command);

  const float theta_electrical = encoder_->sample().electrical_rad;
  if (PrimeFoc(theta_electrical, 0.0f, 0.0f, &theta_electrical) != Result::Ok)
  {
    mit_->Stop();
    return Result::Failed;
  }

  gate_driver_->PowerOn();
  output_enabled_ = true;
  mode_ = Mode::Mit;
  StartIsr();
  return Result::Ok;
}

Result MotorControlService::StartCalibration(
    const CalibrationCommand& command)
{
  if (command.kind == CalibrationKind::Abort)
  {
    Stop();
    return Result::Ok;
  }
  if (phase_pwm_ == nullptr || !phase_pwm_->init_ok())
  {
    return Result::NotReady;
  }
  if (calibration_ == nullptr || current_adc_ == nullptr ||
      gate_driver_ == nullptr)
  {
    return Result::Failed;
  }

  if ((command.kind == CalibrationKind::EncoderPhase ||
       command.kind == CalibrationKind::EncoderLock ||
       command.kind == CalibrationKind::Bemf ||
       command.kind == CalibrationKind::Cogging) &&
      (encoder_ == nullptr || !encoder_->valid()))
  {
    return Result::Failed;
  }

  Stop();
  calibration_->SetLastKind(CalibrationKindId(command.kind));

  if (command.kind == CalibrationKind::Resistance)
  {
    if (foc_ == nullptr)
    {
      return Result::Failed;
    }
    math::calibration::RIdentCal::Options options;
    options.max_current_A = command.resistance_max_current_A > 0.05f
                                ? command.resistance_max_current_A
                                : 1.5f;
    if (command.resistance_points > 0)
    {
      options.n_points = command.resistance_points;
    }
    calibration_->resistance().Start(options);
    if (PrimeFoc(0.0f, calibration_->resistance().id_cmd_A(), 0.0f,
                 nullptr) != Result::Ok)
    {
      calibration_->resistance().Stop();
      return Result::Failed;
    }
  }
  else if (command.kind == CalibrationKind::Inductance)
  {
    if (dq_modulator_ == nullptr || foc_ == nullptr)
    {
      return Result::Failed;
    }
    math::calibration::LIdentCal::Options options;
    options.resistance_ohm = foc_->resistance_ohm();
    if (command.inductance_step_voltage_V > 0.0f)
    {
      options.step_voltage_V = command.inductance_step_voltage_V;
    }
    if (command.inductance_trials > 0)
    {
      options.n_trials = command.inductance_trials;
    }
    calibration_->inductance().Start(options);

    math::foc::DqModulator::Command dq_command;
    dq_command.theta_rad = 0.0f;
    dq_command.voltage_V = calibration_->inductance().voltage_d_cmd_V();
    dq_command.q_voltage_V = calibration_->inductance().voltage_q_cmd_V();
    dq_command.theta_rate_rad_s = 0.0f;
    dq_modulator_->Start(dq_command);
    math::foc::DqModulator::Duties duties;
    if (!dq_modulator_->Step(0.0f, &duties))
    {
      dq_modulator_->Stop();
      calibration_->inductance().Stop();
      return Result::Failed;
    }
    last_current_ = current_adc_->ReadLatest();
    ObserveDq(dq_modulator_->theta_rad(), last_current_);
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
  }
  else if (command.kind == CalibrationKind::Bemf ||
           command.kind == CalibrationKind::Cogging)
  {
    if (servo_ == nullptr || foc_ == nullptr ||
        !encoder_->pll().theta_valid())
    {
      return Result::Failed;
    }
    float velocity = 0.0f;
    if (command.kind == CalibrationKind::Bemf)
    {
      math::calibration::BemfIdentCal::Options options;
      options.resistance_ohm = foc_->resistance_ohm();
      options.max_speed_rad_s = command.bemf_max_speed_rad_s > 1.0f
                                    ? command.bemf_max_speed_rad_s
                                    : 60.0f;
      if (command.bemf_points > 0)
      {
        options.n_points = command.bemf_points;
      }
      calibration_->bemf().Start(options);
      velocity = calibration_->bemf().velocity_cmd_mech_rad_s();
    }
    else
    {
      math::calibration::CoggingCal::Options options;
      options.velocity_mech_rad_s = command.cogging_velocity_rad_s > 0.05f
                                        ? command.cogging_velocity_rad_s
                                        : 5.0f;
      if (command.cogging_record_revs > 0.0f)
      {
        options.record_revs = command.cogging_record_revs;
      }
      calibration_->cogging().Start(options);
      velocity = calibration_->cogging().velocity_cmd_mech_rad_s();
    }

    encoder_->SampleBlocking();
    encoder_->ResetPll();
    servo_->Start(math::foc::QuietNan(), velocity, 0.0f);
    const float theta_electrical = encoder_->sample().electrical_rad;
    if (PrimeFoc(theta_electrical, 0.0f, 0.0f,
                 &theta_electrical) != Result::Ok)
    {
      servo_->Stop();
      if (command.kind == CalibrationKind::Bemf)
      {
        calibration_->bemf().Stop();
      }
      else
      {
        calibration_->cogging().Stop();
      }
      return Result::Failed;
    }
  }
  else
  {
    if (foc_ == nullptr)
    {
      return Result::Failed;
    }
    math::calibration::EncoderPhaseCal::Options options;
    options.method = command.kind == CalibrationKind::EncoderLock
                         ? math::calibration::EncoderPhaseCal::Method::Lock
                         : math::calibration::EncoderPhaseCal::Method::Spin;
    options.current_A = command.encoder_current_A > 0.05f
                            ? command.encoder_current_A
                            : 1.0f;
    if (options.current_A > 2.0f)
    {
      options.current_A = 2.0f;
    }
    options.omega_elec_rad_s = command.encoder_electrical_speed_rad_s;
    if (options.omega_elec_rad_s < 0.0f)
    {
      options.omega_elec_rad_s = -options.omega_elec_rad_s;
    }
    options.pole_pairs = command.pole_pairs > 0.0f ? command.pole_pairs : 1.0f;
    options.mech_revs_each_way = 3.0f;
    if (command.kind == CalibrationKind::EncoderLock)
    {
      options.omega_elec_rad_s = 0.0f;
    }
    else
    {
      if (options.omega_elec_rad_s <= 1.0f)
      {
        options.omega_elec_rad_s = 40.0f;
      }
      if (options.omega_elec_rad_s > 80.0f)
      {
        options.omega_elec_rad_s = 80.0f;
      }
    }

    if (!calibration_->StartEncoderPhase(options))
    {
      return Result::Failed;
    }
    math::foc::FocController::Command foc_command;
    foc_command.theta_rad = 0.0f;
    foc_command.id_A = options.current_A;
    foc_command.iq_A = 0.0f;
    foc_command.theta_rate_rad_s =
        calibration_->encoder_phase().omega_cmd_elec();
    foc_->Start(foc_command);
    foc_->SetOmega(0.0f, 0.0f);
    last_current_ = current_adc_->ReadLatest();
    math::foc::FocController::Duties duties;
    if (!foc_->Step(0.0f, last_current_.i1_A, last_current_.i2_A,
                    last_current_.i3_A, &duties))
    {
      foc_->Stop();
      calibration_->encoder_phase().Stop();
      encoder_->RestoreCalibration();
      return Result::Failed;
    }
    id_A_ = foc_->id_A();
    iq_A_ = foc_->iq_A();
    dq_valid_ = last_current_.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
  }

  gate_driver_->PowerOn();
  output_enabled_ = true;
  mode_ = Mode::Calibration;
  StartIsr();
  return Result::Ok;
}

void MotorControlService::Stop()
{
  if (phase_pwm_ != nullptr)
  {
    phase_pwm_->DisableControlIsr();
  }
  if (snapshot_ != nullptr)
  {
    snapshot_->Abort();
  }
  last_control_us_ = 0;
  overcurrent_count_ = 0;
  if (calibration_ != nullptr)
  {
    calibration_->Abort();
  }
  if (dq_modulator_ != nullptr) dq_modulator_->Stop();
  if (servo_ != nullptr) servo_->Stop();
  if (mit_ != nullptr) mit_->Stop();
  if (foc_ != nullptr) foc_->Stop();
  if (phase_pwm_ != nullptr) phase_pwm_->Stop();
  if (gate_driver_ != nullptr) gate_driver_->PowerOff();
  output_enabled_ = false;
  dq_valid_ = false;
  mode_ = Mode::Stopped;
}

void MotorControlService::StopFinishedSession()
{
  if (dq_modulator_ != nullptr) dq_modulator_->Stop();
  if (servo_ != nullptr) servo_->Stop();
  if (mit_ != nullptr) mit_->Stop();
  if (foc_ != nullptr) foc_->Stop();
  if (gate_driver_ != nullptr) gate_driver_->PowerOff();
  if (phase_pwm_ != nullptr)
  {
    phase_pwm_->SetDuty(hal::PhasePwm::kDutyMinMilli,
                        hal::PhasePwm::kDutyMinMilli,
                        hal::PhasePwm::kDutyMinMilli);
  }
  output_enabled_ = false;
  mode_ = Mode::Stopped;
}

void MotorControlService::StartIsr()
{
  if (phase_pwm_ == nullptr || !phase_pwm_->init_ok())
  {
    return;
  }
  last_control_us_ = 0;
  phase_pwm_->EnableControlIsr(&MotorControlService::IsrThunk, this);
}

void MotorControlService::IsrThunk(void* context)
{
  if (context != nullptr)
  {
    static_cast<MotorControlService*>(context)->StepIsr();
  }
}

void MotorControlService::ObserveDq(
    float theta_rad, const hal::PhaseCurrentAdc::Sample& sample)
{
  if (!sample.ok)
  {
    dq_valid_ = false;
    return;
  }
  const math::SinCos sin_cos = math::SinCosFromRadians(theta_rad);
  const math::DqTransform dq(sin_cos, sample.i1_A, sample.i2_A, sample.i3_A);
  id_A_ = dq.d;
  iq_A_ = dq.q;
  dq_valid_ = true;
}

void MotorControlService::StepIsr()
{
  if (phase_pwm_ == nullptr || current_adc_ == nullptr || encoder_ == nullptr ||
      calibration_ == nullptr)
  {
    return;
  }

  const bool encoder_phase_active = calibration_->encoder_phase().active();
  const bool bemf_active = calibration_->bemf().active();
  const bool resistance_active = calibration_->resistance().active();
  const bool inductance_active = calibration_->inductance().active();
  const bool cogging_active = calibration_->cogging().active();
  const bool servo_active = mode_ == Mode::Servo && servo_ != nullptr &&
                            servo_->active() && !bemf_active && !cogging_active;
  const bool mit_active = mode_ == Mode::Mit && mit_ != nullptr &&
                          mit_->active() && foc_ != nullptr && foc_->active();
  const bool current_active = mode_ == Mode::Current && foc_ != nullptr &&
                              foc_->active();
  const bool control_active = encoder_phase_active || bemf_active ||
      resistance_active || inductance_active || cogging_active ||
      servo_active || mit_active || current_active;

  float dt_s = phase_pwm_->period_s();
  if (timer_ != nullptr)
  {
    const auto now = timer_->read_us();
    if (last_control_us_ != 0)
    {
      const uint32_t dt_us = static_cast<uint32_t>(
          hal::MillisecondTimer::subtract_us(now, last_control_us_));
      if (dt_us >= 20u && dt_us <= 500u)
      {
        dt_s = static_cast<float>(dt_us) * 1.0e-6f;
      }
    }
    last_control_us_ = now;
  }

  if (!control_active)
  {
    if (snapshot_ != nullptr && snapshot_->capturing())
    {
      last_current_ = current_adc_->ReadLatest();
      if (encoder_->valid()) encoder_->UpdatePwmIsr(dt_s);
      const uint32_t dt_us = static_cast<uint32_t>(dt_s * 1.0e6f + 0.5f);
      snapshot_->PushIsr(id_A_, iq_A_, last_current_.i1_A,
                         last_current_.i2_A, last_current_.i3_A,
                         encoder_->sample().mechanical_rad,
                         encoder_->sample().electrical_rad, dt_us);
    }
    else if (phase_pwm_->control_isr_on())
    {
      phase_pwm_->DisableControlIsr();
      last_control_us_ = 0;
    }
    return;
  }

  const auto sample = current_adc_->ReadLatest();
  last_current_ = sample;
  if (sample.ok && overcurrent_trip_A_ > 0.0f)
  {
    const float ia = sample.i1_A >= 0.0f ? sample.i1_A : -sample.i1_A;
    const float ib = sample.i2_A >= 0.0f ? sample.i2_A : -sample.i2_A;
    const float ic = sample.i3_A >= 0.0f ? sample.i3_A : -sample.i3_A;
    if (ia > overcurrent_trip_A_ || ib > overcurrent_trip_A_ ||
        ic > overcurrent_trip_A_)
    {
      if (++overcurrent_count_ >= kOvercurrentTripCount)
      {
        Stop();
        return;
      }
    }
    else
    {
      overcurrent_count_ = 0;
    }
  }

  if (encoder_->valid() || encoder_phase_active)
  {
    encoder_->UpdatePwmIsr(dt_s);
  }

  if (encoder_phase_active)
  {
    foc_->SetThetaRate(calibration_->encoder_phase().omega_cmd_elec());
    foc_->SetRefs(calibration_->encoder_phase().current_A(), 0.0f);
    math::foc::FocController::Duties duties;
    if (!foc_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A, &duties))
      return;
    id_A_ = foc_->id_A();
    iq_A_ = foc_->iq_A();
    dq_valid_ = sample.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    output_enabled_ = true;
    if (calibration_->encoder_phase().Step(
            dt_s, foc_->theta_rad(), encoder_->sample().counts_rad))
    {
      if (calibration_->encoder_phase().state() ==
          math::calibration::EncoderPhaseCal::State::Done)
      {
        calibration_->ApplyEncoderResult();
      }
      else
      {
        encoder_->RestoreCalibration();
      }
      StopFinishedSession();
    }
  }
  else if (bemf_active || cogging_active)
  {
    if (!encoder_->valid() || !encoder_->pll().theta_valid())
    {
      if (bemf_active) calibration_->bemf().Stop();
      if (cogging_active) calibration_->cogging().Stop();
      Stop();
      return;
    }
    const float velocity = bemf_active
        ? calibration_->bemf().velocity_cmd_mech_rad_s()
        : calibration_->cogging().velocity_cmd_mech_rad_s();
    servo_->SetVelocity(velocity);
    servo_->SetIdRef(0.0f);
    const float iq = servo_->Step(dt_s, encoder_->pll().position_rad(),
                                  encoder_->pll().velocity_mech());
    if (servo_->faulted())
    {
      Stop();
      return;
    }
    foc_->SetRefs(servo_->id_ref_A(), iq);
    foc_->SetOmega(encoder_->pll().omega_elec(),
                   encoder_->pll().velocity_mech());
    const float theta = encoder_->pll().electrical_theta();
    math::foc::FocController::Duties duties;
    if (!foc_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A,
                    &duties, &theta))
      return;
    id_A_ = foc_->id_A();
    iq_A_ = foc_->iq_A();
    dq_valid_ = sample.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    output_enabled_ = true;

    const bool finished = bemf_active
        ? calibration_->bemf().Step(dt_s, encoder_->pll().velocity_mech(),
                                    foc_->vq_V(), foc_->iq_A())
        : calibration_->cogging().Step(
              dt_s, encoder_->sample().mechanical_rad, iq);
    if (finished)
    {
      if (bemf_active && calibration_->bemf().state() ==
                             math::calibration::BemfIdentCal::State::Done)
      {
        calibration_->ApplyBemfResult();
        if (calibration_result_sink_ != nullptr)
          calibration_result_sink_->AcceptBemfCalibration();
      }
      if (cogging_active && calibration_->cogging().state() ==
                                math::calibration::CoggingCal::State::Done)
      {
        calibration_->ApplyCoggingResult();
      }
      StopFinishedSession();
    }
  }
  else if (resistance_active)
  {
    foc_->SetRefs(calibration_->resistance().id_cmd_A(), 0.0f);
    math::foc::FocController::Duties duties;
    if (!foc_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A,
                    &duties, nullptr))
      return;
    id_A_ = foc_->id_A();
    iq_A_ = foc_->iq_A();
    dq_valid_ = sample.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    output_enabled_ = true;
    if (calibration_->resistance().Step(dt_s, id_A_, foc_->vd_V()))
    {
      if (calibration_->resistance().state() ==
          math::calibration::RIdentCal::State::Done)
      {
        calibration_->ApplyResistanceResult();
        if (calibration_result_sink_ != nullptr)
          calibration_result_sink_->AcceptResistanceCalibration();
      }
      StopFinishedSession();
    }
  }
  else if (inductance_active)
  {
    dq_modulator_->SetDqVoltage(
        calibration_->inductance().voltage_d_cmd_V(),
        calibration_->inductance().voltage_q_cmd_V());
    math::foc::DqModulator::Duties duties;
    if (!dq_modulator_->Step(dt_s, &duties)) return;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    output_enabled_ = true;
    ObserveDq(dq_modulator_->theta_rad(), sample);
    if (calibration_->inductance().Step(dt_s, id_A_, iq_A_))
    {
      if (calibration_->inductance().state() ==
          math::calibration::LIdentCal::State::Done)
      {
        calibration_->ApplyInductanceResult();
        if (calibration_result_sink_ != nullptr)
          calibration_result_sink_->AcceptInductanceCalibration();
      }
      StopFinishedSession();
    }
  }
  else
  {
    if (!encoder_->valid() || !encoder_->pll().theta_valid())
    {
      Stop();
      return;
    }

    if (servo_active)
    {
      const float iq = servo_->Step(dt_s, encoder_->pll().position_rad(),
                                    encoder_->pll().velocity_mech());
      if (servo_->faulted())
      {
        Stop();
        return;
      }
      foc_->SetRefs(servo_->id_ref_A(),
                    iq + encoder_->CoggingCurrentCompensation());
    }
    else if (mit_active)
    {
      const float iq = mit_->Step(dt_s, encoder_->pll().position_rad(),
                                  encoder_->pll().velocity_mech());
      foc_->SetRefs(0.0f, iq + encoder_->CoggingCurrentCompensation());
    }

    foc_->SetOmega(encoder_->pll().omega_elec(),
                   encoder_->pll().velocity_mech());
    const float theta = encoder_->pll().electrical_theta();
    math::foc::FocController::Duties duties;
    if (!foc_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A,
                    &duties, &theta))
      return;
    id_A_ = foc_->id_A();
    iq_A_ = foc_->iq_A();
    dq_valid_ = sample.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    output_enabled_ = true;
  }

  if (snapshot_ != nullptr)
  {
    const uint32_t dt_us = static_cast<uint32_t>(dt_s * 1.0e6f + 0.5f);
    snapshot_->PushIsr(id_A_, iq_A_, sample.i1_A, sample.i2_A, sample.i3_A,
                       encoder_->sample().mechanical_rad,
                       encoder_->sample().electrical_rad, dt_us);
  }
}

}  // namespace middleware::control
