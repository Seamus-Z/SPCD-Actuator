#include "application.h"

#include <cstring>

#include "BL_Config.h"
#include "board_config.h"
#include "bootloader.h"
#include "math/constants.h"
#include "math/foc/transform.h"
#include "stm32g4xx.h"

namespace app
{
namespace
{

constexpr uint32_t kBootRequestId = 0x7E;
constexpr char kBootRequestPayload[] = "BOOT";
// Host streams Query/control at ~50 Hz; board replies with CtrlReply.
// The host streams control at 50 Hz.  Allow scheduler/CAN stalls without
// spuriously cycling velocity mode, while still disabling output on link loss.
constexpr uint32_t kCmdTimeoutUs = 500000;

void LedOn() { GPIOB->BSRR = 0x80000000; }
void LedOff() { GPIOB->BSRR = 0x00008000; }

void DelayNops(uint32_t count)
{
  for (volatile uint32_t d = 0; d < count; ++d)
  {
    __NOP();
  }
}

int32_t AmpsToMilli(float a)
{
  return static_cast<int32_t>(a * 1000.0f);
}

}  // namespace

Application::Application(::pool::Pool* pool)
    : pool_(pool),
      timer_(pool),
      can_(pool, board::CanOptions()),
      gate_driver_(pool, timer_.get(), board::GateDriverOptions()),
      current_adc_(pool, timer_.get(), board::CurrentSenseOptions()),
      phase_pwm_(pool, board::PhasePwmOptions()),
      dq_modulator_(pool, board::DqModulatorOptions()),
      foc_(pool, board::FocControllerOptions()),
      servo_mode_(pool, board::ServoModeOptions()),
      encoder_(pool, timer_.get(), board::EncoderOptions()),
      calibration_(&encoder_, foc_.get(), servo_mode_.get(), phase_pwm_.get()),
      binary_link_(pool, can_.get(), kDefaultCanId)
{
  binary_link_->SetCommandHandler(&Application::CommandThunk, this);
}

uint8_t Application::StartServo(
    const math::servo_mode::ServoMode::Command& command_in)
{
  if (state_ != State::RUN || phase_pwm_->init_ok() == false)
  {
    return telemetry::xt_can::kStatusNotRun;
  }
  if (!encoder_.valid() ||
      servo_mode_.get() == nullptr)
  {
    return telemetry::xt_can::kStatusFail;
  }

  // Finite position/stop commands are single-turn [0, 2π). Map onto the nearest
  // unwrapped target around the current PLL position so the outer loop still
  // sees a continuous mechanical trajectory (shortest-path).
  auto MapSingleTurnCommand = [this](float cmd_rad) -> float {
    if (!math::foc::IsFinite(cmd_rad))
    {
      return cmd_rad;
    }
    const float cmd = math::WrapZeroToTwoPi(cmd_rad);
    const float meas = encoder_.pll().position_rad();
    return meas + math::WrapNegPiToPi(cmd - math::WrapZeroToTwoPi(meas));
  };

  math::servo_mode::ServoMode::Command command = command_in;
  command.position_rad = MapSingleTurnCommand(command.position_rad);
  command.stop_position_rad = MapSingleTurnCommand(command.stop_position_rad);

  if (mode_ == telemetry::xt_can::kModeServo &&
      servo_mode_.get() != nullptr && servo_mode_->active() &&
      foc_.get() != nullptr && foc_->active())
  {
    servo_mode_->SetCommand(command);
    foc_->SetRefs(command.id_ref_A, foc_->iq_ref_A());
    foc_->SetOmega(encoder_.pll().omega_elec(),
                                  encoder_.pll().velocity_mech());
    return telemetry::xt_can::kStatusOk;
  }

  dq_modulator_->Stop();
  phase_pwm_->DisableControlIsr();

  encoder_.SampleBlocking();
  encoder_.ResetPll();
  // Remap after PLL reset so the first command is near the fresh unwrap.
  command.position_rad = MapSingleTurnCommand(command_in.position_rad);
  command.stop_position_rad = MapSingleTurnCommand(command_in.stop_position_rad);
  servo_mode_->Start(command);

  const float theta_elec = encoder_.sample().electrical_rad;
  math::foc::FocController::Command cmd;
  cmd.theta_rad = theta_elec;
  cmd.id_A = command.id_ref_A;
  cmd.iq_A = 0.0f;
  cmd.theta_rate_rad_s = 0.0f;
  foc_->Start(cmd);
  foc_->SetOmega(encoder_.pll().omega_elec(),
                                encoder_.pll().velocity_mech());

  last_current_ = current_adc_->ReadLatest();
  math::foc::FocController::Duties duties;
  if (foc_->Step(0.0f, last_current_.i1_A,
                                last_current_.i2_A,
                                last_current_.i3_A, &duties,
                                &theta_elec) == false)
  {
    foc_->Stop();
    servo_mode_->Stop();
    return telemetry::xt_can::kStatusFail;
  }
  id_A_ = foc_->id_A();
  iq_A_ = foc_->iq_A();
  dq_valid_ = last_current_.ok;
  phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
  gate_driver_->PowerOn();
  pwm_output_on_ = true;
  mode_ = telemetry::xt_can::kModeServo;
  StartControlIsr();
  return telemetry::xt_can::kStatusOk;
}

uint8_t Application::StartCurrent(float id_A, float iq_A)
{
  if (state_ != State::RUN || phase_pwm_->init_ok() == false)
  {
    return telemetry::xt_can::kStatusNotRun;
  }
  if (!encoder_.valid() || foc_.get() == nullptr)
  {
    return telemetry::xt_can::kStatusFail;
  }

  if (mode_ == telemetry::xt_can::kModeCurrent && foc_->active())
  {
    foc_->SetRefs(id_A, iq_A);
    foc_->SetOmega(encoder_.pll().omega_elec(),
                   encoder_.pll().velocity_mech());
    return telemetry::xt_can::kStatusOk;
  }

  if (servo_mode_.get() != nullptr)
  {
    servo_mode_->Stop();
  }
  dq_modulator_->Stop();
  phase_pwm_->DisableControlIsr();

  encoder_.SampleBlocking();
  encoder_.ResetPll();

  const float theta_elec = encoder_.sample().electrical_rad;
  math::foc::FocController::Command cmd;
  cmd.theta_rad = theta_elec;
  cmd.id_A = id_A;
  cmd.iq_A = iq_A;
  cmd.theta_rate_rad_s = 0.0f;
  foc_->Start(cmd);
  foc_->SetOmega(encoder_.pll().omega_elec(),
                 encoder_.pll().velocity_mech());

  last_current_ = current_adc_->ReadLatest();
  math::foc::FocController::Duties duties;
  if (foc_->Step(0.0f, last_current_.i1_A,
                 last_current_.i2_A,
                 last_current_.i3_A, &duties,
                 &theta_elec) == false)
  {
    foc_->Stop();
    return telemetry::xt_can::kStatusFail;
  }
  id_A_ = foc_->id_A();
  iq_A_ = foc_->iq_A();
  dq_valid_ = last_current_.ok;
  phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
  gate_driver_->PowerOn();
  pwm_output_on_ = true;
  mode_ = telemetry::xt_can::kModeCurrent;
  StartControlIsr();
  return telemetry::xt_can::kStatusOk;
}

void Application::Run()
{
  while (true)
  {
    switch (state_)
    {
      case State::INIT:
        state_ = Init() ? State::RUN : State::DRIVER_FAULT;
        break;
      case State::RUN:
        RunOnce();
        break;
      case State::DRIVER_FAULT:
        DriverFault();
        break;
      case State::ENTER_BOOTLOADER:
        EnterBootloaderMode();
        break;
    }
  }
}

bool Application::Init()
{
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE15_Msk) |
                 (1 << GPIO_MODER_MODE15_Pos);

  if (!gate_driver_->Init(board::GateDriverConfig()))
  {
    return false;
  }
  if (!current_adc_->Init())
  {
    return false;
  }
  if (!current_adc_->CalibrateOffset())
  {
    return false;
  }
  if (!phase_pwm_->Init())
  {
    return false;
  }
  if (!current_adc_->StartPwmSync())
  {
    return false;
  }
  phase_pwm_->EnableAdcTrigger();

  // Sensor faults are non-fatal at boot; closed-loop modes require encoder_.valid().
  (void)encoder_.Init();
  (void)calibration_.LoadPersistentConfig();

  telem_last_us_ = timer_->read_us();
  LedOn();
  return true;
}

void Application::RunOnce()
{
  const auto driver_status = gate_driver_->ReadStatus();
  if (driver_status.fault || driver_status.fault_line)
  {
    StopOutput();
    gate_driver_->Disable();
    state_ = State::DRIVER_FAULT;
    return;
  }

  calibration_.PersistPending();

  PollCan();
  MaybeCommandTimeout();
  MaybeSendSnapshot();
  LedOn();
}

void Application::DriverFault()
{
  StopOutput();
  PollCan();
  // Host Query still works in fault; no free-running telem.

  // Transient DRV faults leave nFAULT sticky after Disable(); try clear.
  if (gate_driver_.get() != nullptr)
  {
    gate_driver_->Enable();
    DelayNops(200000);
    const auto st = gate_driver_->ReadStatus();
    if (!st.fault && !st.fault_line)
    {
      state_ = State::RUN;
      LedOn();
      return;
    }
    gate_driver_->Disable();
  }

  LedOn();
  DelayNops(50000);
  LedOff();
  DelayNops(50000);
}

void Application::StopOutput()
{
  if (phase_pwm_.get() != nullptr)
  {
    phase_pwm_->DisableControlIsr();
  }
  // Abort in-flight snapshot so a later Capture is not stuck busy forever.
  if (snapshot_.busy())
  {
    snapshot_.Finish();
    snap_meta_sent_ = false;
  }
  last_control_us_ = 0;
  calibration_.Abort();
  if (dq_modulator_.get() != nullptr)
  {
    dq_modulator_->Stop();
  }
  if (servo_mode_.get() != nullptr)
  {
    servo_mode_->Stop();
  }
  if (foc_.get() != nullptr)
  {
    foc_->Stop();
  }
  if (phase_pwm_.get() != nullptr)
  {
    phase_pwm_->Stop();
  }
  if (gate_driver_.get() != nullptr)
  {
    gate_driver_->PowerOff();
  }
  pwm_output_on_ = false;
  dq_valid_ = false;
  mode_ = telemetry::xt_can::kModeStop;
}

void Application::StartControlIsr()
{
  if (phase_pwm_.get() == nullptr || !phase_pwm_->init_ok())
  {
    return;
  }
  last_control_us_ = 0;
    phase_pwm_->EnableControlIsr(&Application::ControlIsrThunk, this);
}

void Application::ControlIsrThunk(void* context)
{
  if (context == nullptr)
  {
    return;
  }
  static_cast<Application*>(context)->ControlIsrStep();
}

void Application::ObserveDqFromSample(float theta_rad,
                                      const hal::PhaseCurrentAdc::Sample& s)
{
  if (!s.ok)
  {
    dq_valid_ = false;
    return;
  }
  const math::SinCos sc = math::SinCosFromRadians(theta_rad);
  const math::DqTransform dq(sc, s.i1_A, s.i2_A, s.i3_A);
  id_A_ = dq.d;
  iq_A_ = dq.q;
  dq_valid_ = true;
}

void Application::ControlIsrStep()
{
  const bool cal_on = calibration_.encoder_phase().active();
  const bool bemf_on = calibration_.bemf().active();
  const bool r_on = calibration_.resistance().active();
  const bool l_on = calibration_.inductance().active();
  const bool cogging_on = calibration_.cogging().active();
  const bool vel_on =
      servo_mode_.get() != nullptr && servo_mode_->active() &&
      !bemf_on && !cogging_on;
  const bool current_on =
      mode_ == telemetry::xt_can::kModeCurrent &&
      foc_.get() != nullptr && foc_->active() &&
      !bemf_on && !cogging_on && !cal_on && !r_on && !l_on;

  if (phase_pwm_.get() == nullptr || current_adc_.get() == nullptr)
  {
    return;
  }

  const bool control_active =
      vel_on || current_on || cal_on || bemf_on || r_on || l_on || cogging_on;

  // Prefer measured ISR period. Nominal period_s() can disagree with real UEV
  // spacing; that scales PLL ω and makes vel PID think it is already tracking.
  float dt_s = phase_pwm_->period_s();
  if (timer_.get() != nullptr)
  {
    const auto now = timer_->read_us();
    if (last_control_us_ != 0)
    {
      const uint32_t dt_us = static_cast<uint32_t>(
          hal::MillisecondTimer::subtract_us(now, last_control_us_));
      // 15 kHz -> ~67 us; clamp coalesced/stalled IRQs so ω stays sane.
      if (dt_us >= 20u && dt_us <= 500u)
      {
        dt_s = static_cast<float>(dt_us) * 1.0e-6f;
      }
    }
    last_control_us_ = now;
  }

  // Calibration/mode end paths historically left TIM5 ISR enabled with every
  // loop inactive. Keep snapshot capture alive in that window; otherwise stop
  // the zombie ISR so Snap no longer arms into a permanent Capturing hang.
  if (!control_active)
  {
    if (snapshot_.state() == SnapshotCapture::State::Capturing)
    {
      const auto sample = current_adc_->ReadLatest();
      last_current_ = sample;
      if (encoder_.valid())
      {
        encoder_.UpdatePwmIsr(dt_s);
      }
      const uint32_t dt_us = static_cast<uint32_t>(dt_s * 1.0e6f + 0.5f);
      snapshot_.PushIsr(id_A_, iq_A_, sample.i1_A, sample.i2_A, sample.i3_A,
                        encoder_.sample().mechanical_rad, encoder_.sample().electrical_rad, dt_us);
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

  if (encoder_.valid() || cal_on)
  {
    encoder_.UpdatePwmIsr(dt_s);
  }

  if (cal_on)
  {
    foc_->SetThetaRate(calibration_.encoder_phase().omega_cmd_elec());
    foc_->SetRefs(calibration_.encoder_phase().current_A(), 0.0f);
    math::foc::FocController::Duties duties;
    if (!foc_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A,
                             &duties))
    {
      return;
    }
    id_A_ = foc_->id_A();
    iq_A_ = foc_->iq_A();
    dq_valid_ = sample.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    pwm_output_on_ = true;

    const bool finished = calibration_.encoder_phase().Step(
        dt_s, foc_->theta_rad(), encoder_.sample().counts_rad);
    if (finished)
    {
      if (calibration_.encoder_phase().state() == math::calibration::EncoderPhaseCal::State::Done)
      {
        calibration_.ApplyEncoderResult();
      }
      else
      {
        encoder_.RestoreCalibration();
      }
      foc_->Stop();
      gate_driver_->PowerOff();
      pwm_output_on_ = false;
      mode_ = telemetry::xt_can::kModeStop;
      phase_pwm_->SetDuty(hal::PhasePwm::kDutyMinMilli,
                          hal::PhasePwm::kDutyMinMilli,
                          hal::PhasePwm::kDutyMinMilli);
    }
  }
  else if (bemf_on)
  {
    // Ke identification: closed-loop velocity sweep, Id=0, regress Vq/Iq.
    if (!encoder_.valid() || !encoder_.pll().theta_valid())
    {
      calibration_.bemf().Stop();
      servo_mode_->Stop();
      foc_->Stop();
      if (gate_driver_.get() != nullptr)
      {
        gate_driver_->PowerOff();
      }
      pwm_output_on_ = false;
      dq_valid_ = false;
      mode_ = telemetry::xt_can::kModeStop;
      return;
    }
    servo_mode_->SetVelocity(calibration_.bemf().velocity_cmd_mech_rad_s());
    servo_mode_->SetIdRef(0.0f);
    const float iq = servo_mode_->Step(
        dt_s, encoder_.pll().position_rad(), encoder_.pll().velocity_mech());
    if (servo_mode_->faulted())
    {
      calibration_.bemf().Stop();
      StopOutput();
      return;
    }
    foc_->SetRefs(servo_mode_->id_ref_A(), iq);
    // Use measured speed for BEMF/phase-lead (same as dq). Command-speed FF
    // over-leads the lagging direction and turns a small encoder-cal bias into
    // a hard one-sided speed ceiling that flips after recalibration.
    const float w_meas = encoder_.pll().velocity_mech();
    foc_->SetOmega(encoder_.pll().omega_elec(), w_meas);
    const float theta_elec = encoder_.pll().electrical_theta();
    math::foc::FocController::Duties duties;
    if (!foc_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A,
                             &duties, &theta_elec))
    {
      return;
    }
    id_A_ = foc_->id_A();
    iq_A_ = foc_->iq_A();
    dq_valid_ = sample.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    pwm_output_on_ = true;

    const bool finished = calibration_.bemf().Step(dt_s, w_meas, foc_->vq_V(),
                                         foc_->iq_A());
    if (finished)
    {
      if (calibration_.bemf().state() == math::calibration::BemfIdentCal::State::Done)
      {
        calibration_.ApplyBemfResult();
      }
      servo_mode_->Stop();
      foc_->Stop();
      gate_driver_->PowerOff();
      pwm_output_on_ = false;
      mode_ = telemetry::xt_can::kModeStop;
      phase_pwm_->SetDuty(hal::PhasePwm::kDutyMinMilli,
                          hal::PhasePwm::kDutyMinMilli,
                          hal::PhasePwm::kDutyMinMilli);
    }
  }
  else if (r_on)
  {
    foc_->SetRefs(calibration_.resistance().id_cmd_A(), 0.0f);
    math::foc::FocController::Duties duties;
    if (!foc_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A,
                             &duties, nullptr))
    {
      return;
    }
    id_A_ = foc_->id_A();
    iq_A_ = foc_->iq_A();
    dq_valid_ = sample.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    pwm_output_on_ = true;

    const bool finished = calibration_.resistance().Step(dt_s, id_A_, foc_->vd_V());
    if (finished)
    {
      if (calibration_.resistance().state() == math::calibration::RIdentCal::State::Done)
      {
        calibration_.ApplyResistanceResult();
      }
      foc_->Stop();
      gate_driver_->PowerOff();
      pwm_output_on_ = false;
      mode_ = telemetry::xt_can::kModeStop;
      phase_pwm_->SetDuty(hal::PhasePwm::kDutyMinMilli,
                          hal::PhasePwm::kDutyMinMilli,
                          hal::PhasePwm::kDutyMinMilli);
    }
  }
  else if (l_on)
  {
    dq_modulator_->SetDqVoltage(calibration_.inductance().voltage_d_cmd_V(),
                               calibration_.inductance().voltage_q_cmd_V());
    math::foc::DqModulator::Duties duties;
    if (!dq_modulator_->Step(dt_s, &duties))
    {
      return;
    }
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    pwm_output_on_ = true;
    ObserveDqFromSample(dq_modulator_->theta_rad(), sample);

    const bool finished = calibration_.inductance().Step(dt_s, id_A_, iq_A_);
    if (finished)
    {
      if (calibration_.inductance().state() == math::calibration::LIdentCal::State::Done)
      {
        calibration_.ApplyInductanceResult();
      }
      dq_modulator_->Stop();
      gate_driver_->PowerOff();
      pwm_output_on_ = false;
      mode_ = telemetry::xt_can::kModeStop;
      phase_pwm_->SetDuty(hal::PhasePwm::kDutyMinMilli,
                          hal::PhasePwm::kDutyMinMilli,
                          hal::PhasePwm::kDutyMinMilli);
    }
  }
  else if (cogging_on)
  {
    // Cogging map: run the velocity loop at a slow constant speed (fwd then
    // rev), record the torque current per rotor position. No cogging FF is
    // injected here — we are measuring the raw ripple.
    if (!encoder_.valid() || !encoder_.pll().theta_valid())
    {
      calibration_.cogging().Stop();
      servo_mode_->Stop();
      foc_->Stop();
      if (gate_driver_.get() != nullptr)
      {
        gate_driver_->PowerOff();
      }
      pwm_output_on_ = false;
      dq_valid_ = false;
      mode_ = telemetry::xt_can::kModeStop;
      return;
    }
    servo_mode_->SetVelocity(calibration_.cogging().velocity_cmd_mech_rad_s());
    servo_mode_->SetIdRef(0.0f);
    const float iq = servo_mode_->Step(
        dt_s, encoder_.pll().position_rad(), encoder_.pll().velocity_mech());
    if (servo_mode_->faulted())
    {
      calibration_.cogging().Stop();
      StopOutput();
      return;
    }
    foc_->SetRefs(servo_mode_->id_ref_A(), iq);
    foc_->SetOmega(encoder_.pll().omega_elec(),
                            encoder_.pll().velocity_mech());
    const float theta_elec = encoder_.pll().electrical_theta();
    math::foc::FocController::Duties duties;
    if (!foc_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A,
                             &duties, &theta_elec))
    {
      return;
    }
    id_A_ = foc_->id_A();
    iq_A_ = foc_->iq_A();
    dq_valid_ = sample.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    pwm_output_on_ = true;

    const bool finished = calibration_.cogging().Step(dt_s, encoder_.sample().mechanical_rad, iq);
    if (finished)
    {
      if (calibration_.cogging().state() == math::calibration::CoggingCal::State::Done)
      {
        calibration_.ApplyCoggingResult();
      }
      servo_mode_->Stop();
      foc_->Stop();
      gate_driver_->PowerOff();
      pwm_output_on_ = false;
      mode_ = telemetry::xt_can::kModeStop;
      phase_pwm_->SetDuty(hal::PhasePwm::kDutyMinMilli,
                          hal::PhasePwm::kDutyMinMilli,
                          hal::PhasePwm::kDutyMinMilli);
    }
  }
  else if (current_on)
  {
    // Direct Id/Iq current mode: encoder θ_e commutation, no outer servo.
    if (!encoder_.valid() || !encoder_.pll().theta_valid())
    {
      foc_->Stop();
      if (gate_driver_.get() != nullptr)
      {
        gate_driver_->PowerOff();
      }
      pwm_output_on_ = false;
      dq_valid_ = false;
      mode_ = telemetry::xt_can::kModeStop;
      return;
    }
    foc_->SetOmega(encoder_.pll().omega_elec(),
                            encoder_.pll().velocity_mech());
    const float theta_elec = encoder_.pll().electrical_theta();
    math::foc::FocController::Duties duties;
    if (!foc_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A,
                             &duties, &theta_elec))
    {
      return;
    }
    id_A_ = foc_->id_A();
    iq_A_ = foc_->iq_A();
    dq_valid_ = sample.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    pwm_output_on_ = true;
  }
  else if (vel_on)
  {
    // moteus position/velocity: PID → Iq, encoder θ_e commutation.
    if (!encoder_.valid() || !encoder_.pll().theta_valid())
    {
      servo_mode_->Stop();
      foc_->Stop();
      if (gate_driver_.get() != nullptr)
      {
        gate_driver_->PowerOff();
      }
      pwm_output_on_ = false;
      dq_valid_ = false;
      mode_ = telemetry::xt_can::kModeStop;
      return;
    }
    const float iq = servo_mode_->Step(
        dt_s, encoder_.pll().position_rad(), encoder_.pll().velocity_mech());
    if (servo_mode_->faulted())
    {
      StopOutput();
      return;
    }
    foc_->SetRefs(servo_mode_->id_ref_A(), iq + encoder_.CoggingCurrentCompensation());
    // Measured ω for FF/lead. Using control_velocity here made a static
    // encoder-cal phase bias choose which direction could reach high speed.
    foc_->SetOmega(encoder_.pll().omega_elec(),
                            encoder_.pll().velocity_mech());
    const float theta_elec = encoder_.pll().electrical_theta();
    math::foc::FocController::Duties duties;
    if (!foc_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A,
                             &duties, &theta_elec))
    {
      return;
    }
    id_A_ = foc_->id_A();
    iq_A_ = foc_->iq_A();
    dq_valid_ = sample.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    pwm_output_on_ = true;
  }


  const uint32_t dt_us = static_cast<uint32_t>(dt_s * 1.0e6f + 0.5f);
  snapshot_.PushIsr(id_A_, iq_A_, sample.i1_A, sample.i2_A, sample.i3_A,
                    encoder_.sample().mechanical_rad, encoder_.sample().electrical_rad, dt_us);
}

void Application::PollCan()
{
  const auto st = can_->status();
  if (st.BusOff)
  {
    can_->RecoverBusOff();
    return;
  }

  for (int i = 0; i < 8; ++i)
  {
    FDCAN_RxHeaderTypeDef header = {};
    uint8_t data[64] = {};
    size_t len = 0;
    if (!can_->Poll(&header, data, sizeof(data), &len))
    {
      break;
    }
    if (timer_.get() != nullptr)
    {
      last_rx_us_ = timer_->read_us();
    }

    if (header.IdType == FDCAN_STANDARD_ID &&
        header.Identifier == kBootRequestId &&
        header.RxFrameType == FDCAN_DATA_FRAME &&
        len == sizeof(kBootRequestPayload) - 1 &&
        std::memcmp(data, kBootRequestPayload,
                    sizeof(kBootRequestPayload) - 1) == 0)
    {
      state_ = State::ENTER_BOOTLOADER;
      return;
    }

    (void)binary_link_->HandleFrame(header, data, len);
  }
}

telemetry::xt_can::Telemetry Application::BuildTelemetry() const
{
  telemetry::xt_can::Telemetry t{};
  t.flags = 0;
  if (pwm_output_on_)
  {
    t.flags = static_cast<uint16_t>(t.flags | telemetry::xt_can::kFlagPwmOn);
  }
  if (phase_pwm_.get() != nullptr && phase_pwm_->control_isr_on())
  {
    t.flags = static_cast<uint16_t>(t.flags | telemetry::xt_can::kFlagCisr);
  }
  if (dq_valid_)
  {
    t.flags = static_cast<uint16_t>(t.flags | telemetry::xt_can::kFlagDqValid);
  }
  if (state_ == State::DRIVER_FAULT)
  {
    t.flags = static_cast<uint16_t>(t.flags | telemetry::xt_can::kFlagFault);
  }
  if (encoder_.valid())
  {
    t.flags = static_cast<uint16_t>(t.flags | telemetry::xt_can::kFlagEncOk);
  }
  if (encoder_.valid() && encoder_.pll().theta_valid() &&
      (mode_ == telemetry::xt_can::kModeServo ||
       mode_ == telemetry::xt_can::kModeCurrent))
  {
    t.flags = static_cast<uint16_t>(t.flags | telemetry::xt_can::kFlagEncMode);
  }

  t.id_mA = AmpsToMilli(id_A_);
  t.iq_mA = AmpsToMilli(iq_A_);
  t.i1_mA = AmpsToMilli(last_current_.i1_A);
  t.i2_mA = AmpsToMilli(last_current_.i2_A);
  t.i3_mA = AmpsToMilli(last_current_.i3_A);

  if (foc_.get() != nullptr && foc_->active())
  {
    t.idref_mA = AmpsToMilli(foc_->id_ref_A());
    t.iqref_mA = AmpsToMilli(foc_->iq_ref_A());
    if (encoder_.valid() && encoder_.pll().theta_valid())
    {
      t.theta_mrad =
          static_cast<int32_t>(encoder_.pll().electrical_theta() * 1000.0f);
      t.omega_mrad_s =
          static_cast<int32_t>(encoder_.pll().velocity_mech() * 1000.0f);
    }
    else
    {
      t.theta_mrad = static_cast<int32_t>(foc_->theta_rad() * 1000.0f);
      t.omega_mrad_s = static_cast<int32_t>(
          foc_->theta_rate_rad_s() /
          board::MotorParams().pole_pairs * 1000.0f);
    }
    t.vd_mV = static_cast<int32_t>(foc_->vd_V() * 1000.0f);
    t.vq_mV = static_cast<int32_t>(foc_->vq_V() * 1000.0f);
    t.bus_mV = static_cast<uint16_t>(foc_->bus_V() * 1000.0f + 0.5f);
  }
  else if (dq_modulator_.get() != nullptr && dq_modulator_->active())
  {
    t.theta_mrad = static_cast<int32_t>(dq_modulator_->theta_rad() * 1000.0f);
    t.omega_mrad_s = static_cast<int32_t>(
        dq_modulator_->theta_rate_rad_s() /
        board::MotorParams().pole_pairs * 1000.0f);
    t.vd_mV = static_cast<int32_t>(dq_modulator_->voltage_V() * 1000.0f);
    t.vq_mV = 0;
    t.bus_mV = static_cast<uint16_t>(dq_modulator_->bus_V() * 1000.0f + 0.5f);
  }
  else
  {
    t.bus_mV = static_cast<uint16_t>(board::kBusVoltage_V * 1000.0f + 0.5f);
  }

  if (phase_pwm_.get() != nullptr)
  {
    t.duty_a = phase_pwm_->duty_a();
    t.duty_b = phase_pwm_->duty_b();
    t.duty_c = phase_pwm_->duty_c();
  }
  t.mode = mode_;
  return t;
}

void Application::MaybeCommandTimeout()
{
  if (timer_.get() == nullptr || last_rx_us_ == 0)
  {
    return;
  }
  if (mode_ == telemetry::xt_can::kModeStop)
  {
    return;
  }
  // Host pauses Live streaming while a snapshot dump is in flight; do not
  // treat that quiet period as link loss or we abort the dump mid-send.
  if (snapshot_.busy())
  {
    return;
  }
  const auto now = timer_->read_us();
  const auto since_rx = static_cast<uint32_t>(
      hal::MillisecondTimer::subtract_us(now, last_rx_us_));
  if (since_rx >= kCmdTimeoutUs)
  {
    StopOutput();
  }
}

void Application::ReplyCtrl(uint8_t cmd, uint8_t seq, uint8_t status)
{
  if (binary_link_.get() == nullptr)
  {
    return;
  }
  // Snapshot TX owns the bus; skip Live reply until burst finishes.
  if (snapshot_.busy())
  {
    return;
  }
  if (phase_pwm_.get() == nullptr || !phase_pwm_->control_isr_on())
  {
    encoder_.SampleBlocking();
  }
  binary_link_->SendCtrlReply(BuildCtrlReply(cmd, seq, status));
  if (calibration_.encoder_phase().state() != math::calibration::EncoderPhaseCal::State::Idle ||
      calibration_.bemf().state() != math::calibration::BemfIdentCal::State::Idle ||
      calibration_.resistance().state() != math::calibration::RIdentCal::State::Idle ||
      calibration_.inductance().state() != math::calibration::LIdentCal::State::Idle ||
      calibration_.cogging().state() != math::calibration::CoggingCal::State::Idle)
  {
    binary_link_->SendCalTelem(BuildCalTelem());
  }
}

void Application::MaybeSendTelemetry()
{
  // Free-running Tel/Enc disabled — host Query/control pulls CtrlReply.
}


telemetry::xt_can::CalTelem Application::BuildCalTelem() const
{
  if (calibration_.last_kind() == telemetry::xt_can::kCalSubBemf)
  {
    telemetry::xt_can::CalTelem c{};
    c.kind = telemetry::xt_can::kCalSubBemf;
    switch (calibration_.bemf().state())
    {
      case math::calibration::BemfIdentCal::State::Running:
        c.state = telemetry::xt_can::kCalStateBemfRun;
        break;
      case math::calibration::BemfIdentCal::State::Done:
        c.state = telemetry::xt_can::kCalStateDone;
        break;
      case math::calibration::BemfIdentCal::State::Failed:
        c.state = telemetry::xt_can::kCalStateFailed;
        break;
      default:
        c.state = telemetry::xt_can::kCalStateIdle;
        break;
    }
    c.progress_pm = calibration_.bemf().progress_permille();
    const auto& br = calibration_.bemf().result();
    c.offset_mrad = static_cast<int32_t>(br.ke_v_s_per_rad * 1.0e6f);
    c.residual_mrad = static_cast<int32_t>(br.r2 * 1.0e6f);
    c.sign = 0;
    c.ok = br.ok ? 1 : 0;
    {
      uint16_t samples = br.points_used;
      if (calibration_.bemf_persisted() && samples < 0x8000)
      {
        samples = static_cast<uint16_t>(samples | 0x8000u);
      }
      c.samples = samples;
    }
    return c;
  }
  if (calibration_.last_kind() == telemetry::xt_can::kCalSubResistance)
  {
    telemetry::xt_can::CalTelem c{};
    c.kind = telemetry::xt_can::kCalSubResistance;
    switch (calibration_.resistance().state())
    {
      case math::calibration::RIdentCal::State::Running:
        c.state = telemetry::xt_can::kCalStateRRun;
        break;
      case math::calibration::RIdentCal::State::Done:
        c.state = telemetry::xt_can::kCalStateDone;
        break;
      case math::calibration::RIdentCal::State::Failed:
        c.state = telemetry::xt_can::kCalStateFailed;
        break;
      default:
        c.state = telemetry::xt_can::kCalStateIdle;
        break;
    }
    c.progress_pm = calibration_.resistance().progress_permille();
    const auto& rr = calibration_.resistance().result();
    c.offset_mrad = static_cast<int32_t>(rr.resistance_ohm * 1.0e6f);
    c.residual_mrad = static_cast<int32_t>(rr.r2 * 1.0e6f);
    c.sign = 0;
    c.ok = rr.ok ? 1 : 0;
    {
      uint16_t samples = rr.points_used;
      if (calibration_.resistance_persisted() && samples < 0x8000)
      {
        samples = static_cast<uint16_t>(samples | 0x8000u);
      }
      c.samples = samples;
    }
    return c;
  }
  if (calibration_.last_kind() == telemetry::xt_can::kCalSubInductance)
  {
    telemetry::xt_can::CalTelem c{};
    c.kind = telemetry::xt_can::kCalSubInductance;
    switch (calibration_.inductance().state())
    {
      case math::calibration::LIdentCal::State::Running:
        c.state = telemetry::xt_can::kCalStateLRun;
        break;
      case math::calibration::LIdentCal::State::Done:
        c.state = telemetry::xt_can::kCalStateDone;
        break;
      case math::calibration::LIdentCal::State::Failed:
        c.state = telemetry::xt_can::kCalStateFailed;
        break;
      default:
        c.state = telemetry::xt_can::kCalStateIdle;
        break;
    }
    c.progress_pm = calibration_.inductance().progress_permille();
    const auto& lr = calibration_.inductance().result();
    c.offset_mrad = static_cast<int32_t>(lr.inductance_d_H * 1.0e9f);
    c.residual_mrad = static_cast<int32_t>(lr.inductance_q_H * 1.0e9f);
    c.sign = 0;
    c.ok = lr.ok ? 1 : 0;
    {
      uint16_t samples =
          static_cast<uint16_t>(lr.trials_d_used + lr.trials_q_used);
      if (calibration_.inductance_persisted() && samples < 0x8000)
      {
        samples = static_cast<uint16_t>(samples | 0x8000u);
      }
      c.samples = samples;
    }
    return c;
  }
  if (calibration_.last_kind() == telemetry::xt_can::kCalSubCogging)
  {
    telemetry::xt_can::CalTelem c{};
    c.kind = telemetry::xt_can::kCalSubCogging;
    switch (calibration_.cogging().state())
    {
      case math::calibration::CoggingCal::State::Running:
        c.state = telemetry::xt_can::kCalStateCoggingRun;
        break;
      case math::calibration::CoggingCal::State::Done:
        c.state = telemetry::xt_can::kCalStateDone;
        break;
      case math::calibration::CoggingCal::State::Failed:
        c.state = telemetry::xt_can::kCalStateFailed;
        break;
      default:
        c.state = telemetry::xt_can::kCalStateIdle;
        break;
    }
    c.progress_pm = calibration_.cogging().progress_permille();
    const auto& cr = calibration_.cogging().result();
    c.offset_mrad = static_cast<int32_t>(cr.scale * 1.0e6f);
    c.residual_mrad = static_cast<int32_t>(cr.peak_A * 1.0e6f);
    c.sign = 0;
    c.ok = cr.ok ? 1 : 0;
    {
      uint16_t samples = calibration_.cogging_persisted() ? 0x8001u : 1u;
      c.samples = samples;
    }
    return c;
  }
  telemetry::xt_can::CalTelem c{};
  c.kind = static_cast<uint8_t>(
      calibration_.encoder_phase().method() == math::calibration::EncoderPhaseCal::Method::Lock
          ? telemetry::xt_can::kCalSubEncLock
          : telemetry::xt_can::kCalSubEncPhase);
  c.state = static_cast<uint8_t>(calibration_.encoder_phase().state());
  c.progress_pm = calibration_.encoder_phase().progress_permille();
  const auto& r = calibration_.encoder_phase().result();
  c.offset_mrad = static_cast<int32_t>(r.offset_rad * 1000.0f);
  c.residual_mrad = static_cast<int32_t>(r.residual_rad_rms * 1000.0f);
  c.sign = (r.sign >= 0.0f) ? 1 : -1;
  c.ok = r.ok ? 1 : 0;
  uint16_t samples = r.samples;
  if (calibration_.encoder_persisted() && samples < 0x8000)
  {
    samples = static_cast<uint16_t>(samples | 0x8000u);
  }
  c.samples = samples;
  return c;
}


telemetry::xt_can::EncTelem Application::BuildEncTelem() const
{
  telemetry::xt_can::EncTelem e{};
  e.raw = 0;
  e.theta_mech_mrad = 0;
  e.theta_elec_mrad = 0;
  e.sign = 1;
  e.ok = 0;
  e.reserved[0] = 0;
  e.reserved[1] = 0;
  if (!encoder_.valid())
  {
    return e;
  }
  const auto& encoder_sample = encoder_.sample();
  e.raw = encoder_sample.raw;
  e.theta_mech_mrad =
      static_cast<int32_t>(encoder_sample.mechanical_rad * 1000.0f);
  e.theta_elec_mrad =
      static_cast<int32_t>(encoder_sample.electrical_rad * 1000.0f);
  e.sign = encoder_.calibration().sign >= 0.0f ? 1 : -1;
  e.ok = 1;
  return e;
}

telemetry::xt_can::CtrlReply Application::BuildCtrlReply(uint8_t cmd, uint8_t seq,
                                                         uint8_t status) const
{
  telemetry::xt_can::CtrlReply r{};
  r.hdr.seq = seq;
  r.cmd = cmd;
  r.status = status;
  r.flags = 0;
  if (pwm_output_on_)
  {
    r.flags = static_cast<uint16_t>(r.flags | telemetry::xt_can::kFlagPwmOn);
  }
  if (phase_pwm_.get() != nullptr && phase_pwm_->control_isr_on())
  {
    r.flags = static_cast<uint16_t>(r.flags | telemetry::xt_can::kFlagCisr);
  }
  if (dq_valid_)
  {
    r.flags = static_cast<uint16_t>(r.flags | telemetry::xt_can::kFlagDqValid);
  }
  if (state_ == State::DRIVER_FAULT)
  {
    r.flags = static_cast<uint16_t>(r.flags | telemetry::xt_can::kFlagFault);
  }
  if (encoder_.valid())
  {
    r.flags = static_cast<uint16_t>(r.flags | telemetry::xt_can::kFlagEncOk);
  }
  if (encoder_.valid() && encoder_.pll().theta_valid() &&
      (mode_ == telemetry::xt_can::kModeServo ||
       mode_ == telemetry::xt_can::kModeCurrent))
  {
    r.flags = static_cast<uint16_t>(r.flags | telemetry::xt_can::kFlagEncMode);
  }

  r.mode = mode_;
  r.enc_ok = encoder_.valid() ? 1 : 0;
  r.enc_sign = 1;
  {
    const uint32_t spikes = encoder_.pll().spike_count();
    r.enc_spike = (spikes > 255u) ? 255u : static_cast<uint8_t>(spikes);
  }
  r.id_mA = AmpsToMilli(id_A_);
  r.iq_mA = AmpsToMilli(iq_A_);
  r.idref_mA = 0;
  r.iqref_mA = 0;
  r.theta_elec_mrad = 0;
  r.omega_mech_mrad_s = 0;
  r.vd_mV = 0;
  r.vq_mV = 0;
  r.bus_mV = static_cast<uint16_t>(board::kBusVoltage_V * 1000.0f + 0.5f);
  r.enc_raw = 0;
  r.theta_mech_mrad = static_cast<int32_t>(encoder_.sample().mechanical_rad * 1000.0f);
  r.omega_cmd_mrad_s = 0;
  r.omega_elec_mrad_s = 0;
  r.voltage_headroom_mV = 0;

  if (encoder_.valid())
  {
    r.enc_raw = encoder_.sample().raw;
    r.enc_sign = encoder_.calibration().sign >= 0.0f ? 1 : -1;
    r.theta_elec_mrad = static_cast<int32_t>(
        encoder_.sample().electrical_rad * 1000.0f);
  }

  if (foc_.get() != nullptr && foc_->active())
  {
    r.idref_mA = AmpsToMilli(foc_->id_ref_A());
    r.iqref_mA = AmpsToMilli(foc_->iq_ref_A());
    r.vd_mV = static_cast<int32_t>(foc_->vd_V() * 1000.0f);
    r.vq_mV = static_cast<int32_t>(foc_->vq_V() * 1000.0f);
    r.bus_mV = static_cast<uint16_t>(foc_->bus_V() * 1000.0f + 0.5f);
    r.voltage_headroom_mV =
        static_cast<int32_t>(foc_->voltage_headroom_V() * 1000.0f);
    if (encoder_.valid() && encoder_.pll().theta_valid())
    {
      r.theta_elec_mrad =
          static_cast<int32_t>(encoder_.pll().electrical_theta() * 1000.0f);
    }
    else
    {
      r.theta_elec_mrad =
          static_cast<int32_t>(foc_->theta_rad() * 1000.0f);
    }
  }
  else if (dq_modulator_.get() != nullptr && dq_modulator_->active())
  {
    r.theta_elec_mrad =
        static_cast<int32_t>(dq_modulator_->theta_rad() * 1000.0f);
    r.vd_mV = static_cast<int32_t>(dq_modulator_->voltage_V() * 1000.0f);
    r.vq_mV = 0;
    r.bus_mV = static_cast<uint16_t>(dq_modulator_->bus_V() * 1000.0f + 0.5f);
  }

  // Live omega fields are measured mechanical/electrical speed. Keep the
  // command separate so tracking error remains observable.
  if (servo_mode_.get() != nullptr && servo_mode_->active())
  {
    r.omega_cmd_mrad_s =
        static_cast<int32_t>(servo_mode_->velocity_cmd() * 1000.0f);
  }
  else if (foc_.get() != nullptr && foc_->active())
  {
    r.omega_cmd_mrad_s =
        static_cast<int32_t>(foc_->theta_rate_rad_s() /
                             board::MotorParams().pole_pairs * 1000.0f);
  }
  else if (dq_modulator_.get() != nullptr && dq_modulator_->active())
  {
    r.omega_cmd_mrad_s =
        static_cast<int32_t>(dq_modulator_->theta_rate_rad_s() /
                             board::MotorParams().pole_pairs * 1000.0f);
  }
  if (encoder_.valid() && encoder_.pll().theta_valid())
  {
    r.omega_mech_mrad_s =
        static_cast<int32_t>(encoder_.pll().velocity_mech() * 1000.0f);
    r.omega_elec_mrad_s = static_cast<int32_t>(
        encoder_.pll().omega_elec() * 1000.0f);
  }
  return r;
}

void Application::MaybeSendSnapshot()
{
  if (binary_link_.get() == nullptr)
  {
    return;
  }
  if (snapshot_.state() == SnapshotCapture::State::Ready)
  {
    telemetry::xt_can::SnapMeta meta{};
    snapshot_.FillMeta(&meta, snap_seq_);
    // Do not enter Sending until Meta is queued; otherwise the host waits
    // forever while we burn through data frames.
    if (!binary_link_->SendSnapMeta(meta))
    {
      return;
    }
    snap_meta_sent_ = true;
    snapshot_.BeginSend();
  }

  if (snapshot_.state() != SnapshotCapture::State::Sending)
  {
    return;
  }

  // Queue as many FD frames as the TX FIFO will take this iteration.
  // Advance the read cursor ONLY after a successful queue — a full FIFO used
  // to drop the payload while still moving send_index_, so the host saw
  // "incomplete got=N/512" while Live replies resumed almost immediately.
  for (int i = 0; i < 16; ++i)
  {
    if (snapshot_.SendComplete())
    {
      snapshot_.Finish();
      snap_meta_sent_ = false;
      break;
    }
    telemetry::xt_can::SnapData frame{};
    if (!snapshot_.FillDataFrame(&frame, snap_seq_))
    {
      snapshot_.Finish();
      snap_meta_sent_ = false;
      break;
    }
    if (!binary_link_->SendSnapData(frame))
    {
      break;
    }
    snapshot_.AdvanceSend(frame.n);
  }
}

void Application::EnterBootloaderMode()
{
  StopOutput();
  gate_driver_->Disable();
  for (int i = 0; i < 3; i++)
  {
    LedOn();
    DelayNops(200000);
    LedOff();
    DelayNops(200000);
  }
  EnterBootloader();
}

}  // namespace app
