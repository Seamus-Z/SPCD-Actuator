#include "application.h"

#include <cstring>

#include "BL_Config.h"
#include "board_config.h"
#include "bootloader.h"
#include "math/foc.h"
#include "nvs/encoder_cal_store.h"
#include "stm32g4xx.h"

namespace app
{
namespace
{

constexpr uint32_t kBootRequestId = 0x7E;
constexpr char kBootRequestPayload[] = "BOOT";
// Host streams Query/control at ~50 Hz; board replies with CtrlReply.
// If the host stops (USB unplug / GUI crash), drop PWM after this quiet window.
constexpr uint32_t kCmdTimeoutUs = 100000;

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
      voltage_foc_(pool, board::VoltageFocOptions()),
      current_loop_(pool, board::CurrentLoopOptions()),
      encoder_pll_(board::EncoderPllOptions()),
      position_loop_(pool, board::PositionLoopOptions()),
      ma600_(pool, timer_.get(), board::Ma600Options()),
      binary_link_(pool, can_.get(), kDefaultCanId),
      commands_(this)
{
  binary_link_->SetCommandHandler(&BinaryCommands::Thunk, &commands_);
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

  // Encoder bring-up is non-fatal; dq commutation requires encoder_ok_.
  encoder_ok_ = (ma600_.get() != nullptr) && ma600_->Init();
  if (encoder_ok_)
  {
    nvs::EncoderCalData cal{};
    if (nvs::EncoderCalStore::Load(&cal))
    {
      ma600_->SetSign(cal.sign);
      ma600_->SetOffsetRad(cal.offset_rad);
      encoder_cal_persisted_ = true;
    }
    SampleEncoder();
    encoder_pll_.Reset(enc_theta_mech_rad_);
    enc_theta_elec_rad_ = encoder_pll_.electrical_theta();
  }

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

  if (encoder_cal_dirty_)
  {
    PersistEncoderCalResult();
    encoder_cal_dirty_ = false;
  }

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
  last_control_us_ = 0;
  if (encoder_cal_.active())
  {
    encoder_cal_.Stop();
  }
  if (voltage_foc_.get() != nullptr)
  {
    voltage_foc_->Stop();
  }
  if (position_loop_.get() != nullptr)
  {
    position_loop_->Stop();
  }
  if (current_loop_.get() != nullptr)
  {
    current_loop_->Stop();
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
  const bool cal_on = encoder_cal_.active();
  const bool vel_on =
      position_loop_.get() != nullptr && position_loop_->active();
  const bool vfoc_on =
      voltage_foc_.get() != nullptr && voltage_foc_->active() && !cal_on;
  const bool dq_on = current_loop_.get() != nullptr && current_loop_->active() &&
                     !vel_on && !cal_on && mode_ == telemetry::xt_can::kModeDq;
  if ((!vfoc_on && !dq_on && !vel_on && !cal_on) ||
      phase_pwm_.get() == nullptr || current_adc_.get() == nullptr)
  {
    return;
  }

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
      // 15 kHz → ~67 us; clamp coalesced/stalled IRQs so ω stays sane.
      if (dt_us >= 20u && dt_us <= 500u)
      {
        dt_s = static_cast<float>(dt_us) * 1.0e-6f;
      }
    }
    last_control_us_ = now;
  }
  const auto sample = current_adc_->ReadLatest();
  last_current_ = sample;

  if (encoder_ok_ || cal_on)
  {
    UpdateEncoderPll(dt_s);
  }

  if (cal_on)
  {
    const float omega = encoder_cal_.omega_cmd_elec();
    if (omega != cal_last_omega_cmd_)
    {
      voltage_foc_->SetThetaRate(omega);
      cal_last_omega_cmd_ = omega;
    }
    foc_ctrl::VoltageFoc::Duties duties;
    if (!voltage_foc_->Step(dt_s, &duties))
    {
      return;
    }
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    pwm_output_on_ = true;
    ObserveDqFromSample(voltage_foc_->theta_rad(), sample);

    const bool finished =
        encoder_cal_.Step(dt_s, voltage_foc_->theta_rad(), enc_counts_rad_);
    if (finished)
    {
      if (encoder_cal_.state() == calibration::EncoderPhaseCal::State::Done)
      {
        ApplyEncoderCalResult();
      }
      voltage_foc_->Stop();
      gate_driver_->PowerOff();
      pwm_output_on_ = false;
      mode_ = telemetry::xt_can::kModeStop;
      phase_pwm_->SetDuty(hal::PhasePwm::kDutyMinMilli,
                          hal::PhasePwm::kDutyMinMilli,
                          hal::PhasePwm::kDutyMinMilli);
    }
  }
  else if (vel_on)
  {
    // moteus velocity mode: PID(position=NaN, ω) → Iq, encoder θ_e commutation.
    if (!encoder_ok_ || !encoder_pll_.theta_valid())
    {
      position_loop_->Stop();
      current_loop_->Stop();
      if (gate_driver_.get() != nullptr)
      {
        gate_driver_->PowerOff();
      }
      pwm_output_on_ = false;
      dq_valid_ = false;
      mode_ = telemetry::xt_can::kModeStop;
      return;
    }
    const float iq = position_loop_->Step(
        dt_s, encoder_pll_.position_rad(), encoder_pll_.velocity_mech());
    current_loop_->SetRefs(position_loop_->id_ref_A(), iq);
    const float w_cmd = position_loop_->velocity_cmd();
    current_loop_->SetOmega(w_cmd * board::MotorParams().pole_pairs, w_cmd);
    const float theta_elec = encoder_pll_.electrical_theta();
    foc_ctrl::CurrentLoop::Duties duties;
    if (!current_loop_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A,
                             &duties, &theta_elec))
    {
      return;
    }
    id_A_ = current_loop_->id_A();
    iq_A_ = current_loop_->iq_A();
    dq_valid_ = sample.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    pwm_output_on_ = true;
  }
  else if (dq_on)
  {
    foc_ctrl::CurrentLoop::Duties duties;
    if (encoder_ok_ && encoder_pll_.theta_valid())
    {
      current_loop_->SetOmega(encoder_pll_.omega_elec(),
                              encoder_pll_.velocity_mech());
      const float theta_elec = encoder_pll_.electrical_theta();
      if (!current_loop_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A,
                               &duties, &theta_elec))
      {
        return;
      }
    }
    else
    {
      // Fallback: open-loop θ integration (pre-encoder bring-up).
      if (!current_loop_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A,
                               &duties, nullptr))
      {
        return;
      }
    }
    id_A_ = current_loop_->id_A();
    iq_A_ = current_loop_->iq_A();
    dq_valid_ = sample.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    pwm_output_on_ = true;
  }
  else
  {
    foc_ctrl::VoltageFoc::Duties duties;
    if (!voltage_foc_->Step(dt_s, &duties))
    {
      return;
    }
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    pwm_output_on_ = true;
    ObserveDqFromSample(voltage_foc_->theta_rad(), sample);
  }

  const uint32_t dt_us = static_cast<uint32_t>(dt_s * 1.0e6f + 0.5f);
  snapshot_.PushIsr(id_A_, iq_A_, sample.i1_A, sample.i2_A, sample.i3_A, dt_us);
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
  if (encoder_ok_)
  {
    t.flags = static_cast<uint16_t>(t.flags | telemetry::xt_can::kFlagEncOk);
  }
  if (encoder_ok_ && encoder_pll_.theta_valid() &&
      (mode_ == telemetry::xt_can::kModeDq ||
       mode_ == telemetry::xt_can::kModeVel))
  {
    t.flags = static_cast<uint16_t>(t.flags | telemetry::xt_can::kFlagEncMode);
  }

  t.id_mA = AmpsToMilli(id_A_);
  t.iq_mA = AmpsToMilli(iq_A_);
  t.i1_mA = AmpsToMilli(last_current_.i1_A);
  t.i2_mA = AmpsToMilli(last_current_.i2_A);
  t.i3_mA = AmpsToMilli(last_current_.i3_A);

  if (current_loop_.get() != nullptr && current_loop_->active())
  {
    t.idref_mA = AmpsToMilli(current_loop_->id_ref_A());
    t.iqref_mA = AmpsToMilli(current_loop_->iq_ref_A());
    if (encoder_ok_ && encoder_pll_.theta_valid())
    {
      t.theta_mrad =
          static_cast<int32_t>(encoder_pll_.electrical_theta() * 1000.0f);
      const float omega_mech =
          (position_loop_.get() != nullptr && position_loop_->active())
              ? position_loop_->velocity_cmd()
              : encoder_pll_.velocity_mech();
      t.omega_mrad_s = static_cast<int32_t>(
          omega_mech * board::MotorParams().pole_pairs * 1000.0f);
    }
    else
    {
      t.theta_mrad = static_cast<int32_t>(current_loop_->theta_rad() * 1000.0f);
      t.omega_mrad_s =
          static_cast<int32_t>(current_loop_->theta_rate_rad_s() * 1000.0f);
    }
    t.vd_mV = static_cast<int32_t>(current_loop_->vd_V() * 1000.0f);
    t.vq_mV = static_cast<int32_t>(current_loop_->vq_V() * 1000.0f);
    t.bus_mV = static_cast<uint16_t>(current_loop_->bus_V() * 1000.0f + 0.5f);
  }
  else if (voltage_foc_.get() != nullptr && voltage_foc_->active())
  {
    t.theta_mrad = static_cast<int32_t>(voltage_foc_->theta_rad() * 1000.0f);
    t.omega_mrad_s =
        static_cast<int32_t>(voltage_foc_->theta_rate_rad_s() * 1000.0f);
    t.vd_mV = static_cast<int32_t>(voltage_foc_->voltage_V() * 1000.0f);
    t.vq_mV = 0;
    t.bus_mV = static_cast<uint16_t>(voltage_foc_->bus_V() * 1000.0f + 0.5f);
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
    SampleEncoder();
  }
  binary_link_->SendCtrlReply(BuildCtrlReply(cmd, seq, status));
  if (encoder_cal_.state() != calibration::EncoderPhaseCal::State::Idle)
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
  telemetry::xt_can::CalTelem c{};
  c.kind = static_cast<uint8_t>(
      encoder_cal_.method() == calibration::EncoderPhaseCal::Method::Lock
          ? telemetry::xt_can::kCalSubEncLock
          : telemetry::xt_can::kCalSubEncPhase);
  c.state = static_cast<uint8_t>(encoder_cal_.state());
  c.progress_pm = encoder_cal_.progress_permille();
  const auto& r = encoder_cal_.result();
  c.offset_mrad = static_cast<int32_t>(r.offset_rad * 1000.0f);
  c.residual_mrad = static_cast<int32_t>(r.residual_rad_rms * 1000.0f);
  c.sign = (r.sign >= 0.0f) ? 1 : -1;
  c.ok = r.ok ? 1 : 0;
  uint16_t samples = r.samples;
  if (encoder_cal_persisted_ && samples < 0x8000)
  {
    samples = static_cast<uint16_t>(samples | 0x8000u);
  }
  c.samples = samples;
  return c;
}


void Application::SampleEncoder()
{
  if (ma600_.get() == nullptr)
  {
    return;
  }
  (void)ma600_->Sample();
  enc_counts_rad_ = ma600_->angle_counts_rad();
  enc_theta_mech_rad_ = ma600_->angle_mech_rad();
  if (encoder_pll_.theta_valid())
  {
    enc_theta_elec_rad_ = encoder_pll_.electrical_theta();
  }
  else
  {
    enc_theta_elec_rad_ =
        ma600_->angle_elec_rad(board::MotorParams().pole_pairs);
  }
}

void Application::UpdateEncoderPll(float dt_s)
{
  if (ma600_.get() == nullptr)
  {
    return;
  }
  (void)ma600_->Sample();
  enc_counts_rad_ = ma600_->angle_counts_rad();
  enc_theta_mech_rad_ = ma600_->angle_mech_rad();
  encoder_pll_.Update(enc_theta_mech_rad_, dt_s);
  enc_theta_elec_rad_ = encoder_pll_.electrical_theta();
}

void Application::ApplyEncoderCalResult()
{
  const auto& r = encoder_cal_.result();
  if (!r.ok || ma600_.get() == nullptr)
  {
    return;
  }
  // Runtime register-like write (immediate).
  ma600_->SetSign(r.sign);
  ma600_->SetOffsetRad(r.offset_rad);
  enc_counts_rad_ = ma600_->angle_counts_rad();
  enc_theta_mech_rad_ = ma600_->angle_mech_rad();
  encoder_pll_.Reset(enc_theta_mech_rad_);
  enc_theta_elec_rad_ = encoder_pll_.electrical_theta();
  // Defer flash write to main loop (ISR must not program flash).
  encoder_cal_dirty_ = true;
  encoder_cal_persisted_ = false;
}

void Application::PersistEncoderCalResult()
{
  if (ma600_.get() == nullptr)
  {
    return;
  }
  nvs::EncoderCalData data;
  data.offset_rad = ma600_->options().offset_rad;
  data.sign = ma600_->options().sign;
  // Flash erase/program can stall; keep PWM ISR off.
  if (phase_pwm_.get() != nullptr)
  {
    phase_pwm_->DisableControlIsr();
  }
  encoder_cal_persisted_ = nvs::EncoderCalStore::Save(data);
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
  if (ma600_.get() == nullptr)
  {
    return e;
  }
  e.raw = ma600_->raw();
  e.theta_mech_mrad =
      static_cast<int32_t>(enc_theta_mech_rad_ * 1000.0f);
  e.theta_elec_mrad =
      static_cast<int32_t>(enc_theta_elec_rad_ * 1000.0f);
  e.sign = (ma600_->options().sign >= 0.0f) ? 1 : -1;
  e.ok = encoder_ok_ ? 1 : 0;
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
  if (encoder_ok_)
  {
    r.flags = static_cast<uint16_t>(r.flags | telemetry::xt_can::kFlagEncOk);
  }
  if (encoder_ok_ && encoder_pll_.theta_valid() &&
      (mode_ == telemetry::xt_can::kModeDq ||
       mode_ == telemetry::xt_can::kModeVel))
  {
    r.flags = static_cast<uint16_t>(r.flags | telemetry::xt_can::kFlagEncMode);
  }

  r.mode = mode_;
  r.enc_ok = encoder_ok_ ? 1 : 0;
  r.enc_sign = 1;
  r.reserved = 0;
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
  r.theta_mech_mrad = static_cast<int32_t>(enc_theta_mech_rad_ * 1000.0f);
  r.reserved2 = 0;

  if (ma600_.get() != nullptr)
  {
    r.enc_raw = ma600_->raw();
    r.enc_sign = (ma600_->options().sign >= 0.0f) ? 1 : -1;
    r.theta_elec_mrad =
        static_cast<int32_t>(enc_theta_elec_rad_ * 1000.0f);
  }

  if (current_loop_.get() != nullptr && current_loop_->active())
  {
    r.idref_mA = AmpsToMilli(current_loop_->id_ref_A());
    r.iqref_mA = AmpsToMilli(current_loop_->iq_ref_A());
    r.vd_mV = static_cast<int32_t>(current_loop_->vd_V() * 1000.0f);
    r.vq_mV = static_cast<int32_t>(current_loop_->vq_V() * 1000.0f);
    r.bus_mV = static_cast<uint16_t>(current_loop_->bus_V() * 1000.0f + 0.5f);
    if (encoder_ok_ && encoder_pll_.theta_valid())
    {
      r.theta_elec_mrad =
          static_cast<int32_t>(encoder_pll_.electrical_theta() * 1000.0f);
    }
    else
    {
      r.theta_elec_mrad =
          static_cast<int32_t>(current_loop_->theta_rad() * 1000.0f);
    }
  }
  else if (voltage_foc_.get() != nullptr && voltage_foc_->active())
  {
    r.theta_elec_mrad =
        static_cast<int32_t>(voltage_foc_->theta_rad() * 1000.0f);
    r.vd_mV = static_cast<int32_t>(voltage_foc_->voltage_V() * 1000.0f);
    r.vq_mV = 0;
    r.bus_mV = static_cast<uint16_t>(voltage_foc_->bus_V() * 1000.0f + 0.5f);
  }

  // Live omega = measured mech ω. Command goes in reserved2 for GUI compare.
  if (position_loop_.get() != nullptr && position_loop_->active())
  {
    r.reserved2 = static_cast<int32_t>(position_loop_->velocity_cmd() * 1000.0f);
  }
  else if (current_loop_.get() != nullptr && current_loop_->active())
  {
    r.reserved2 =
        static_cast<int32_t>(current_loop_->theta_rate_rad_s() * 1000.0f);
  }
  else if (voltage_foc_.get() != nullptr && voltage_foc_->active())
  {
    r.reserved2 =
        static_cast<int32_t>(voltage_foc_->theta_rate_rad_s() * 1000.0f);
  }
  if (encoder_ok_ && encoder_pll_.theta_valid())
  {
    r.omega_mech_mrad_s =
        static_cast<int32_t>(encoder_pll_.velocity_mech() * 1000.0f);
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
    binary_link_->SendSnapMeta(meta);
    snap_meta_sent_ = true;
    snapshot_.BeginSend();
  }

  if (snapshot_.state() != SnapshotCapture::State::Sending)
  {
    return;
  }

  // Burst a few FD frames per main-loop iteration.
  for (int i = 0; i < 4; ++i)
  {
    telemetry::xt_can::SnapData frame{};
    if (!snapshot_.FillDataFrame(&frame, snap_seq_))
    {
      snapshot_.Finish();
      snap_meta_sent_ = false;
      break;
    }
    binary_link_->SendSnapData(frame);
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
