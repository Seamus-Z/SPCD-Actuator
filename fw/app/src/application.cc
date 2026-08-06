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
  nvs::EncoderCalData cal{};
  const bool cal_loaded = nvs::EncoderCalStore::Load(&cal);

  encoder_ok_ = (ma600_.get() != nullptr) && ma600_->Init();
  if (encoder_ok_)
  {
    if (cal_loaded)
    {
      ma600_->SetSign(cal.sign);
      ma600_->SetOffsetRad(cal.offset_rad);
      ma600_->SetCommutationOffsets(cal.commutation_offset_rad,
                                    cal.commutation_valid);
      encoder_cal_persisted_ = true;
    }
    SampleEncoder();
    encoder_pll_.Reset(enc_theta_mech_rad_,
                       ma600_->commutation_offset_rad());
    enc_theta_elec_rad_ = encoder_pll_.electrical_theta();
  }

  if (cal_loaded && cal.bemf_valid && cal.bemf_v_per_hz > 0.005f)
  {
    if (current_loop_.get() != nullptr)
    {
      current_loop_->SetVPerHz(cal.bemf_v_per_hz);
    }
    if (position_loop_.get() != nullptr)
    {
      position_loop_->SetTorqueConstant(
          board::IdentifiedTorqueConstantNmPerA(cal.bemf_v_per_hz));
    }
    bemf_cal_persisted_ = true;
  }

  if (cal_loaded && (cal.resistance_valid || cal.inductance_valid) &&
      current_loop_.get() != nullptr)
  {
    const float r = cal.resistance_valid ? cal.resistance_ohm
                                         : current_loop_->resistance_ohm();
    const float ld = cal.inductance_valid ? cal.inductance_d_H
                                          : current_loop_->inductance_d_H();
    const float lq = cal.inductance_valid ? cal.inductance_q_H
                                          : current_loop_->inductance_q_H();
    if (r > 0.001f && ld > 0.0f && lq > 0.0f)
    {
      current_loop_->SetResistanceInductance(r, ld, lq);
    }
    r_cal_persisted_ = cal.resistance_valid;
    l_cal_persisted_ = cal.inductance_valid;
  }

  if (cal_loaded && cal.cogging_valid)
  {
    cogging_table_ = cal.cogging_table;
    cogging_scale_ = cal.cogging_scale;
    cogging_valid_ = true;
    cogging_cal_persisted_ = true;
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
  if (bemf_cal_dirty_)
  {
    PersistBemfCalResult();
    bemf_cal_dirty_ = false;
  }
  if (r_cal_dirty_)
  {
    PersistRIdentResult();
    r_cal_dirty_ = false;
  }
  if (l_cal_dirty_)
  {
    PersistLIdentResult();
    l_cal_dirty_ = false;
  }
  if (cogging_cal_dirty_)
  {
    PersistCoggingResult();
    cogging_cal_dirty_ = false;
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
  if (bemf_cal_.active())
  {
    bemf_cal_.Stop();
  }
  if (r_cal_.active())
  {
    r_cal_.Stop();
  }
  if (l_cal_.active())
  {
    l_cal_.Stop();
  }
  if (cogging_cal_.active())
  {
    cogging_cal_.Stop();
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
  const bool bemf_on = bemf_cal_.active();
  const bool r_on = r_cal_.active();
  const bool l_on = l_cal_.active();
  const bool cogging_on = cogging_cal_.active();
  const bool vel_on =
      position_loop_.get() != nullptr && position_loop_->active() &&
      !bemf_on && !cogging_on;
  const bool vfoc_on =
      voltage_foc_.get() != nullptr && voltage_foc_->active() && !cal_on &&
      !l_on;
  const bool dq_on = current_loop_.get() != nullptr && current_loop_->active() &&
                     !vel_on && !cal_on && !bemf_on &&
                     mode_ == telemetry::xt_can::kModeDq;
  if ((!vfoc_on && !dq_on && !vel_on && !cal_on && !bemf_on && !r_on &&
       !l_on && !cogging_on) ||
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
      current_loop_->SetThetaRate(omega);
      cal_last_omega_cmd_ = omega;
    }
    current_loop_->SetRefs(encoder_cal_.current_A(), 0.0f);
    foc_ctrl::CurrentLoop::Duties duties;
    if (!current_loop_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A,
                             &duties))
    {
      return;
    }
    id_A_ = current_loop_->id_A();
    iq_A_ = current_loop_->iq_A();
    dq_valid_ = sample.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    pwm_output_on_ = true;

    const bool finished = encoder_cal_.Step(
        dt_s, current_loop_->theta_rad(), enc_counts_rad_);
    if (finished)
    {
      if (encoder_cal_.state() == calibration::EncoderPhaseCal::State::Done)
      {
        ApplyEncoderCalResult();
      }
      else
      {
        RestoreEncoderCalBackup();
      }
      current_loop_->Stop();
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
    if (!encoder_ok_ || !encoder_pll_.theta_valid())
    {
      bemf_cal_.Stop();
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
    position_loop_->SetVelocity(bemf_cal_.velocity_cmd_mech_rad_s());
    position_loop_->SetIdRef(0.0f);
    const float iq = position_loop_->Step(
        dt_s, encoder_pll_.position_rad(), encoder_pll_.velocity_mech());
    if (position_loop_->faulted())
    {
      bemf_cal_.Stop();
      StopOutput();
      return;
    }
    current_loop_->SetRefs(position_loop_->id_ref_A(), iq);
    const float w_cmd = position_loop_->control_velocity();
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

    const float w_meas = encoder_pll_.velocity_mech();
    const bool finished = bemf_cal_.Step(dt_s, w_meas, current_loop_->vq_V(),
                                         current_loop_->iq_A());
    if (finished)
    {
      if (bemf_cal_.state() == calibration::BemfIdentCal::State::Done)
      {
        ApplyBemfCalResult();
      }
      position_loop_->Stop();
      current_loop_->Stop();
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
    current_loop_->SetRefs(r_cal_.id_cmd_A(), 0.0f);
    foc_ctrl::CurrentLoop::Duties duties;
    if (!current_loop_->Step(dt_s, sample.i1_A, sample.i2_A, sample.i3_A,
                             &duties, nullptr))
    {
      return;
    }
    id_A_ = current_loop_->id_A();
    iq_A_ = current_loop_->iq_A();
    dq_valid_ = sample.ok;
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    pwm_output_on_ = true;

    const bool finished = r_cal_.Step(dt_s, id_A_, current_loop_->vd_V());
    if (finished)
    {
      if (r_cal_.state() == calibration::RIdentCal::State::Done)
      {
        ApplyRIdentResult();
      }
      current_loop_->Stop();
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
    voltage_foc_->SetDqVoltage(l_cal_.voltage_d_cmd_V(),
                               l_cal_.voltage_q_cmd_V());
    foc_ctrl::VoltageFoc::Duties duties;
    if (!voltage_foc_->Step(dt_s, &duties))
    {
      return;
    }
    phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
    gate_driver_->PowerOn();
    pwm_output_on_ = true;
    ObserveDqFromSample(voltage_foc_->theta_rad(), sample);

    const bool finished = l_cal_.Step(dt_s, id_A_, iq_A_);
    if (finished)
    {
      if (l_cal_.state() == calibration::LIdentCal::State::Done)
      {
        ApplyLIdentResult();
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
  else if (cogging_on)
  {
    // Cogging map: run the velocity loop at a slow constant speed (fwd then
    // rev), record the torque current per rotor position. No cogging FF is
    // injected here — we are measuring the raw ripple.
    if (!encoder_ok_ || !encoder_pll_.theta_valid())
    {
      cogging_cal_.Stop();
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
    position_loop_->SetVelocity(cogging_cal_.velocity_cmd_mech_rad_s());
    position_loop_->SetIdRef(0.0f);
    const float iq = position_loop_->Step(
        dt_s, encoder_pll_.position_rad(), encoder_pll_.velocity_mech());
    if (position_loop_->faulted())
    {
      cogging_cal_.Stop();
      StopOutput();
      return;
    }
    current_loop_->SetRefs(position_loop_->id_ref_A(), iq);
    const float w_cmd = position_loop_->control_velocity();
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

    const bool finished = cogging_cal_.Step(dt_s, enc_theta_mech_rad_, iq);
    if (finished)
    {
      if (cogging_cal_.state() == calibration::CoggingCal::State::Done)
      {
        ApplyCoggingResult();
      }
      position_loop_->Stop();
      current_loop_->Stop();
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
    if (position_loop_->faulted())
    {
      StopOutput();
      return;
    }
    current_loop_->SetRefs(position_loop_->id_ref_A(), iq + CoggingComp());
    const float w_cmd = position_loop_->control_velocity();
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
  snapshot_.PushIsr(id_A_, iq_A_, sample.i1_A, sample.i2_A, sample.i3_A,
                    enc_theta_mech_rad_, enc_theta_elec_rad_, dt_us);
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
      t.omega_mrad_s =
          static_cast<int32_t>(encoder_pll_.velocity_mech() * 1000.0f);
    }
    else
    {
      t.theta_mrad = static_cast<int32_t>(current_loop_->theta_rad() * 1000.0f);
      t.omega_mrad_s = static_cast<int32_t>(
          current_loop_->theta_rate_rad_s() /
          board::MotorParams().pole_pairs * 1000.0f);
    }
    t.vd_mV = static_cast<int32_t>(current_loop_->vd_V() * 1000.0f);
    t.vq_mV = static_cast<int32_t>(current_loop_->vq_V() * 1000.0f);
    t.bus_mV = static_cast<uint16_t>(current_loop_->bus_V() * 1000.0f + 0.5f);
  }
  else if (voltage_foc_.get() != nullptr && voltage_foc_->active())
  {
    t.theta_mrad = static_cast<int32_t>(voltage_foc_->theta_rad() * 1000.0f);
    t.omega_mrad_s = static_cast<int32_t>(
        voltage_foc_->theta_rate_rad_s() /
        board::MotorParams().pole_pairs * 1000.0f);
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
  if (encoder_cal_.state() != calibration::EncoderPhaseCal::State::Idle ||
      bemf_cal_.state() != calibration::BemfIdentCal::State::Idle ||
      r_cal_.state() != calibration::RIdentCal::State::Idle ||
      l_cal_.state() != calibration::LIdentCal::State::Idle ||
      cogging_cal_.state() != calibration::CoggingCal::State::Idle)
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
  if (last_cal_kind_ == telemetry::xt_can::kCalSubBemf)
  {
    telemetry::xt_can::CalTelem c{};
    c.kind = telemetry::xt_can::kCalSubBemf;
    switch (bemf_cal_.state())
    {
      case calibration::BemfIdentCal::State::Running:
        c.state = telemetry::xt_can::kCalStateBemfRun;
        break;
      case calibration::BemfIdentCal::State::Done:
        c.state = telemetry::xt_can::kCalStateDone;
        break;
      case calibration::BemfIdentCal::State::Failed:
        c.state = telemetry::xt_can::kCalStateFailed;
        break;
      default:
        c.state = telemetry::xt_can::kCalStateIdle;
        break;
    }
    c.progress_pm = bemf_cal_.progress_permille();
    const auto& br = bemf_cal_.result();
    c.offset_mrad = static_cast<int32_t>(br.ke_v_s_per_rad * 1.0e6f);
    c.residual_mrad = static_cast<int32_t>(br.r2 * 1.0e6f);
    c.sign = 0;
    c.ok = br.ok ? 1 : 0;
    {
      uint16_t samples = br.points_used;
      if (bemf_cal_persisted_ && samples < 0x8000)
      {
        samples = static_cast<uint16_t>(samples | 0x8000u);
      }
      c.samples = samples;
    }
    return c;
  }
  if (last_cal_kind_ == telemetry::xt_can::kCalSubResistance)
  {
    telemetry::xt_can::CalTelem c{};
    c.kind = telemetry::xt_can::kCalSubResistance;
    switch (r_cal_.state())
    {
      case calibration::RIdentCal::State::Running:
        c.state = telemetry::xt_can::kCalStateRRun;
        break;
      case calibration::RIdentCal::State::Done:
        c.state = telemetry::xt_can::kCalStateDone;
        break;
      case calibration::RIdentCal::State::Failed:
        c.state = telemetry::xt_can::kCalStateFailed;
        break;
      default:
        c.state = telemetry::xt_can::kCalStateIdle;
        break;
    }
    c.progress_pm = r_cal_.progress_permille();
    const auto& rr = r_cal_.result();
    c.offset_mrad = static_cast<int32_t>(rr.resistance_ohm * 1.0e6f);
    c.residual_mrad = static_cast<int32_t>(rr.r2 * 1.0e6f);
    c.sign = 0;
    c.ok = rr.ok ? 1 : 0;
    {
      uint16_t samples = rr.points_used;
      if (r_cal_persisted_ && samples < 0x8000)
      {
        samples = static_cast<uint16_t>(samples | 0x8000u);
      }
      c.samples = samples;
    }
    return c;
  }
  if (last_cal_kind_ == telemetry::xt_can::kCalSubInductance)
  {
    telemetry::xt_can::CalTelem c{};
    c.kind = telemetry::xt_can::kCalSubInductance;
    switch (l_cal_.state())
    {
      case calibration::LIdentCal::State::Running:
        c.state = telemetry::xt_can::kCalStateLRun;
        break;
      case calibration::LIdentCal::State::Done:
        c.state = telemetry::xt_can::kCalStateDone;
        break;
      case calibration::LIdentCal::State::Failed:
        c.state = telemetry::xt_can::kCalStateFailed;
        break;
      default:
        c.state = telemetry::xt_can::kCalStateIdle;
        break;
    }
    c.progress_pm = l_cal_.progress_permille();
    const auto& lr = l_cal_.result();
    c.offset_mrad = static_cast<int32_t>(lr.inductance_d_H * 1.0e9f);
    c.residual_mrad = static_cast<int32_t>(lr.inductance_q_H * 1.0e9f);
    c.sign = 0;
    c.ok = lr.ok ? 1 : 0;
    {
      uint16_t samples =
          static_cast<uint16_t>(lr.trials_d_used + lr.trials_q_used);
      if (l_cal_persisted_ && samples < 0x8000)
      {
        samples = static_cast<uint16_t>(samples | 0x8000u);
      }
      c.samples = samples;
    }
    return c;
  }
  if (last_cal_kind_ == telemetry::xt_can::kCalSubCogging)
  {
    telemetry::xt_can::CalTelem c{};
    c.kind = telemetry::xt_can::kCalSubCogging;
    switch (cogging_cal_.state())
    {
      case calibration::CoggingCal::State::Running:
        c.state = telemetry::xt_can::kCalStateCoggingRun;
        break;
      case calibration::CoggingCal::State::Done:
        c.state = telemetry::xt_can::kCalStateDone;
        break;
      case calibration::CoggingCal::State::Failed:
        c.state = telemetry::xt_can::kCalStateFailed;
        break;
      default:
        c.state = telemetry::xt_can::kCalStateIdle;
        break;
    }
    c.progress_pm = cogging_cal_.progress_permille();
    const auto& cr = cogging_cal_.result();
    c.offset_mrad = static_cast<int32_t>(cr.scale * 1.0e6f);
    c.residual_mrad = static_cast<int32_t>(cr.peak_A * 1.0e6f);
    c.sign = 0;
    c.ok = cr.ok ? 1 : 0;
    {
      uint16_t samples = cogging_cal_persisted_ ? 0x8001u : 1u;
      c.samples = samples;
    }
    return c;
  }
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
    enc_theta_elec_rad_ = math::WrapZeroToTwoPi(
        ma600_->angle_elec_rad(board::MotorParams().pole_pairs) +
        ma600_->commutation_offset_rad());
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
  encoder_pll_.Update(enc_theta_mech_rad_, dt_s,
                      ma600_->commutation_offset_rad());
  enc_theta_elec_rad_ = encoder_pll_.electrical_theta();
}
void Application::RestoreEncoderCalBackup()
{
  if (!encoder_cal_backup_valid_ || ma600_.get() == nullptr)
  {
    return;
  }
  ma600_->SetSign(encoder_cal_backup_.sign);
  ma600_->SetOffsetRad(encoder_cal_backup_.offset_rad);
  ma600_->SetCommutationOffsets(encoder_cal_backup_.commutation_offset_rad,
                                encoder_cal_backup_.commutation_valid);
  enc_counts_rad_ = ma600_->angle_counts_rad();
  enc_theta_mech_rad_ = ma600_->angle_mech_rad();
  encoder_pll_.Reset(enc_theta_mech_rad_, ma600_->commutation_offset_rad());
  enc_theta_elec_rad_ = encoder_pll_.electrical_theta();
  encoder_cal_backup_valid_ = false;
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
  ma600_->SetCommutationOffsets(r.commutation_offset_rad,
                                r.commutation_valid);
  enc_counts_rad_ = ma600_->angle_counts_rad();
  enc_theta_mech_rad_ = ma600_->angle_mech_rad();
  encoder_pll_.Reset(enc_theta_mech_rad_,
                     ma600_->commutation_offset_rad());
  enc_theta_elec_rad_ = encoder_pll_.electrical_theta();
  encoder_cal_backup_valid_ = false;
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
  nvs::EncoderCalData data{};
  nvs::EncoderCalStore::Load(&data);  // preserve any existing bemf override
  data.offset_rad = ma600_->options().offset_rad;
  data.sign = ma600_->options().sign;
  data.commutation_offset_rad =
      ma600_->options().commutation_offset_rad;
  data.commutation_valid = ma600_->options().commutation_valid;
  data.commutation_residual_rad = encoder_cal_.result().residual_rad_rms;
  // Flash erase/program can stall; keep PWM ISR off.
  if (phase_pwm_.get() != nullptr)
  {
    phase_pwm_->DisableControlIsr();
  }
  encoder_cal_persisted_ = nvs::EncoderCalStore::Save(data);
}

void Application::ApplyBemfCalResult()
{
  const auto& r = bemf_cal_.result();
  if (!r.ok || r.ke_v_s_per_rad <= 0.005f)
  {
    return;
  }
  if (current_loop_.get() != nullptr)
  {
    current_loop_->SetVPerHz(r.ke_v_s_per_rad);
  }
  if (position_loop_.get() != nullptr)
  {
    position_loop_->SetTorqueConstant(
        board::IdentifiedTorqueConstantNmPerA(r.ke_v_s_per_rad));
  }
  // Defer flash write to main loop (ISR must not program flash).
  bemf_cal_dirty_ = true;
  bemf_cal_persisted_ = false;
}

void Application::PersistBemfCalResult()
{
  const auto& r = bemf_cal_.result();
  if (!r.ok || r.ke_v_s_per_rad <= 0.005f)
  {
    return;
  }
  nvs::EncoderCalData data{};
  nvs::EncoderCalStore::Load(&data);  // preserve encoder offset/sign
  data.bemf_v_per_hz = r.ke_v_s_per_rad;
  data.bemf_valid = true;
  // Flash erase/program can stall; keep PWM ISR off.
  if (phase_pwm_.get() != nullptr)
  {
    phase_pwm_->DisableControlIsr();
  }
  bemf_cal_persisted_ = nvs::EncoderCalStore::Save(data);
}

void Application::ApplyRIdentResult()
{
  const auto& r = r_cal_.result();
  if (!r.ok || r.resistance_ohm <= 0.001f)
  {
    return;
  }
  if (current_loop_.get() != nullptr)
  {
    current_loop_->SetResistanceInductance(
        r.resistance_ohm, current_loop_->inductance_d_H(),
        current_loop_->inductance_q_H());
  }
  // Defer flash write to main loop (ISR must not program flash).
  r_cal_dirty_ = true;
  r_cal_persisted_ = false;
}

void Application::PersistRIdentResult()
{
  const auto& r = r_cal_.result();
  if (!r.ok || r.resistance_ohm <= 0.001f)
  {
    return;
  }
  nvs::EncoderCalData data{};
  nvs::EncoderCalStore::Load(&data);  // preserve the other fields
  data.resistance_ohm = r.resistance_ohm;
  data.resistance_valid = true;
  if (phase_pwm_.get() != nullptr)
  {
    phase_pwm_->DisableControlIsr();
  }
  r_cal_persisted_ = nvs::EncoderCalStore::Save(data);
}

void Application::ApplyLIdentResult()
{
  const auto& r = l_cal_.result();
  if (!r.ok || r.inductance_d_H <= 0.0f || r.inductance_q_H <= 0.0f)
  {
    return;
  }
  if (current_loop_.get() != nullptr)
  {
    current_loop_->SetResistanceInductance(
        current_loop_->resistance_ohm(), r.inductance_d_H,
        r.inductance_q_H);
  }
  l_cal_dirty_ = true;
  l_cal_persisted_ = false;
}

void Application::PersistLIdentResult()
{
  const auto& r = l_cal_.result();
  if (!r.ok || r.inductance_d_H <= 0.0f || r.inductance_q_H <= 0.0f)
  {
    return;
  }
  nvs::EncoderCalData data{};
  nvs::EncoderCalStore::Load(&data);  // preserve the other fields
  data.inductance_d_H = r.inductance_d_H;
  data.inductance_q_H = r.inductance_q_H;
  data.inductance_valid = true;
  if (phase_pwm_.get() != nullptr)
  {
    phase_pwm_->DisableControlIsr();
  }
  l_cal_persisted_ = nvs::EncoderCalStore::Save(data);
}

void Application::ApplyCoggingResult()
{
  const auto& r = cogging_cal_.result();
  if (!r.ok)
  {
    return;
  }
  // Zero scale == no measurable cogging; disable rather than divide by zero.
  cogging_table_ = r.table;
  cogging_scale_ = r.scale;
  cogging_valid_ = r.scale > 0.0f;
  cogging_cal_dirty_ = true;
  cogging_cal_persisted_ = false;
}

void Application::PersistCoggingResult()
{
  const auto& r = cogging_cal_.result();
  if (!r.ok)
  {
    return;
  }
  nvs::EncoderCalData data{};
  nvs::EncoderCalStore::Load(&data);  // preserve the other fields
  data.cogging_table = r.table;
  data.cogging_scale = r.scale;
  data.cogging_valid = r.scale > 0.0f;
  if (phase_pwm_.get() != nullptr)
  {
    phase_pwm_->DisableControlIsr();
  }
  cogging_cal_persisted_ = nvs::EncoderCalStore::Save(data);
}

float Application::CoggingComp() const
{
  if (!cogging_valid_)
  {
    return 0.0f;
  }
  return math::InterpolateCogging(cogging_table_, cogging_scale_,
                                  enc_theta_mech_rad_);
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
  r.omega_cmd_mrad_s = 0;
  r.omega_elec_mrad_s = 0;
  r.voltage_headroom_mV = 0;

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
    r.voltage_headroom_mV =
        static_cast<int32_t>(current_loop_->voltage_headroom_V() * 1000.0f);
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

  // Live omega fields are measured mechanical/electrical speed. Keep the
  // command separate so tracking error remains observable.
  if (position_loop_.get() != nullptr && position_loop_->active())
  {
    r.omega_cmd_mrad_s =
        static_cast<int32_t>(position_loop_->velocity_cmd() * 1000.0f);
  }
  else if (current_loop_.get() != nullptr && current_loop_->active())
  {
    r.omega_cmd_mrad_s =
        static_cast<int32_t>(current_loop_->theta_rate_rad_s() /
                             board::MotorParams().pole_pairs * 1000.0f);
  }
  else if (voltage_foc_.get() != nullptr && voltage_foc_->active())
  {
    r.omega_cmd_mrad_s =
        static_cast<int32_t>(voltage_foc_->theta_rate_rad_s() /
                             board::MotorParams().pole_pairs * 1000.0f);
  }
  if (encoder_ok_ && encoder_pll_.theta_valid())
  {
    r.omega_mech_mrad_s =
        static_cast<int32_t>(encoder_pll_.velocity_mech() * 1000.0f);
    r.omega_elec_mrad_s = static_cast<int32_t>(
        encoder_pll_.omega_elec() * 1000.0f);
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
