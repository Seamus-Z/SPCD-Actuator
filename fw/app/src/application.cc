#include "application.h"

#include <cstring>

#include "BL_Config.h"
#include "board_config.h"
#include "bootloader.h"
#include "math/foc.h"
#include "stm32g4xx.h"

namespace app
{
namespace
{

constexpr uint32_t kBootRequestId = 0x7E;
constexpr char kBootRequestPayload[] = "BOOT";
// Live telemetry ~500 Hz (snapshot dump uses the bus when active).
constexpr uint32_t kTelemPeriodUs = 2000;

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

  // Encoder readout only — failure must not block motor bring-up.
  encoder_ok_ = (ma600_.get() != nullptr) && ma600_->Init();
  if (encoder_ok_)
  {
    SampleEncoder();
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

  PollCan();
  MaybeSendSnapshot();
  MaybeSendTelemetry();
  LedOn();
}

void Application::DriverFault()
{
  StopOutput();
  PollCan();
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
  if (voltage_foc_.get() != nullptr)
  {
    voltage_foc_->Stop();
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
  const bool vfoc_on = voltage_foc_.get() != nullptr && voltage_foc_->active();
  const bool dq_on = current_loop_.get() != nullptr && current_loop_->active();
  if ((!vfoc_on && !dq_on) || phase_pwm_.get() == nullptr ||
      current_adc_.get() == nullptr)
  {
    return;
  }

  const float dt_s = phase_pwm_->period_s();
  const auto sample = current_adc_->ReadLatest();
  last_current_ = sample;

  if (dq_on)
  {
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

  // PWM-rate snapshot fill (Id/Iq + phase currents).
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

  t.id_mA = AmpsToMilli(id_A_);
  t.iq_mA = AmpsToMilli(iq_A_);
  t.i1_mA = AmpsToMilli(last_current_.i1_A);
  t.i2_mA = AmpsToMilli(last_current_.i2_A);
  t.i3_mA = AmpsToMilli(last_current_.i3_A);

  if (current_loop_.get() != nullptr && current_loop_->active())
  {
    t.idref_mA = AmpsToMilli(current_loop_->id_ref_A());
    t.iqref_mA = AmpsToMilli(current_loop_->iq_ref_A());
    t.theta_mrad = static_cast<int32_t>(current_loop_->theta_rad() * 1000.0f);
    t.omega_mrad_s =
        static_cast<int32_t>(current_loop_->theta_rate_rad_s() * 1000.0f);
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

void Application::MaybeSendTelemetry()
{
  if (binary_link_.get() == nullptr || timer_.get() == nullptr)
  {
    return;
  }
  // Pause live telem while snapshot occupies the TX path.
  if (snapshot_.busy())
  {
    return;
  }
  const auto now = timer_->read_us();
  const auto elapsed =
      static_cast<uint32_t>(hal::MillisecondTimer::subtract_us(now, telem_last_us_));
  if (elapsed < kTelemPeriodUs)
  {
    return;
  }
  telem_last_us_ = now;
  SampleEncoder();
  binary_link_->SendTelemetry(BuildTelemetry());
  if (ma600_.get() != nullptr)
  {
    binary_link_->SendEncTelem(BuildEncTelem());
  }
}


void Application::SampleEncoder()
{
  if (ma600_.get() == nullptr)
  {
    return;
  }
  (void)ma600_->Sample();
  enc_theta_mech_rad_ = ma600_->angle_mech_rad();
  enc_theta_elec_rad_ =
      ma600_->angle_elec_rad(board::MotorParams().pole_pairs);
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
