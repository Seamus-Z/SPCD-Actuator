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

void LedOn() { GPIOB->BSRR = 0x80000000; }
void LedOff() { GPIOB->BSRR = 0x00008000; }

void DelayNops(uint32_t count)
{
  for (volatile uint32_t d = 0; d < count; ++d)
  {
    __NOP();
  }
}

// Parse decimal uint16; returns false on empty/non-digit/overflow.
bool ParseU16(std::string_view text, uint16_t* out)
{
  if (out == nullptr || text.empty())
  {
    return false;
  }
  unsigned value = 0;
  for (char c : text)
  {
    if (c < '0' || c > '9')
    {
      return false;
    }
    value = value * 10u + static_cast<unsigned>(c - '0');
    if (value > 65535u)
    {
      return false;
    }
  }
  *out = static_cast<uint16_t>(value);
  return true;
}

// Simple decimal float parser (optional sign + fraction). No exponent.
bool ParseF32(std::string_view text, float* out)
{
  if (out == nullptr || text.empty())
  {
    return false;
  }

  size_t i = 0;
  float sign = 1.0f;
  if (text[i] == '-')
  {
    sign = -1.0f;
    ++i;
  }
  else if (text[i] == '+')
  {
    ++i;
  }
  if (i >= text.size())
  {
    return false;
  }

  float value = 0.0f;
  bool saw_digit = false;
  while (i < text.size() && text[i] >= '0' && text[i] <= '9')
  {
    saw_digit = true;
    value = value * 10.0f + static_cast<float>(text[i] - '0');
    ++i;
  }

  if (i < text.size() && text[i] == '.')
  {
    ++i;
    float place = 0.1f;
    while (i < text.size() && text[i] >= '0' && text[i] <= '9')
    {
      saw_digit = true;
      value += static_cast<float>(text[i] - '0') * place;
      place *= 0.1f;
      ++i;
    }
  }

  if (!saw_digit || i != text.size())
  {
    return false;
  }
  *out = sign * value;
  return true;
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
      registry_(pool),
      diagnostic_(pool, can_.get(), kDefaultCanId, registry_.get()),
      telemetry_(registry_.get(), &state_, gate_driver_.get(),
                 current_adc_.get(), phase_pwm_.get(), this, pool)
{
  diagnostic_->SetAppCommandHandler(&Application::DiagCommandThunk, this);
  telemetry_.Register();
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

  // Current ADC after CSA is configured/calibrated on the DRV.
  if (!current_adc_->Init())
  {
    return false;
  }
  if (!current_adc_->CalibrateOffset())
  {
    return false;
  }

  // PWM after offset cal; leave HiZ + duty=0 until "raw" / "vfoc" / "dq".
  if (!phase_pwm_->Init())
  {
    return false;
  }

  // Arm TIM5 CC4→DMA→LPTIM1→ADC, then enable CC4 DMA requests.
  if (!current_adc_->StartPwmSync())
  {
    return false;
  }
  phase_pwm_->EnableAdcTrigger();

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
    control::CurrentLoop::Duties duties;
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
    return;
  }

  control::VoltageFoc::Duties duties;
  if (!voltage_foc_->Step(dt_s, &duties))
  {
    return;
  }
  phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
  gate_driver_->PowerOn();
  pwm_output_on_ = true;
  ObserveDqFromSample(voltage_foc_->theta_rad(), sample);
}

void Application::PollCan()
{
  const auto st = can_->status();
  if (st.BusOff)
  {
    can_->RecoverBusOff();
    return;
  }

  // Drain the RX FIFO: BOOT request or multiplex telemetry tunnel.
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

    (void)diagnostic_->HandleFrame(header, data, len);
  }

  diagnostic_->RunLine();
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

bool Application::DiagCommandThunk(void* context, std::string_view verb,
                                   mjlib::base::Tokenizer& tokenizer,
                                   mjlib::base::BufferWriteStream& writer)
{
  if (context == nullptr)
  {
    return false;
  }
  return static_cast<Application*>(context)->HandleDiagCommand(verb, tokenizer,
                                                               writer);
}

bool Application::HandleDiagCommand(std::string_view verb,
                                    mjlib::base::Tokenizer& tokenizer,
                                    mjlib::base::BufferWriteStream& writer)
{
  if (verb == "raw")
  {
    return HandleRawCommand(tokenizer, writer);
  }
  if (verb == "vfoc")
  {
    return HandleVfocCommand(tokenizer, writer);
  }
  if (verb == "dq")
  {
    return HandleDqCommand(tokenizer, writer);
  }
  if (verb == "stop")
  {
    StopOutput();
    writer.write("OK stop\r\n");
    return true;
  }
  return false;
}

bool Application::HandleRawCommand(mjlib::base::Tokenizer& tokenizer,
                                   mjlib::base::BufferWriteStream& writer)
{
  if (state_ != State::RUN)
  {
    writer.write("ERR not in RUN\r\n");
    return true;
  }
  if (!phase_pwm_->init_ok())
  {
    writer.write("ERR pwm not ready\r\n");
    return true;
  }

  uint16_t a = 0;
  uint16_t b = 0;
  uint16_t c = 0;
  if (!ParseU16(tokenizer.next(), &a) || !ParseU16(tokenizer.next(), &b) ||
      !ParseU16(tokenizer.next(), &c))
  {
    writer.write("ERR usage: raw a b c (0-1000 milli)\r\n");
    return true;
  }
  if (a > hal::PhasePwm::kDutyMax || b > hal::PhasePwm::kDutyMax ||
      c > hal::PhasePwm::kDutyMax)
  {
    writer.write("ERR duty > 1000\r\n");
    return true;
  }

  voltage_foc_->Stop();
  current_loop_->Stop();
  phase_pwm_->DisableControlIsr();
  dq_valid_ = false;

  phase_pwm_->SetDuty(a, b, c);
  gate_driver_->PowerOn();
  pwm_output_on_ = true;
  writer.write("OK raw\r\n");
  return true;
}

bool Application::HandleVfocCommand(mjlib::base::Tokenizer& tokenizer,
                                    mjlib::base::BufferWriteStream& writer)
{
  if (state_ != State::RUN)
  {
    writer.write("ERR not in RUN\r\n");
    return true;
  }
  if (!phase_pwm_->init_ok())
  {
    writer.write("ERR pwm not ready\r\n");
    return true;
  }

  float theta = 0.0f;
  float voltage = 0.0f;
  if (!ParseF32(tokenizer.next(), &theta) ||
      !ParseF32(tokenizer.next(), &voltage))
  {
    writer.write("ERR usage: vfoc theta_rad V [rate_rad_s]\r\n");
    return true;
  }

  float rate = 0.0f;
  const auto rate_tok = tokenizer.next();
  if (!rate_tok.empty() && !ParseF32(rate_tok, &rate))
  {
    writer.write("ERR bad rate\r\n");
    return true;
  }

  current_loop_->Stop();
  phase_pwm_->DisableControlIsr();

  control::VoltageFoc::Command cmd;
  cmd.theta_rad = theta;
  cmd.voltage_V = voltage;
  cmd.theta_rate_rad_s = rate;
  voltage_foc_->Start(cmd);

  // Seed first duties then let PWM-rate ISR continue.
  control::VoltageFoc::Duties duties;
  if (!voltage_foc_->Step(0.0f, &duties))
  {
    writer.write("ERR foc step\r\n");
    voltage_foc_->Stop();
    return true;
  }
  phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
  gate_driver_->PowerOn();
  pwm_output_on_ = true;
  last_current_ = current_adc_->ReadLatest();
  ObserveDqFromSample(voltage_foc_->theta_rad(), last_current_);
  StartControlIsr();

  writer.write("OK vfoc\r\n");
  return true;
}

bool Application::HandleDqCommand(mjlib::base::Tokenizer& tokenizer,
                                  mjlib::base::BufferWriteStream& writer)
{
  if (state_ != State::RUN)
  {
    writer.write("ERR not in RUN\r\n");
    return true;
  }
  if (!phase_pwm_->init_ok())
  {
    writer.write("ERR pwm not ready\r\n");
    return true;
  }

  // dq <id_A> <iq_A> [theta_rate_rad_s]  (θ starts at 0)
  float id = 0.0f;
  float iq = 0.0f;
  if (!ParseF32(tokenizer.next(), &id) || !ParseF32(tokenizer.next(), &iq))
  {
    writer.write("ERR usage: dq id_A iq_A [rate_rad_s]\r\n");
    return true;
  }

  float rate = 0.0f;
  const auto rate_tok = tokenizer.next();
  if (!rate_tok.empty() && !ParseF32(rate_tok, &rate))
  {
    writer.write("ERR bad rate\r\n");
    return true;
  }

  voltage_foc_->Stop();
  phase_pwm_->DisableControlIsr();

  control::CurrentLoop::Command cmd;
  cmd.theta_rad = 0.0f;
  cmd.id_A = id;
  cmd.iq_A = iq;
  cmd.theta_rate_rad_s = rate;
  current_loop_->Start(cmd);

  last_current_ = current_adc_->ReadLatest();
  control::CurrentLoop::Duties duties;
  if (!current_loop_->Step(0.0f, last_current_.i1_A, last_current_.i2_A,
                           last_current_.i3_A, &duties))
  {
    writer.write("ERR dq step\r\n");
    current_loop_->Stop();
    return true;
  }
  id_A_ = current_loop_->id_A();
  iq_A_ = current_loop_->iq_A();
  dq_valid_ = last_current_.ok;
  phase_pwm_->SetDuty(duties.a_milli, duties.b_milli, duties.c_milli);
  gate_driver_->PowerOn();
  pwm_output_on_ = true;
  StartControlIsr();

  writer.write("OK dq\r\n");
  return true;
}

}  // namespace app
