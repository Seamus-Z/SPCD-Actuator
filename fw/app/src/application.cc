#include "application.h"

#include <cstring>

#include "BL_Config.h"
#include "board_config.h"
#include "bootloader.h"
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

}  // namespace

Application::Application(::pool::Pool* pool)
    : pool_(pool),
      timer_(pool),
      can_(pool, board::CanOptions()),
      gate_driver_(pool, timer_.get(), board::GateDriverOptions()),
      current_adc_(pool, timer_.get(), board::CurrentSenseOptions()),
      phase_pwm_(pool, board::PhasePwmOptions()),
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

  // PWM after offset cal; leave HiZ + duty=0 until "raw".
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
    phase_pwm_->Stop();
  }
  if (gate_driver_.get() != nullptr)
  {
    gate_driver_->PowerOff();
  }
  pwm_output_on_ = false;
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

  // moteus order: write CCR first, then release HiZ.
  phase_pwm_->SetDuty(a, b, c);
  gate_driver_->PowerOn();
  pwm_output_on_ = true;
  writer.write("OK raw\r\n");
  return true;
}

}  // namespace app
