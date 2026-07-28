#include "application.h"

#include <cstring>

#include "BL_Config.h"
#include "board_config.h"
#include "bootloader.h"
#include "stm32g4xx.h"
#include "telemetry/text_format.h"

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

const char* StateName(State state)
{
  switch (state)
  {
    case State::INIT: return "INIT";
    case State::RUN: return "RUN";
    case State::DRIVER_FAULT: return "DRIVER_FAULT";
    case State::ENTER_BOOTLOADER: return "ENTER_BOOTLOADER";
  }
  return "?";
}

}  // namespace

Application::Application()
    : can_(board::CanOptions()),
      gate_driver_(&timer_, board::GateDriverOptions()),
      diagnostic_(&can_, kDefaultCanId, &registry_)
{
  RegisterTelemetry();
}

void Application::RegisterTelemetry()
{
  registry_.Register(device::Drv8353s::kTelemetryChannel,
                     &device::Drv8353s::TelemetryExport,
                     &gate_driver_);
  registry_.Register(kTelemetryChannel, &Application::TelemetryExport, this);
}

size_t Application::TelemetryExport(void* context, char* out, size_t out_capacity)
{
  if (context == nullptr)
  {
    return 0;
  }
  return static_cast<Application*>(context)->FormatTelemetry(out, out_capacity);
}

size_t Application::FormatTelemetry(char* out, size_t out_capacity) const
{
  if (out == nullptr || out_capacity == 0)
  {
    return 0;
  }

  using telemetry::text::AppendKeyUInt;
  using telemetry::text::AppendStr;
  size_t pos = 0;
  pos = AppendStr(out, out_capacity, pos, "state=");
  pos = AppendStr(out, out_capacity, pos, StateName(state_));
  pos = AppendKeyUInt(out, out_capacity, pos, "driver_ok",
                      gate_driver_.init_ok() ? 1u : 0u, true);
  return pos;
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

  if (!gate_driver_.Init(board::GateDriverConfig()))
  {
    return false;
  }

  LedOn();
  return true;
}

void Application::RunOnce()
{
  const auto driver_status = gate_driver_.ReadStatus();
  if (driver_status.fault || driver_status.fault_line)
  {
    gate_driver_.Disable();
    state_ = State::DRIVER_FAULT;
    return;
  }

  PollCan();
  LedOn();
}

void Application::DriverFault()
{
  gate_driver_.PowerOff();
  PollCan();

  LedOn();
  DelayNops(50000);
  LedOff();
  DelayNops(50000);
}

void Application::PollCan()
{
  const auto st = can_.status();
  if (st.BusOff)
  {
    can_.RecoverBusOff();
    return;
  }

  // Drain the RX FIFO: BOOT request or multiplex telemetry tunnel.
  for (int i = 0; i < 8; ++i)
  {
    FDCAN_RxHeaderTypeDef header = {};
    uint8_t data[64] = {};
    size_t len = 0;
    if (!can_.Poll(&header, data, sizeof(data), &len))
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

    (void)diagnostic_.HandleFrame(header, data, len);
  }

  diagnostic_.RunLine();
}

void Application::EnterBootloaderMode()
{
  gate_driver_.Disable();
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
