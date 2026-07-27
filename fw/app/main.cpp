// Consolidated application: state machine, CAN boot request, and CRT entry.
#include <cstring>

#include "HAL/fdcan.h"
#include "bootloader.h"
#include "stm32g4xx.h"

namespace app
{

enum class State
{
  INIT,
  RUN,
  ENTER_BOOTLOADER,
};

class Application
{
 public:
  Application();

  [[noreturn]] void Run();

  State state() const { return state_; }

 private:
  void Init();
  void RunOnce();
  void EnterBootloaderMode();
  bool PollBootRequest();

  State state_ = State::INIT;
  hal::FDCan can_;
};

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

hal::FDCan::Options MakeCanOptions()
{
  hal::FDCan::Options options;
  options.instance = FDCAN2;
  options.slow_bitrate = 1000000;
  options.fast_bitrate = 2000000;
  options.fdcan_frame = true;
  options.bitrate_switch = true;
  options.automatic_retransmission = false;
  return options;
}

}  // namespace

Application::Application() : can_(MakeCanOptions()) {}

void Application::Run()
{
  while (true)
  {
    switch (state_)
    {
      case State::INIT:
        Init();
        state_ = State::RUN;
        break;
      case State::RUN:
        RunOnce();
        break;
      case State::ENTER_BOOTLOADER:
        EnterBootloaderMode();
        break;
    }
  }
}

void Application::Init()
{
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE15_Msk) |
                 (1 << GPIO_MODER_MODE15_Pos);

  LedOn();
  DelayNops(2000000);
  LedOff();
  DelayNops(500000);
}

void Application::RunOnce()
{
  if (PollBootRequest())
  {
    state_ = State::ENTER_BOOTLOADER;
    return;
  }

  LedOn();
  DelayNops(100000);
  LedOff();
  DelayNops(100000);
}

bool Application::PollBootRequest()
{
  const auto st = can_.status();
  if (st.BusOff)
  {
    can_.RecoverBusOff();
    return false;
  }

  FDCAN_RxHeaderTypeDef header = {};
  uint8_t data[64] = {};
  size_t len = 0;
  if (!can_.Poll(&header, data, sizeof(data), &len))
  {
    return false;
  }

  if (header.IdType != FDCAN_STANDARD_ID ||
      header.Identifier != kBootRequestId ||
      header.RxFrameType != FDCAN_DATA_FRAME ||
      len != sizeof(kBootRequestPayload) - 1)
  {
    return false;
  }

  return std::memcmp(data, kBootRequestPayload,
                     sizeof(kBootRequestPayload) - 1) == 0;
}

void Application::EnterBootloaderMode()
{
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

extern "C"
{

extern uint8_t __bss_start__;
extern uint8_t __bss_end__;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sidata;

void AppDefault(void) { while (1) {} }
void AppHardFault(void) { while (1) {} }

void AppReset(void)
{
  for (uint8_t* p = &__bss_start__; p < &__bss_end__; ++p)
  {
    *p = 0;
  }
  {
    uint32_t* dst = &_sdata;
    const uint32_t* src = &_sidata;
    while (dst < &_edata)
    {
      *dst++ = *src++;
    }
  }

  SCB->VTOR = 0x08010000;
  SystemCoreClock = 16000000;
  __enable_irq();

  app::Application application;
  application.Run();
}

}  // extern "C"
