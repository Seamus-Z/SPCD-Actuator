// Bootloader core: BootOrchestrator FSM + CRT entry.
//
// ★ MAIN is BootloaderEntry() near the bottom (no libc main() in bare-metal).
// Read order:
//   1) helpers
//   2) BootOrchestrator::Run and each state handler
//   3) BootloaderEntry  — reset vector lands here (= main)
//   4) fault handlers   — only if something goes wrong
#include <cstdint>
#include <cstring>

#include "stm32g4xx.h"
#include "bootloader.h"
#include "BL_Config.h"
#include "BL_CommandServer.h"

namespace boot
{
namespace
{

void LedOn() { GPIOB->BSRR = 0x80000000; }
void LedOff() { GPIOB->BSRR = 0x00008000; }

void DelayNops(uint32_t count)
{
  for (volatile uint32_t d = 0; d < count; d++)
  {
    __NOP();
  }
}

void ClockInit()
{
  while (!(RCC->CR & RCC_CR_HSIRDY))
  {
  }
  SystemCoreClock = kSystemClockHz;
  RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_FDCANSEL_Msk) |
               (2UL << RCC_CCIPR_FDCANSEL_Pos);
}

void DwtInit()
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
}

bool IsBootloaderRequested()
{
  return *reinterpret_cast<uint32_t*>(kBootMagicAddr) == kBootMagicValue;
}

}  // namespace

enum class BootState : uint8_t
{
  INIT_HARDWARE,
  DECIDE,
  JUMP_TO_APP,
  ENTER_SERVE,
  SERVE,
};

// State machine: InitHardware → Decide → JumpToApp | EnterServe → Serve.
// Methods are defined inline in that same order.
class BootOrchestrator
{
 public:
  void Run()
 {
    while (true)
    {
      switch (state_)
      {
        case BootState::INIT_HARDWARE:
          InitHardware();
          state_ = BootState::DECIDE;
          break;
        case BootState::DECIDE:
          Decide();
          break;
        case BootState::JUMP_TO_APP:
          if (!JumpToApp(kAppStart))
        {
            state_ = BootState::ENTER_SERVE;
          }
          break;
        case BootState::ENTER_SERVE:
          EnterServe();
          state_ = BootState::SERVE;
          break;
        case BootState::SERVE:
          Serve();
          break;
      }
    }
  }

  BootState state() const { return state_; }

 private:
  void InitHardware()
 {
    ClockInit();
    DwtInit();

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE15_Msk) |
                   (1 << GPIO_MODER_MODE15_Pos);
    LedOff();
  }

  void Decide()
  {
    if (IsBootloaderRequested())
    {
      state_ = BootState::ENTER_SERVE;
      return;
    }

    for (int i = 0; i < 3; i++)
    {
      LedOn();
      DelayNops(200000);
      LedOff();
      DelayNops(200000);
    }
    state_ = BootState::JUMP_TO_APP;
  }

  bool JumpToApp(uint32_t app_base)
  {
    const uint32_t sp = *reinterpret_cast<uint32_t*>(app_base);
    const uint32_t pc = *reinterpret_cast<uint32_t*>(app_base + 4);

    if (sp < 0x20000200 || sp > 0x20020000 || (sp & 0x7))
    {
      return false;
    }
    if (pc < kAppStart || pc >= kFlashEnd || !(pc & 1))
    {
      return false;
    }

    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    for (int i = 0; i < 8; i++)
    {
      NVIC->ICER[i] = 0xFFFFFFFF;
      NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    SCB->VTOR = app_base;
    __set_CONTROL(0);
    __set_BASEPRI(0);
    __set_FAULTMASK(0);
    __DSB();
    __ISB();

    __set_MSP(sp);
    __enable_irq();
    __asm volatile("bx %0\n" ::"r"(pc) : "memory");
    return true;
  }

  void EnterServe()
  {
    for (int i = 0; i < 4; i++)
    {
      LedOn();
      DelayNops(400000);
      LedOff();
      DelayNops(400000);
    }
    __disable_irq();
  }

  [[noreturn]] void Serve()
  {
    BL_CommandServer server(kDefaultCanId, FDCAN2);
    while (true)
    {
      server.Step();
    }
  }

  BootState state_ = BootState::INIT_HARDWARE;
};

}  // namespace boot

// =============================================================================
// MAIN — bootloader has no main(); BootloaderEntry is the reset-vector entry.
// vectors.S → BootloaderEntry → CRT init → BootOrchestrator::Run()
// =============================================================================

extern "C"
{

extern uint8_t __bss_start__;
extern uint8_t __bss_end__;
extern char _sdata;
extern char _edata;
extern char _sidata;

// Zero .bss (uninitialized globals). Linker provides the symbol range.
static void CrtInitBss()
{
  std::memset(&__bss_start__, 0, &__bss_end__ - &__bss_start__);
}

// Copy .data from Flash load address into RAM.
static void CrtInitData()
{
  char* dst = &_sdata;
  char* src = &_sidata;
  while (dst != &_edata)
  {
    *dst++ = *src++;
  }
}

}  // extern "C"

extern "C" __attribute__((section(".boot_entry"))) void BootloaderEntry()
{
  CrtInitBss();
  CrtInitData();

  // Hand off to the boot state machine (never returns on Serve path).
  boot::BootOrchestrator{}.Run();
}

// =============================================================================
// Fault paths (not part of the happy boot flow)
// =============================================================================

extern "C"
{

static void FaultBlinkReset()
{
  for (int i = 0; i < 20; i++)
  {
    GPIOB->BSRR = 0x80000000;
    for (volatile uint32_t d = 0; d < 100000; d++)
    {
      __NOP();
    }
    GPIOB->BSRR = 0x00008000;
    for (volatile uint32_t d = 0; d < 100000; d++)
    {
      __NOP();
    }
  }
  NVIC_SystemReset();
  while (1)
  {
  }
}

void NMI_Handler(void) { FaultBlinkReset(); }
void HardFault_Handler(void) { FaultBlinkReset(); }
void MemManage_Handler(void) { FaultBlinkReset(); }
void BusFault_Handler(void) { FaultBlinkReset(); }
void UsageFault_Handler(void) { FaultBlinkReset(); }
void SVC_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) {}
void Default_Handler(void) { FaultBlinkReset(); }

}  // extern "C"

namespace mjlib
{
namespace base
{
void __attribute__((weak)) assertion_failed(const char* /*expression*/,
                                             const char* /*filename*/,
                                             int /*line*/)
{
  while (1)
  {
  }
}
}  // namespace base
}  // namespace mjlib
