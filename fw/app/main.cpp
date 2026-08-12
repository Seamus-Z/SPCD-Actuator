// CRT entry only. Application and modules live in an explicit static pool.
#include <new>

#include "HAL/system_clock.h"
#include "board/xtellar_stm32g4/firmware_composition.h"
#include "core/memory/pool.h"
#include "stm32g4xx.h"

namespace
{
// Raw storage for the pool object itself (constructed after BSS init).
// Cogging-compensation table (1024 int8) plus measurement accumulators live
// inside Application, so the pool must be sized generously.
alignas(::core::memory::SizedPool<40960>)
    uint8_t g_pool_storage[sizeof(::core::memory::SizedPool<40960>)];

void FaultLedInit()
{
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE15_Msk) |
                 (1u << GPIO_MODER_MODE15_Pos);
}

void FaultLedBlinkForever()
{
  FaultLedInit();
  while (true)
  {
    GPIOB->BSRR = 0x80000000;  // LED on (active low)
    for (volatile uint32_t d = 0; d < 100000; ++d)
    {
      __NOP();
    }
    GPIOB->BSRR = 0x00008000;  // LED off
    for (volatile uint32_t d = 0; d < 100000; ++d)
    {
      __NOP();
    }
  }
}

// Fault-context motor kill. TIM5 keeps generating PWM autonomously after the
// CPU faults, so the gate drive must be cut with raw register writes before
// anything else (C++ object state cannot be trusted here).
// DRV8353S pins (board_config.h GateDriverOptions): HiZ = PC15, ENABLE = PC14.
void KillMotorOutputRaw()
{
  TIM5->CCR1 = 0;
  TIM5->CCR2 = 0;
  TIM5->CCR3 = 0;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
  __DSB();
  GPIOC->BSRR = (1u << 15) << 16;  // HiZ low: gate outputs off
  GPIOC->BSRR = (1u << 14) << 16;  // ENABLE low: driver disabled
}
}  // namespace

extern "C"
{

extern uint8_t __bss_start__;
extern uint8_t __bss_end__;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sidata;

void AppDefault(void)
{
  KillMotorOutputRaw();
  FaultLedBlinkForever();
}

void AppHardFault(void)
{
  KillMotorOutputRaw();
  FaultLedBlinkForever();
}

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

  // Enable CP10/CP11 before any float code (PhaseCurrentAdc ctor uses VFP).
  // Without this, softfp VFP ops HardFault and CAN boot entry is dead.
  SCB->CPACR |= (0xFu << 20);
  __DSB();
  __ISB();

  // HSI→PLL 170 MHz + moteus bus/periph map (safe if bootloader already did it).
  hal::SetupSystemClock();

  __enable_irq();

  // Bare-metal has no global ctor CRT: construct the pool explicitly.
  auto* pool = new (g_pool_storage) ::core::memory::SizedPool<40960>();
  ::core::memory::PoolPtr<app::board::FirmwareComposition> firmware(pool, pool);
  firmware->Run();
}

}  // extern "C"
