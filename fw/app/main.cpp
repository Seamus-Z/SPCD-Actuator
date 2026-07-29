// CRT entry only. Application and modules live in an explicit static pool.
#include <new>

#include "application.h"
#include "pool/pool.h"
#include "stm32g4xx.h"

namespace
{
// Raw storage for the pool object itself (constructed after BSS init).
alignas(::pool::SizedPool<8192>)
    uint8_t g_pool_storage[sizeof(::pool::SizedPool<8192>)];

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
}  // namespace

extern "C"
{

extern uint8_t __bss_start__;
extern uint8_t __bss_end__;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sidata;

void AppDefault(void) { FaultLedBlinkForever(); }
void AppHardFault(void) { FaultLedBlinkForever(); }

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

  // Enable CP10/CP11 before any float code (PhaseCurrentAdc ctor uses VFP).
  // Without this, softfp VFP ops HardFault and CAN boot entry is dead.
  SCB->CPACR |= (0xFu << 20);
  __DSB();
  __ISB();

  __enable_irq();

  // Bare-metal has no global ctor CRT: construct the pool explicitly.
  auto* pool = new (g_pool_storage) ::pool::SizedPool<8192>();
  ::pool::PoolPtr<app::Application> application(pool, pool);
  application->Run();
}

}  // extern "C"
