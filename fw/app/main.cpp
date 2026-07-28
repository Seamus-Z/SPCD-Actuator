// CRT entry only. Application lives in application.cc.
#include <new>

#include "application.h"
#include "stm32g4xx.h"

namespace
{
// Large object off the MSP stack (moteus keeps big state in pool/static storage).
alignas(app::Application) uint8_t g_application_storage[sizeof(app::Application)];

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
    GPIOB->BSRR = 0x80000000;  // LED on (active low assumption may differ)
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
  __enable_irq();

  auto* application = new (g_application_storage) app::Application();
  application->Run();
}

}  // extern "C"
