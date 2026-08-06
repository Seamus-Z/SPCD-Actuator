#include "HAL/system_clock.h"

#include "stm32g4xx.h"

namespace hal
{
namespace
{

[[noreturn]] void Halt()
{
  while (true)
  {
  }
}

void WaitBits(volatile uint32_t* reg, uint32_t mask, uint32_t value)
{
  uint32_t spins = 1000000u;
  while (spins-- != 0u)
  {
    if ((*reg & mask) == value)
    {
      return;
    }
  }
  Halt();
}

void ApplyBusAndPeriphClocks()
{
  // FLASH latency must cover 170 MHz before/while raising HCLK.
  FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | FLASH_ACR_LATENCY_6WS;
  WaitBits(&FLASH->ACR, FLASH_ACR_LATENCY_Msk, FLASH_ACR_LATENCY_6WS);

  // HCLK=/1 (170), APB1=/2 (85), APB2=/2 (85) — same as moteus SetupClock.
  RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk |
                             RCC_CFGR_PPRE2_Msk)) |
              RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2;

  // Switch SYSCLK to PLL if needed.
  if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
  {
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL;
    WaitBits(&RCC->CFGR, RCC_CFGR_SWS_Msk, RCC_CFGR_SWS_PLL);
  }

  // FDCAN ← PCLK1, ADC12/345 ← SYSCLK (moteus peripheral clock map).
  RCC->CCIPR =
      (RCC->CCIPR & ~(RCC_CCIPR_FDCANSEL_Msk | RCC_CCIPR_ADC12SEL_Msk |
                      RCC_CCIPR_ADC345SEL_Msk)) |
      RCC_CCIPR_FDCANSEL_1 | RCC_CCIPR_ADC12SEL_1 | RCC_CCIPR_ADC345SEL_1;

  SystemCoreClock = 170000000u;
}

void EnablePllFromHsi()
{
  // Range1 boost required for 170 MHz (clear R1MODE, VOS=Range1).
  RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
  PWR->CR5 &= ~PWR_CR5_R1MODE;
  PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS_Msk) | PWR_CR1_VOS_0;
  {
    uint32_t spins = 1000000u;
    while ((PWR->SR2 & PWR_SR2_VOSF) != 0u)
    {
      if (--spins == 0u)
      {
        Halt();
      }
    }
  }

  RCC->CR |= RCC_CR_HSION;
  WaitBits(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY);

  RCC->CRRCR |= RCC_CRRCR_HSI48ON;
  WaitBits(&RCC->CRRCR, RCC_CRRCR_HSI48RDY, RCC_CRRCR_HSI48RDY);

  // Cannot reconfigure PLL while it is the SYSCLK source.
  if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)
  {
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_HSI;
    WaitBits(&RCC->CFGR, RCC_CFGR_SWS_Msk, RCC_CFGR_SWS_HSI);
  }

  if ((RCC->CR & RCC_CR_PLLON) != 0u)
  {
    RCC->CR &= ~RCC_CR_PLLON;
    WaitBits(&RCC->CR, RCC_CR_PLLRDY, 0u);
  }

  // HSI/4 * 85 / 2 = 170 MHz on PLLR (same as mbed SetSysClock_PLL_HSI).
  // PLLQ/PLLR encode ((div >> 1) - 1); PLLP uses PLLPDIV = div.
  RCC->PLLCFGR =
      RCC_PLLCFGR_PLLSRC_HSI |
      (((4u - 1u) << RCC_PLLCFGR_PLLM_Pos) & RCC_PLLCFGR_PLLM) |
      ((85u << RCC_PLLCFGR_PLLN_Pos) & RCC_PLLCFGR_PLLN) |
      ((2u << RCC_PLLCFGR_PLLPDIV_Pos) & RCC_PLLCFGR_PLLPDIV) |
      ((((2u >> 1u) - 1u) << RCC_PLLCFGR_PLLQ_Pos) & RCC_PLLCFGR_PLLQ) |
      ((((2u >> 1u) - 1u) << RCC_PLLCFGR_PLLR_Pos) & RCC_PLLCFGR_PLLR) |
      RCC_PLLCFGR_PLLPEN | RCC_PLLCFGR_PLLQEN | RCC_PLLCFGR_PLLREN;

  RCC->CR |= RCC_CR_PLLON;
  WaitBits(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY);
}

}  // namespace

void SetupSystemClock()
{
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;

  FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

  const uint32_t pllcfgr = RCC->PLLCFGR;
  const bool pll_already =
      ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) &&
      ((pllcfgr & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI) &&
      (((pllcfgr & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) == (4u - 1u)) &&
      (((pllcfgr & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos) == 85u) &&
      (((pllcfgr & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) ==
       ((2u >> 1u) - 1u));

  if (!pll_already)
  {
    EnablePllFromHsi();
  }

  ApplyBusAndPeriphClocks();
}

}  // namespace hal
