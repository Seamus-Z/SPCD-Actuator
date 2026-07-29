#include "HAL/phase_pwm.h"

#include "stm32g4xx.h"

namespace hal
{
namespace
{

void SetAfPp(GPIO_TypeDef* port, unsigned pin, unsigned af)
{
  port->MODER = (port->MODER & ~(3u << (pin * 2u))) | (2u << (pin * 2u));
  port->OTYPER &= ~(1u << pin);
  port->PUPDR &= ~(3u << (pin * 2u));
  port->OSPEEDR |= (3u << (pin * 2u));
  volatile uint32_t* afr = (pin < 8u) ? &port->AFR[0] : &port->AFR[1];
  const unsigned shift = (pin % 8u) * 4u;
  *afr = (*afr & ~(0xFu << shift)) | ((af & 0xFu) << shift);
}

}  // namespace

PhasePwm::PhasePwm(const Options& options) : options_(options) {}

bool PhasePwm::Init()
{
  init_ok_ = false;
  adc_trigger_on_ = false;
  if (options_.rate_hz == 0)
  {
    return false;
  }

  ConfigureGpio();
  ConfigureTimer();
  SetDuty(0, 0, 0);
  init_ok_ = true;
  return true;
}

void PhasePwm::ConfigureGpio()
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  (void)options_;
  SetAfPp(GPIOA, 0, 2);
  SetAfPp(GPIOA, 1, 2);
  SetAfPp(GPIOA, 2, 2);
}

void PhasePwm::ConfigureTimer()
{
  __HAL_RCC_TIM5_CLK_ENABLE();

  // Center-aligned mode 2: f_pwm = TIMCLK / (2 * ARR).
  const uint32_t timclk = SystemCoreClock;
  arr_ = timclk / (2u * static_cast<uint32_t>(options_.rate_hz));
  if (arr_ < 2u)
  {
    arr_ = 2u;
  }

  TIM5->CR1 = 0;
  TIM5->DIER = 0;  // CC4DE enabled later via EnableAdcTrigger()
  TIM5->PSC = 0;
  TIM5->ARR = arr_;

  TIM5->CCMR1 =
      (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
      (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM5->CCMR2 =
      (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
  // CH4 stays frozen OC; compare event still fires for DMA without CC4E.
  TIM5->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
  TIM5->CCR1 = 0;
  TIM5->CCR2 = 0;
  TIM5->CCR3 = 0;
  // Peak sample point (CNT==ARR while counting up in CMS=2).
  TIM5->CCR4 = arr_;

  TIM5->CR1 =
      (2u << TIM_CR1_CMS_Pos) |
      TIM_CR1_ARPE;
  TIM5->EGR = TIM_EGR_UG;
  TIM5->CR1 |= TIM_CR1_CEN;
}

uint16_t PhasePwm::ClampDuty(uint16_t milli)
{
  if (milli == 0)
  {
    return 0;
  }
  if (milli < kDutyMinMilli)
  {
    return kDutyMinMilli;
  }
  if (milli > kDutyMaxMilli)
  {
    return kDutyMaxMilli;
  }
  return milli;
}

uint16_t PhasePwm::DutyToCcr(uint16_t milli) const
{
  return static_cast<uint16_t>(
      (static_cast<uint32_t>(milli) * arr_) / kDutyMax);
}

void PhasePwm::SetDuty(uint16_t a_milli, uint16_t b_milli, uint16_t c_milli)
{
  duty_a_ = ClampDuty(a_milli > kDutyMax ? kDutyMax : a_milli);
  duty_b_ = ClampDuty(b_milli > kDutyMax ? kDutyMax : b_milli);
  duty_c_ = ClampDuty(c_milli > kDutyMax ? kDutyMax : c_milli);
  TIM5->CCR1 = DutyToCcr(duty_a_);
  TIM5->CCR2 = DutyToCcr(duty_b_);
  TIM5->CCR3 = DutyToCcr(duty_c_);
}

void PhasePwm::Stop()
{
  SetDuty(0, 0, 0);
}

void PhasePwm::EnableAdcTrigger()
{
  TIM5->DIER |= TIM_DIER_CC4DE;
  adc_trigger_on_ = true;
}

void PhasePwm::DisableAdcTrigger()
{
  TIM5->DIER &= ~TIM_DIER_CC4DE;
  adc_trigger_on_ = false;
}

}  // namespace hal
