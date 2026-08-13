#include "HAL/vt_sense_adc.h"

#include "math/protection/thermistor.h"
#include "stm32g4xx_hal.h"

namespace hal
{
namespace
{

void SetAnalogMode(GPIO_TypeDef* port, unsigned pin)
{
  port->MODER = (port->MODER & ~(3u << (pin * 2u))) | (3u << (pin * 2u));
  port->PUPDR &= ~(3u << (pin * 2u));
}

uint32_t MakeAllSampleTimes(uint16_t code)
{
  const uint32_t v = code & 0x7u;
  uint32_t out = 0;
  for (int i = 0; i < 9; ++i)
  {
    out |= (v << (i * 3));
  }
  return out;
}

}  // namespace

VtSenseAdc::VtSenseAdc(MillisecondTimer* timer, const Options& options)
    : timer_(timer), options_(options)
{
}

void VtSenseAdc::ConfigureGpioAnalog()
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  SetAnalogMode(GPIOA, 9);
  SetAnalogMode(GPIOB, 12);
}

void VtSenseAdc::DisableOneAdc(ADC_TypeDef* adc)
{
  if (adc->CR & ADC_CR_ADEN)
  {
    adc->CR |= ADC_CR_ADDIS;
    uint32_t timeout = 100000;
    while ((adc->CR & ADC_CR_ADEN) && --timeout != 0)
    {
    }
  }
}

bool VtSenseAdc::EnableOneAdcSoftware(ADC_TypeDef* adc)
{
  adc->CR &= ~ADC_CR_DEEPPWD;
  adc->CR |= ADC_CR_ADVREGEN;
  timer_->wait_us(20);

  adc->CR |= ADC_CR_ADCAL;
  uint32_t timeout = 100000;
  while ((adc->CR & ADC_CR_ADCAL) && --timeout != 0)
  {
  }
  if (timeout == 0)
  {
    return false;
  }
  timer_->wait_us(1);

  adc->ISR |= ADC_ISR_ADRDY;
  adc->CR |= ADC_CR_ADEN;
  timeout = 100000;
  while (!(adc->ISR & ADC_ISR_ADRDY) && --timeout != 0)
  {
  }
  if (timeout == 0)
  {
    return false;
  }
  adc->ISR |= ADC_ISR_ADRDY;
  adc->CFGR &= ~(ADC_CFGR_CONT | ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN);
  return true;
}

bool VtSenseAdc::ArmHardwareTrigger(ADC_TypeDef* adc)
{
  if (adc->CR & ADC_CR_ADSTART)
  {
    adc->CR |= ADC_CR_ADSTP;
    uint32_t timeout = 100000;
    while ((adc->CR & ADC_CR_ADSTART) && --timeout != 0)
    {
    }
    if (timeout == 0)
    {
      return false;
    }
  }
  adc->CFGR =
      (adc->CFGR & ~(ADC_CFGR_CONT | ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN)) |
      (0x1Du << ADC_CFGR_EXTSEL_Pos) |
      (0x1u << ADC_CFGR_EXTEN_Pos) |
      ADC_CFGR_OVRMOD;
  adc->ISR = ADC_ISR_EOS | ADC_ISR_EOC | ADC_ISR_EOSMP | ADC_ISR_OVR;
  adc->CR |= ADC_CR_ADSTART;
  return true;
}

bool VtSenseAdc::Init()
{
  init_ok_ = false;
  sync_on_ = false;
  latest_ = Sample{};
  if (timer_ == nullptr || options_.vsense_adc_scale <= 0.0f)
  {
    return false;
  }

  ConfigureGpioAnalog();
  __HAL_RCC_ADC345_CLK_ENABLE();

  DisableOneAdc(ADC4);
  DisableOneAdc(ADC5);
  if (!EnableOneAdcSoftware(ADC4) || !EnableOneAdcSoftware(ADC5))
  {
    return false;
  }

  // ADC4 = FET NTC (IN3 / PB12). ADC5 = bus vsense (IN2 / PA9).
  ADC4->SQR1 = (0u << ADC_SQR1_L_Pos) | (3u << ADC_SQR1_SQ1_Pos);
  ADC5->SQR1 = (0u << ADC_SQR1_L_Pos) | (2u << ADC_SQR1_SQ1_Pos);
  const uint32_t smpr = MakeAllSampleTimes(options_.sample_time_code);
  ADC4->SMPR1 = smpr;
  ADC4->SMPR2 = smpr;
  ADC5->SMPR1 = smpr;
  ADC5->SMPR2 = smpr;
  init_ok_ = true;
  return true;
}

bool VtSenseAdc::StartPwmSync()
{
  if (!init_ok_)
  {
    return false;
  }
  if (!ArmHardwareTrigger(ADC4) || !ArmHardwareTrigger(ADC5))
  {
    return false;
  }
  sync_on_ = true;
  return true;
}

VtSenseAdc::Sample VtSenseAdc::ReadLatest()
{
  if (!init_ok_ || !sync_on_)
  {
    return latest_;
  }

  latest_.fet_raw = static_cast<uint16_t>(ADC4->DR);
  latest_.bus_raw = static_cast<uint16_t>(ADC5->DR);
  latest_.bus_V =
      static_cast<float>(latest_.bus_raw) * options_.vsense_adc_scale;
  latest_.bus_ok = latest_.bus_V > 1.0f && latest_.bus_V < 80.0f;

  math::protection::Thermistor fet;
  fet.r25_ohm = options_.fet_r25_ohm;
  latest_.fet_ok = fet.ToCelsius(latest_.fet_raw, &latest_.fet_temp_C);
  return latest_;
}

}  // namespace hal
