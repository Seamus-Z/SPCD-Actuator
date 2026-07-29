#include "HAL/phase_current_adc.h"

#include "stm32g4xx.h"
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

DMAMUX_Channel_TypeDef* SelectDmamux(DMA_Channel_TypeDef* channel)
{
  // G474: DMA1 → DMAMUX Ch0-7, DMA2 → DMAMUX Ch8-15.
  const bool dma2 = channel >= DMA2_Channel1;
  const uint32_t base = reinterpret_cast<uint32_t>(
      dma2 ? DMAMUX1_Channel8 : DMAMUX1_Channel0);
  const uint32_t ch0 = reinterpret_cast<uint32_t>(
      dma2 ? DMA2_Channel1 : DMA1_Channel1);
  const uint32_t stride =
      reinterpret_cast<uint32_t>(DMA1_Channel2) -
      reinterpret_cast<uint32_t>(DMA1_Channel1);
  const uint32_t index =
      (reinterpret_cast<uint32_t>(channel) - ch0) / stride;
  const uint32_t mux_stride =
      reinterpret_cast<uint32_t>(DMAMUX1_Channel1) -
      reinterpret_cast<uint32_t>(DMAMUX1_Channel0);
  return reinterpret_cast<DMAMUX_Channel_TypeDef*>(base + index * mux_stride);
}

}  // namespace

PhaseCurrentAdc::PhaseCurrentAdc(MillisecondTimer* timer, const Options& options)
    : timer_(timer), options_(options)
{
  if (options_.sense_ohm > 0.0f && options_.csa_gain > 0.0f)
  {
    scale_ = 3.3f / (4096.0f * options_.sense_ohm * options_.csa_gain);
  }
}

void PhaseCurrentAdc::SetError(Error err)
{
  last_error_ = err;
}

bool PhaseCurrentAdc::Init()
{
  init_ok_ = false;
  offset_ok_ = false;
  sync_on_ = false;
  last_error_ = Error::None;

  if (timer_ == nullptr || scale_ <= 0.0f)
  {
    SetError(Error::BadConfig);
    return false;
  }

  ConfigureGpioAnalog();
  ConfigureAdcCommon();

  DisableOneAdc(ADC1);
  DisableOneAdc(ADC2);
  DisableOneAdc(ADC3);

  if (!EnableOneAdcSoftware(ADC1) || !EnableOneAdcSoftware(ADC2) ||
      !EnableOneAdcSoftware(ADC3))
  {
    SetError(Error::EnableFail);
    return false;
  }

  ConfigureChannels();
  init_ok_ = true;
  return true;
}

bool PhaseCurrentAdc::CalibrateOffset()
{
  if (!init_ok_)
  {
    if (last_error_ == Error::None)
    {
      SetError(Error::BadConfig);
    }
    return false;
  }

  uint32_t sum1 = 0;
  uint32_t sum2 = 0;
  uint32_t sum3 = 0;
  for (uint16_t i = 0; i < kCalibrateCount; ++i)
  {
    uint16_t r1 = 0;
    uint16_t r2 = 0;
    uint16_t r3 = 0;
    // Always software-trigger for offset (HiZ, before PWM sync).
    if (!ReadRawSoftware(&r1, &r2, &r3))
    {
      offset_ok_ = false;
      return false;
    }
    sum1 += r1;
    sum2 += r2;
    sum3 += r3;
  }

  offset1_ = static_cast<uint16_t>(sum1 / kCalibrateCount);
  offset2_ = static_cast<uint16_t>(sum2 / kCalibrateCount);
  offset3_ = static_cast<uint16_t>(sum3 / kCalibrateCount);

  const auto near_mid = [](uint16_t v) {
    const int d = static_cast<int>(v) - static_cast<int>(kExpectedMid);
    return d <= static_cast<int>(kMidTolerance) &&
           d >= -static_cast<int>(kMidTolerance);
  };

  offset_ok_ = near_mid(offset1_) && near_mid(offset2_) && near_mid(offset3_);
  if (!offset_ok_)
  {
    SetError(Error::OffsetOutOfRange);
    return false;
  }

  last_error_ = Error::None;
  return true;
}

bool PhaseCurrentAdc::StartPwmSync()
{
  if (!init_ok_)
  {
    SetError(Error::BadConfig);
    return false;
  }

  if (!ConfigureLptim1())
  {
    SetError(Error::SyncFail);
    return false;
  }
  if (!ConfigureDmaLptimTrigger())
  {
    SetError(Error::SyncFail);
    return false;
  }

  // Switch all three ADCs to LPTIM1_OUT rising-edge trigger and arm.
  if (!ArmHardwareTrigger(ADC1) || !ArmHardwareTrigger(ADC2) ||
      !ArmHardwareTrigger(ADC3))
  {
    SetError(Error::SyncFail);
    return false;
  }

  sync_on_ = true;
  last_error_ = Error::None;
  return true;
}

PhaseCurrentAdc::Sample PhaseCurrentAdc::Read()
{
  Sample sample;
  if (!init_ok_)
  {
    if (last_error_ == Error::None)
    {
      SetError(Error::BadConfig);
    }
    return sample;
  }

  if (!ReadRawFamily3(&sample.raw1, &sample.raw2, &sample.raw3))
  {
    return sample;
  }

  sample.i1_A = (static_cast<int>(sample.raw1) - static_cast<int>(offset1_)) * scale_;
  sample.i2_A = (static_cast<int>(sample.raw2) - static_cast<int>(offset2_)) * scale_;
  sample.i3_A = (static_cast<int>(sample.raw3) - static_cast<int>(offset3_)) * scale_;
  sample.ok = true;
  return sample;
}

PhaseCurrentAdc::Sample PhaseCurrentAdc::ReadLatest()
{
  Sample sample;
  if (!init_ok_ || !sync_on_)
  {
    return sample;
  }

  // Family-3 remap: logical i1←ADC3, i2←ADC2, i3←ADC1 (same as synced path).
  const uint16_t adc1 = static_cast<uint16_t>(ADC1->DR);
  const uint16_t adc2 = static_cast<uint16_t>(ADC2->DR);
  const uint16_t adc3 = static_cast<uint16_t>(ADC3->DR);
  sample.raw1 = adc3;
  sample.raw2 = adc2;
  sample.raw3 = adc1;
  sample.i1_A =
      (static_cast<int>(sample.raw1) - static_cast<int>(offset1_)) * scale_;
  sample.i2_A =
      (static_cast<int>(sample.raw2) - static_cast<int>(offset2_)) * scale_;
  sample.i3_A =
      (static_cast<int>(sample.raw3) - static_cast<int>(offset3_)) * scale_;
  sample.ok = true;
  // Do not bump seq_ here: ISR runs at 15 kHz and huge seq digits blow the
  // telemetry line past the response buffer (host then never sees '\n').
  return sample;
}

void PhaseCurrentAdc::ConfigureGpioAnalog()
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  SetAnalogMode(GPIOA, 3);
  SetAnalogMode(GPIOA, 6);
  SetAnalogMode(GPIOB, 1);
  (void)options_;
}

void PhaseCurrentAdc::ConfigureAdcCommon()
{
  __HAL_RCC_ADC12_CLK_ENABLE();
  __HAL_RCC_ADC345_CLK_ENABLE();

  // AHB/2 sync clock. Keep DUAL=0; simultaneous start via LPTIM satisfies
  // ES0430 §2.7.11 without dual-mode register coupling.
  ADC12_COMMON->CCR = (2u << ADC_CCR_CKMODE_Pos);
  ADC345_COMMON->CCR = (2u << ADC_CCR_CKMODE_Pos);
}

void PhaseCurrentAdc::DisableOneAdc(ADC_TypeDef* adc)
{
  if (adc->CR & ADC_CR_ADEN)
  {
    adc->CR |= ADC_CR_ADDIS;
    uint32_t timeout = 100000;
    while (adc->CR & ADC_CR_ADEN)
    {
      if (--timeout == 0)
      {
        break;
      }
    }
  }
}

bool PhaseCurrentAdc::EnableOneAdcSoftware(ADC_TypeDef* adc)
{
  adc->CR &= ~ADC_CR_DEEPPWD;
  adc->CR |= ADC_CR_ADVREGEN;
  timer_->wait_us(20);

  adc->CR |= ADC_CR_ADCAL;
  uint32_t timeout = 100000;
  while (adc->CR & ADC_CR_ADCAL)
  {
    if (--timeout == 0)
    {
      return false;
    }
  }
  timer_->wait_us(1);

  adc->ISR |= ADC_ISR_ADRDY;
  adc->CR |= ADC_CR_ADEN;
  timeout = 100000;
  while (!(adc->ISR & ADC_ISR_ADRDY))
  {
    if (--timeout == 0)
    {
      return false;
    }
  }
  adc->ISR |= ADC_ISR_ADRDY;

  adc->CFGR &= ~(ADC_CFGR_CONT | ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN);
  return true;
}

void PhaseCurrentAdc::ConfigureChannels()
{
  ADC1->SQR1 = (0u << ADC_SQR1_L_Pos) | (4u << ADC_SQR1_SQ1_Pos);
  ADC2->SQR1 = (0u << ADC_SQR1_L_Pos) | (3u << ADC_SQR1_SQ1_Pos);
  ADC3->SQR1 = (0u << ADC_SQR1_L_Pos) | (1u << ADC_SQR1_SQ1_Pos);

  const uint32_t smpr = MakeAllSampleTimes(options_.sample_time_code);
  ADC1->SMPR1 = smpr;
  ADC1->SMPR2 = smpr;
  ADC2->SMPR1 = smpr;
  ADC2->SMPR2 = smpr;
  ADC3->SMPR1 = smpr;
  ADC3->SMPR2 = smpr;
}

bool PhaseCurrentAdc::ConfigureLptim1()
{
  // PCLK1 as LPTIM1 kernel clock (CCIPR LPTIM1SEL = 00).
  RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;

  __HAL_RCC_LPTIM1_FORCE_RESET();
  __HAL_RCC_LPTIM1_RELEASE_RESET();
  __HAL_RCC_LPTIM1_CLK_ENABLE();

  // Minimal register init (matches moteus ARR=4 / CMP=1 / DIV4).
  LPTIM1->CR = 0;
  LPTIM1->CFGR = (2u << LPTIM_CFGR_PRESC_Pos);  // DIV4
  LPTIM1->CR |= LPTIM_CR_ENABLE;

  LPTIM1->ICR = LPTIM_ICR_ARROKCF;
  LPTIM1->ARR = 4;
  uint32_t timeout = 100000;
  while (!(LPTIM1->ISR & LPTIM_ISR_ARROK))
  {
    if (--timeout == 0)
    {
      return false;
    }
  }

  LPTIM1->ICR = LPTIM_ICR_CMPOKCF;
  LPTIM1->CMP = 1;
  timeout = 100000;
  while (!(LPTIM1->ISR & LPTIM_ISR_CMPOK))
  {
    if (--timeout == 0)
    {
      return false;
    }
  }
  return true;
}

bool PhaseCurrentAdc::ConfigureDmaLptimTrigger()
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMAMUX1_CLK_ENABLE();

  DMA_Channel_TypeDef* dma = DMA2_Channel5;
  DMAMUX_Channel_TypeDef* dmamux = SelectDmamux(dma);

  dma->CCR = 0;
  dmamux->CCR = kTim5Ch4Dmamux;

  // Must be a mutable SRAM word (DMA memory→peripheral).
  lptim1_sngstrt_value_ = LPTIM_CR_ENABLE | LPTIM_CR_SNGSTRT;

  dma->CPAR = reinterpret_cast<uint32_t>(&LPTIM1->CR);
  dma->CMAR = reinterpret_cast<uint32_t>(&lptim1_sngstrt_value_);
  dma->CNDTR = 1;
  dma->CCR =
      DMA_CCR_CIRC |
      (2u << DMA_CCR_MSIZE_Pos) |
      (2u << DMA_CCR_PSIZE_Pos) |
      DMA_CCR_DIR |
      DMA_CCR_EN;
  return true;
}

bool PhaseCurrentAdc::ArmHardwareTrigger(ADC_TypeDef* adc)
{
  // If a software conversion is lingering, stop it first.
  if (adc->CR & ADC_CR_ADSTART)
  {
    adc->CR |= ADC_CR_ADSTP;
    uint32_t timeout = 100000;
    while (adc->CR & ADC_CR_ADSTART)
    {
      if (--timeout == 0)
      {
        return false;
      }
    }
  }

  // EXTSEL=0x1D (LPTIM_OUT), EXTEN=rising edge. Then ADSTART arms for HW.
  // OVRMOD=1: DR overwrites on overrun. Required because TIM5 triggers at
  // pwm_rate (e.g. 15 kHz) while telemetry may poll much slower — default
  // OVRMOD=0 freezes DR on the first unread conversion (matches frozen
  // stream: identical raw forever with samp=1).
  adc->CFGR =
      (adc->CFGR & ~(ADC_CFGR_CONT | ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN)) |
      (0x1Du << ADC_CFGR_EXTSEL_Pos) |
      (0x1u << ADC_CFGR_EXTEN_Pos) |
      ADC_CFGR_OVRMOD;
  adc->ISR = ADC_ISR_EOS | ADC_ISR_EOC | ADC_ISR_EOSMP | ADC_ISR_OVR;
  adc->CR |= ADC_CR_ADSTART;
  return true;
}

bool PhaseCurrentAdc::WaitFlag(ADC_TypeDef* adc, uint32_t flag)
{
  // ~several PWM periods at 15 kHz; keep generous for 16 MHz bring-up.
  uint32_t timeout = 500000;
  while (!(adc->ISR & flag))
  {
    if (--timeout == 0)
    {
      return false;
    }
  }
  return true;
}

bool PhaseCurrentAdc::ReadRawSoftware(uint16_t* raw1, uint16_t* raw2,
                                      uint16_t* raw3)
{
  *raw1 = 0;
  *raw2 = 0;
  *raw3 = 0;

  ADC1->CR |= ADC_CR_ADSTART;
  ADC2->CR |= ADC_CR_ADSTART;
  ADC3->CR |= ADC_CR_ADSTART;

  if (!WaitFlag(ADC1, ADC_ISR_EOC) || !WaitFlag(ADC2, ADC_ISR_EOC) ||
      !WaitFlag(ADC3, ADC_ISR_EOC))
  {
    SetError(Error::SampleTimeout);
    return false;
  }

  const uint16_t adc1 = static_cast<uint16_t>(ADC1->DR);
  const uint16_t adc2 = static_cast<uint16_t>(ADC2->DR);
  const uint16_t adc3 = static_cast<uint16_t>(ADC3->DR);
  *raw1 = adc3;
  *raw2 = adc2;
  *raw3 = adc1;
  return true;
}

bool PhaseCurrentAdc::ReadRawSynced(uint16_t* raw1, uint16_t* raw2,
                                    uint16_t* raw3)
{
  *raw1 = 0;
  *raw2 = 0;
  *raw3 = 0;

  // Re-arm if a previous fault cleared ADSTART.
  if (!(ADC1->CR & ADC_CR_ADSTART))
  {
    ADC1->CR |= ADC_CR_ADSTART;
  }
  if (!(ADC2->CR & ADC_CR_ADSTART))
  {
    ADC2->CR |= ADC_CR_ADSTART;
  }
  if (!(ADC3->CR & ADC_CR_ADSTART))
  {
    ADC3->CR |= ADC_CR_ADSTART;
  }

  // Clear stale flags, then require a fresh EOS edge (0 → 1).
  ADC1->ISR = ADC_ISR_EOS | ADC_ISR_EOC | ADC_ISR_EOSMP | ADC_ISR_OVR;
  ADC2->ISR = ADC_ISR_EOS | ADC_ISR_EOC | ADC_ISR_EOSMP | ADC_ISR_OVR;
  ADC3->ISR = ADC_ISR_EOS | ADC_ISR_EOC | ADC_ISR_EOSMP | ADC_ISR_OVR;

  // Confirm clear took effect (detect sticky-EOS bugs).
  uint32_t timeout = 100000;
  while ((ADC1->ISR & ADC_ISR_EOS) || (ADC2->ISR & ADC_ISR_EOS) ||
         (ADC3->ISR & ADC_ISR_EOS))
  {
    ADC1->ISR = ADC_ISR_EOS | ADC_ISR_EOC | ADC_ISR_EOSMP | ADC_ISR_OVR;
    ADC2->ISR = ADC_ISR_EOS | ADC_ISR_EOC | ADC_ISR_EOSMP | ADC_ISR_OVR;
    ADC3->ISR = ADC_ISR_EOS | ADC_ISR_EOC | ADC_ISR_EOSMP | ADC_ISR_OVR;
    if (--timeout == 0)
    {
      SetError(Error::SampleTimeout);
      return false;
    }
  }

  if (!WaitFlag(ADC1, ADC_ISR_EOS) || !WaitFlag(ADC2, ADC_ISR_EOS) ||
      !WaitFlag(ADC3, ADC_ISR_EOS))
  {
    SetError(Error::SampleTimeout);
    return false;
  }

  const uint16_t adc1 = static_cast<uint16_t>(ADC1->DR);
  const uint16_t adc2 = static_cast<uint16_t>(ADC2->DR);
  const uint16_t adc3 = static_cast<uint16_t>(ADC3->DR);
  *raw1 = adc3;
  *raw2 = adc2;
  *raw3 = adc1;
  ++seq_;
  return true;
}

bool PhaseCurrentAdc::ReadRawFamily3(uint16_t* raw1, uint16_t* raw2,
                                     uint16_t* raw3)
{
  if (sync_on_)
  {
    return ReadRawSynced(raw1, raw2, raw3);
  }
  return ReadRawSoftware(raw1, raw2, raw3);
}

}  // namespace hal
