#include "board/xtellar_stm32g4/platform_service.h"

#include "bootloader.h"
#include "stm32g4xx.h"

namespace app::board
{

PlatformService::PlatformService(const Dependencies& dependencies)
    : timer_(dependencies.timer),
      gate_driver_(dependencies.gate_driver),
      current_adc_(dependencies.current_adc),
      vt_sense_(dependencies.vt_sense),
      phase_pwm_(dependencies.phase_pwm),
      gate_driver_config_(dependencies.gate_driver_config)
{
}

void PlatformService::StartWatchdog()
{
  // LSI (~32 kHz) / 32 gives an approximately 1 kHz watchdog tick.
  IWDG->KR = 0x0000CCCCu;
  IWDG->KR = 0x00005555u;
  IWDG->PR = 3u;
  IWDG->RLR = 500u;
  IWDG->KR = 0x0000AAAAu;
}

void PlatformService::FeedWatchdog()
{
  IWDG->KR = 0x0000AAAAu;
}

void PlatformService::InitializeStatusIndicator()
{
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE15_Msk) |
                 (1u << GPIO_MODER_MODE15_Pos);
}

bool PlatformService::InitializeGateDriver()
{
  return gate_driver_ != nullptr && gate_driver_->Init(gate_driver_config_);
}

bool PlatformService::InitializeCurrentSense()
{
  if (current_adc_ == nullptr || !current_adc_->Init())
  {
    return false;
  }
  return vt_sense_ != nullptr && vt_sense_->Init();
}

bool PlatformService::CalibrateCurrentOffset()
{
  return current_adc_ != nullptr && current_adc_->CalibrateOffset();
}

bool PlatformService::InitializePwm()
{
  return phase_pwm_ != nullptr && phase_pwm_->Init();
}

bool PlatformService::StartSynchronizedCurrentSampling()
{
  if (current_adc_ == nullptr || phase_pwm_ == nullptr ||
      !current_adc_->StartPwmSync())
  {
    return false;
  }
  if (vt_sense_ == nullptr || !vt_sense_->StartPwmSync())
  {
    return false;
  }
  phase_pwm_->EnableAdcTrigger();
  return true;
}

bool PlatformService::DriverFaulted()
{
  if (gate_driver_ == nullptr)
  {
    return true;
  }
  const auto status = gate_driver_->ReadStatus();
  return status.fault || status.fault_line;
}

bool PlatformService::TryRecoverDriverFault()
{
  if (gate_driver_ == nullptr)
  {
    return false;
  }
  gate_driver_->Enable();
  DelayNops(200000);
  const auto status = gate_driver_->ReadStatus();
  if (!status.fault && !status.fault_line)
  {
    return true;
  }
  gate_driver_->Disable();
  return false;
}

void PlatformService::DisablePowerStage()
{
  if (gate_driver_ != nullptr)
  {
    gate_driver_->Disable();
  }
}

void PlatformService::SetStatusIndicator(bool on)
{
  GPIOB->BSRR = on ? 0x80000000u : 0x00008000u;
}

void PlatformService::IndicateDriverFault()
{
  SetStatusIndicator(true);
  DelayNops(50000);
  SetStatusIndicator(false);
  DelayNops(50000);
}

uint32_t PlatformService::MonotonicUs()
{
  if (timer_ == nullptr)
  {
    return monotonic_us_;
  }
  const auto current = timer_->read_us();
  if (!timer_started_)
  {
    last_timer_tick_ = current;
    timer_started_ = true;
    return monotonic_us_;
  }
  monotonic_us_ += static_cast<uint32_t>(
      hal::MillisecondTimer::subtract_us(current, last_timer_tick_));
  last_timer_tick_ = current;
  return monotonic_us_;
}

[[noreturn]] void PlatformService::EnterBootloader()
{
  DisablePowerStage();
  for (int i = 0; i < 3; ++i)
  {
    SetStatusIndicator(true);
    DelayNops(200000);
    SetStatusIndicator(false);
    DelayNops(200000);
  }
  ::EnterBootloader();
  while (true)
  {
  }
}

void PlatformService::DelayNops(uint32_t count)
{
  for (volatile uint32_t i = 0; i < count; ++i)
  {
    __NOP();
  }
}

}  // namespace app::board
