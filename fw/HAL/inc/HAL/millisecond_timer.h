// Busy-wait timer based on TIM_MST (1 us tick), aligned with moteus.
// Bare-metal: we must Start the timer ourselves (no mbed HAL_InitTick).
#pragma once

#include "stm32g4xx_hal.h"
#include "ports/platform_ports.h"

// moteus-x1 uses TIM5 for motor PWM; relocate the us ticker to TIM15
// (same override as moteus WORKSPACE MBED_US_TIMER_*).
#ifndef MBED_US_TIMER_TIM
#define MBED_US_TIMER_TIM TIM15
#define MBED_US_TIMER_USCORE_TIM _TIM15
#define MBED_US_TIMER_TIM_USCORE TIM15_
#define TIM_MST_IRQ TIM1_BRK_TIM15_IRQn
#define TIM_MST_BIT_WIDTH 16
#endif

#include "us_ticker_data.h"

namespace hal
{

class MillisecondTimer final : public ports::IDelay
{
 public:
  MillisecondTimer()
  {
    TIM_MST_RCC;

    // G4 TIM_MST is already on the 1x timer clock path used by moteus.
    // SystemCoreClock is the real SYSCLK (170 MHz after SetupSystemClock).
    constexpr int kExtraPrescaler = 1;

    handle_.Instance = TIM_MST;
#if TIM_MST_BIT_WIDTH == 32
    handle_.Init.Period = 0xffffffff;
#elif TIM_MST_BIT_WIDTH == 16
    handle_.Init.Period = 0xffff;
#else
#error "unsupported TIM_MST_BIT_WIDTH"
#endif
    handle_.Init.Prescaler =
        static_cast<uint32_t>(SystemCoreClock / kExtraPrescaler / 1000000U) - 1;
    handle_.Init.ClockDivision = 0;
    handle_.Init.CounterMode = TIM_COUNTERMODE_UP;
    handle_.Init.RepetitionCounter = 0;

    HAL_TIM_Base_Init(&handle_);
    // moteus relies on mbed HAL_InitTick() to have already started TIM_MST.
    // Our bare-metal AppReset does not, so start the counter here.
    HAL_TIM_Base_Start(&handle_);
  }

#if TIM_MST_BIT_WIDTH == 32
  using TimerType = uint32_t;
#elif TIM_MST_BIT_WIDTH == 16
  using TimerType = uint16_t;
#else
#error "unsupported TIM_MST_BIT_WIDTH"
#endif

  TimerType read_ms() { return TIM_MST->CNT / 1000; }
  TimerType read_us() { return TIM_MST->CNT; }

  static TimerType subtract_us(TimerType a, TimerType b)
  {
    return static_cast<TimerType>(a - b);
  }

  void wait_ms(uint32_t delay_ms) { wait_us(delay_ms * 1000); }

  void WaitUs(uint32_t delay_us) override { wait_us(delay_us); }

  void wait_us(uint32_t delay_us)
  {
    while (delay_us > 50000)
    {
      WaitUsHelper(50000);
      delay_us -= 50000;
    }
    WaitUsHelper(delay_us);
  }

 private:
  void WaitUsHelper(uint32_t delay_us)
  {
    TimerType current = TIM_MST->CNT;
    TimerType elapsed = 0;
    while (true)
    {
      const TimerType next = TIM_MST->CNT;
      elapsed += static_cast<TimerType>(next - current);
      // +1 because we may start mid-microsecond.
      if (elapsed >= (delay_us + 1))
      {
        return;
      }
      current = next;
    }
  }

  TIM_HandleTypeDef handle_ = {};
};

}  // namespace hal
