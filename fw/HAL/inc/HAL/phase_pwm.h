// TIM5 center-aligned 3-phase PWM (moteus-x1: PA0/PA1/PA2).
#pragma once

#include <cstdint>

#include "PinNames.h"

namespace hal
{

// Defined in phase_pwm.cc; vector table entry TIM5_IRQHandler calls this.
void Tim5ControlIrq();

class PhasePwm
{
 public:
  struct Options
  {
    PinName pwm1 = PA_0_ALT0;  // TIM5_CH1
    PinName pwm2 = PA_1_ALT0;  // TIM5_CH2
    PinName pwm3 = PA_2_ALT0;  // TIM5_CH3
    // moteus-x1 / family 3 board default.
    uint16_t rate_hz = 15000;
  };

  static constexpr uint16_t kDutyMax = 1000;  // thousandths of full scale
  // Leave ~5% headroom so peak sampling stays in the low-side window
  // (CSA settling ~1.5 us @ gain 20, 15 kHz half-period ~33 us).
  static constexpr uint16_t kDutyMinMilli = 50;
  static constexpr uint16_t kDutyMaxMilli = 950;
  static constexpr const char* kTelemetryChannel = "pwm";

  explicit PhasePwm(const Options& options);

  bool Init();

  // Duty in thousandths [0, 1000]. 0 stays 0 (HiZ idle); non-zero clamped
  // to [kDutyMinMilli, kDutyMaxMilli] for sampling window.
  void SetDuty(uint16_t a_milli, uint16_t b_milli, uint16_t c_milli);
  void Stop();  // CCR = 0

  // Enable TIM5 CC4 DMA request (peak trigger). Call only after ADC/LPTIM/DMA
  // sync chain is armed.
  void EnableAdcTrigger();
  void DisableAdcTrigger();

  // Control ISR once per PWM period (valley UEV after peak ADC sample).
  // |fn| runs from TIM5_IRQHandler — keep it short, no blocking waits.
  using ControlIsrFn = void (*)(void* context);
  void EnableControlIsr(ControlIsrFn fn, void* context);
  void DisableControlIsr();
  bool control_isr_on() const { return control_isr_on_; }
  float period_s() const;

  bool init_ok() const { return init_ok_; }
  bool adc_trigger_on() const { return adc_trigger_on_; }
  uint16_t rate_hz() const { return options_.rate_hz; }
  uint32_t arr() const { return arr_; }
  uint16_t duty_a() const { return duty_a_; }
  uint16_t duty_b() const { return duty_b_; }
  uint16_t duty_c() const { return duty_c_; }

 private:
  friend void Tim5ControlIrq();

  void ConfigureGpio();
  void ConfigureTimer();
  static uint16_t ClampDuty(uint16_t milli);
  uint16_t DutyToCcr(uint16_t milli) const;

  Options options_;
  uint32_t arr_ = 0;
  uint16_t duty_a_ = 0;
  uint16_t duty_b_ = 0;
  uint16_t duty_c_ = 0;
  bool init_ok_ = false;
  bool adc_trigger_on_ = false;
  bool control_isr_on_ = false;
};

}  // namespace hal
