// moteus-x1 / family 3 voltage and temperature ADC (ADC4/ADC5, LPTIM1).
#pragma once

#include <cstdint>

#include "HAL/millisecond_timer.h"
#include "PinNames.h"
#include "stm32g4xx.h"

namespace hal
{

class VtSenseAdc
{
 public:
  struct Options
  {
    PinName vsense = PA_9;        // ADC5_IN2
    PinName tsense = PB_12_ALT0;  // ADC4_IN3 FET NTC
    float vsense_adc_scale = 0.017947f;
    float fet_r25_ohm = 47000.0f;
    uint16_t sample_time_code = 7;  // 640.5 cycles; NTC is slow
  };

  struct Sample
  {
    uint16_t bus_raw = 0;
    uint16_t fet_raw = 0;
    float bus_V = 0.0f;
    float fet_temp_C = 0.0f;
    bool bus_ok = false;
    bool fet_ok = false;
  };

  VtSenseAdc(MillisecondTimer* timer, const Options& options);

  bool Init();
  // Arm ADC4/5 on the existing LPTIM1_OUT trigger. Call after
  // PhaseCurrentAdc::StartPwmSync().
  bool StartPwmSync();
  Sample ReadLatest();

  bool init_ok() const { return init_ok_; }
  bool sync_on() const { return sync_on_; }

 private:
  void ConfigureGpioAnalog();
  bool EnableOneAdcSoftware(ADC_TypeDef* adc);
  bool ArmHardwareTrigger(ADC_TypeDef* adc);
  void DisableOneAdc(ADC_TypeDef* adc);

  MillisecondTimer* timer_ = nullptr;
  Options options_;
  Sample latest_{};
  bool init_ok_ = false;
  bool sync_on_ = false;
};

}  // namespace hal
