// Phase current ADC (moteus-x1 / family 3).
// Offset cal uses software trigger (HiZ). Runtime uses TIM5 CC4→DMA→LPTIM1.
#pragma once

#include <cstdint>

#include "HAL/millisecond_timer.h"
#include "PinNames.h"

namespace hal
{

class PhaseCurrentAdc
{
 public:
  enum class Error : uint8_t
  {
    None = 0,
    BadConfig = 1,
    EnableFail = 2,
    SampleTimeout = 3,
    OffsetOutOfRange = 4,
    SyncFail = 5,  // LPTIM / DMA / HW-trigger arm failed
  };

  struct Options
  {
    PinName current1 = PA_3;       // ADC1_IN4
    PinName current2 = PA_6;       // ADC2_IN3
    PinName current3 = PB_1;       // ADC3_IN1
    float sense_ohm = 0.0005f;
    float csa_gain = 20.0f;
    uint16_t sample_time_code = 1;
  };

  struct Sample
  {
    uint16_t raw1 = 0;
    uint16_t raw2 = 0;
    uint16_t raw3 = 0;
    float i1_A = 0.0f;
    float i2_A = 0.0f;
    float i3_A = 0.0f;
    bool ok = false;
  };

  static constexpr const char* kTelemetryChannel = "cur";
  static constexpr uint16_t kCalibrateCount = 256;
  static constexpr uint16_t kExpectedMid = 2048;
  static constexpr uint16_t kMidTolerance = 200;
  // DMAMUX request ID for TIM5_CH4 (RM0440 Table 91/93).
  static constexpr uint32_t kTim5Ch4Dmamux = 75;

  PhaseCurrentAdc(MillisecondTimer* timer, const Options& options);

  // Enable ADC1/2/3 in software-trigger mode (for offset cal).
  bool Init();

  // Average samples with software trigger (motor HiZ / duty=0).
  bool CalibrateOffset();

  // Switch to TIM5 CC4→DMA2_Ch5→LPTIM1_OUT hardware trigger.
  // Call after PhasePwm::Init(); then PhasePwm::EnableAdcTrigger().
  bool StartPwmSync();

  Sample Read();

  // Non-blocking: grab latest DR (for PWM-rate ISR after peak sample).
  // Does not wait for EOS; use only when sync_on_ and OVRMOD overwrite is on.
  Sample ReadLatest();

  bool init_ok() const { return init_ok_; }
  bool offset_ok() const { return offset_ok_; }
  bool sync_on() const { return sync_on_; }
  uint32_t seq() const { return seq_; }
  Error last_error() const { return last_error_; }
  uint16_t offset1() const { return offset1_; }
  uint16_t offset2() const { return offset2_; }
  uint16_t offset3() const { return offset3_; }
  float scale_A_per_lsb() const { return scale_; }

 private:
  void SetError(Error err);
  void ConfigureGpioAnalog();
  void ConfigureAdcCommon();
  bool EnableOneAdcSoftware(ADC_TypeDef* adc);
  void DisableOneAdc(ADC_TypeDef* adc);
  void ConfigureChannels();
  bool ConfigureLptim1();
  bool ConfigureDmaLptimTrigger();
  bool ArmHardwareTrigger(ADC_TypeDef* adc);
  static bool WaitFlag(ADC_TypeDef* adc, uint32_t flag);
  bool ReadRawSoftware(uint16_t* raw1, uint16_t* raw2, uint16_t* raw3);
  bool ReadRawSynced(uint16_t* raw1, uint16_t* raw2, uint16_t* raw3);
  bool ReadRawFamily3(uint16_t* raw1, uint16_t* raw2, uint16_t* raw3);

  MillisecondTimer* timer_ = nullptr;
  Options options_;
  float scale_ = 0.0f;
  uint16_t offset1_ = kExpectedMid;
  uint16_t offset2_ = kExpectedMid;
  uint16_t offset3_ = kExpectedMid;
  bool init_ok_ = false;
  bool offset_ok_ = false;
  bool sync_on_ = false;
  uint32_t seq_ = 0;
  Error last_error_ = Error::None;

  // Must live in SRAM (DMA cannot fetch from CCM/flash).
  uint32_t lptim1_sngstrt_value_ = 0;
};

}  // namespace hal
