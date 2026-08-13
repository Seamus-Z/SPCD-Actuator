// STM32G4 implementation of the platform operations used by Application.
#pragma once

#include "HAL/millisecond_timer.h"
#include "HAL/phase_current_adc.h"
#include "HAL/phase_pwm.h"
#include "HAL/vt_sense_adc.h"
#include "application_ports.h"
#include "device/drv8353s.h"

namespace app::board
{

class PlatformService final : public IPlatformService
{
 public:
  struct Dependencies
  {
    hal::MillisecondTimer* timer = nullptr;
    device::Drv8353s* gate_driver = nullptr;
    hal::PhaseCurrentAdc* current_adc = nullptr;
    hal::VtSenseAdc* vt_sense = nullptr;
    hal::PhasePwm* phase_pwm = nullptr;
    device::Drv8353s::Config gate_driver_config{};
  };

  explicit PlatformService(const Dependencies& dependencies);

  void StartWatchdog() override;
  void FeedWatchdog() override;

  void InitializeStatusIndicator() override;
  bool InitializeGateDriver() override;
  bool InitializeCurrentSense() override;
  bool CalibrateCurrentOffset() override;
  bool InitializePwm() override;
  bool StartSynchronizedCurrentSampling() override;

  bool DriverFaulted() override;
  bool TryRecoverDriverFault() override;
  void DisablePowerStage() override;
  void SetStatusIndicator(bool on) override;
  void IndicateDriverFault() override;
  uint32_t MonotonicUs() override;

  [[noreturn]] void EnterBootloader() override;

 private:
  static void DelayNops(uint32_t count);

  hal::MillisecondTimer* timer_ = nullptr;
  device::Drv8353s* gate_driver_ = nullptr;
  hal::PhaseCurrentAdc* current_adc_ = nullptr;
  hal::VtSenseAdc* vt_sense_ = nullptr;
  hal::PhasePwm* phase_pwm_ = nullptr;
  device::Drv8353s::Config gate_driver_config_{};
  hal::MillisecondTimer::TimerType last_timer_tick_ = 0;
  uint32_t monotonic_us_ = 0;
  bool timer_started_ = false;
};

}  // namespace app::board
