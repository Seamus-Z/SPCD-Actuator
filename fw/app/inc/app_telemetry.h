// Registers APP debug/telemetry channels onto StatusRegistry.
#pragma once

#include <cstddef>

#include "HAL/phase_current_adc.h"
#include "HAL/phase_pwm.h"
#include "app_state.h"
#include "device/drv8353s.h"
#include "pool/pool.h"
#include "telemetry/status_registry.h"

namespace app
{

class Application;

class AppTelemetry
{
 public:
  static constexpr const char* kStatusChannel = "status";
  static constexpr const char* kMemChannel = "mem";

  AppTelemetry(telemetry::StatusRegistry* registry,
               const State* state,
               device::Drv8353s* gate_driver,
               hal::PhaseCurrentAdc* current_adc,
               hal::PhasePwm* phase_pwm,
               const Application* application,
               ::pool::Pool* pool);

  void Register();

 private:
  static size_t StatusExport(void* context, char* out, size_t out_capacity);
  static size_t MemExport(void* context, char* out, size_t out_capacity);
  static size_t CurExport(void* context, char* out, size_t out_capacity);
  static size_t PwmExport(void* context, char* out, size_t out_capacity);

  size_t FormatStatus(char* out, size_t out_capacity) const;
  size_t FormatMem(char* out, size_t out_capacity) const;
  size_t FormatCur(char* out, size_t out_capacity) const;
  size_t FormatPwm(char* out, size_t out_capacity) const;

  telemetry::StatusRegistry* registry_ = nullptr;
  const State* state_ = nullptr;
  device::Drv8353s* gate_driver_ = nullptr;
  hal::PhaseCurrentAdc* current_adc_ = nullptr;
  hal::PhasePwm* phase_pwm_ = nullptr;
  const Application* application_ = nullptr;
  ::pool::Pool* pool_ = nullptr;
};

}  // namespace app
