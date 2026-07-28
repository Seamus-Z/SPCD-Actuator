// Application state machine: gate driver bring-up, CAN boot, telemetry.
#pragma once

#include "HAL/fdcan.h"
#include "HAL/millisecond_timer.h"
#include "device/drv8353s.h"
#include "telemetry/diagnostic_server.h"
#include "telemetry/status_registry.h"

namespace app
{

enum class State
{
  INIT,
  RUN,
  DRIVER_FAULT,
  ENTER_BOOTLOADER,
};

class Application
{
 public:
  static constexpr const char* kTelemetryChannel = "status";

  Application();

  [[noreturn]] void Run();

  State state() const { return state_; }

 private:
  bool Init();
  void RunOnce();
  void DriverFault();
  void EnterBootloaderMode();
  void PollCan();
  void RegisterTelemetry();

  static size_t TelemetryExport(void* context, char* out, size_t out_capacity);
  size_t FormatTelemetry(char* out, size_t out_capacity) const;

  State state_ = State::INIT;
  // timer_ must outlive gate_driver_ (shared TIM_MST busy-wait).
  hal::MillisecondTimer timer_;
  hal::FDCan can_;
  device::Drv8353s gate_driver_;

  telemetry::StatusRegistry registry_;
  telemetry::DiagnosticServer diagnostic_;
};

}  // namespace app
