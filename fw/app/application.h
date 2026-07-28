// Application state machine: gate driver bring-up, CAN boot, telemetry.
#pragma once

#include "HAL/fdcan.h"
#include "HAL/millisecond_timer.h"
#include "device/drv8353s.h"
#include "pool/pool.h"
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
  static constexpr const char* kMemTelemetryChannel = "mem";

  // |pool| must outlive this object; child modules are allocated from it.
  explicit Application(::pool::Pool* pool);

  [[noreturn]] void Run();

  State state() const { return state_; }
  ::pool::Pool* pool() const { return pool_; }

 private:
  bool Init();
  void RunOnce();
  void DriverFault();
  void EnterBootloaderMode();
  void PollCan();
  void RegisterTelemetry();

  static size_t TelemetryExport(void* context, char* out, size_t out_capacity);
  static size_t MemTelemetryExport(void* context, char* out, size_t out_capacity);
  size_t FormatTelemetry(char* out, size_t out_capacity) const;
  size_t FormatMemTelemetry(char* out, size_t out_capacity) const;

  State state_ = State::INIT;
  ::pool::Pool* pool_ = nullptr;

  // Construction order = lifetime / dependency order.
  ::pool::PoolPtr<hal::MillisecondTimer> timer_;
  ::pool::PoolPtr<hal::FDCan> can_;
  ::pool::PoolPtr<device::Drv8353s> gate_driver_;
  ::pool::PoolPtr<telemetry::StatusRegistry> registry_;
  ::pool::PoolPtr<telemetry::DiagnosticServer> diagnostic_;
};

}  // namespace app
