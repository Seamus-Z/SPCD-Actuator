// Application state machine: gate driver bring-up, CAN boot, telemetry attach.
#pragma once

#include "HAL/fdcan.h"
#include "HAL/millisecond_timer.h"
#include "app_state.h"
#include "app_telemetry.h"
#include "device/drv8353s.h"
#include "pool/pool.h"
#include "telemetry/diagnostic_server.h"
#include "telemetry/status_registry.h"

namespace app
{

class Application
{
 public:
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

  State state_ = State::INIT;
  ::pool::Pool* pool_ = nullptr;

  // Construction order = lifetime / dependency order.
  ::pool::PoolPtr<hal::MillisecondTimer> timer_;
  ::pool::PoolPtr<hal::FDCan> can_;
  ::pool::PoolPtr<device::Drv8353s> gate_driver_;
  ::pool::PoolPtr<telemetry::StatusRegistry> registry_;
  ::pool::PoolPtr<telemetry::DiagnosticServer> diagnostic_;
  // Last: observes modules above; does not own them.
  AppTelemetry telemetry_;
};

}  // namespace app
