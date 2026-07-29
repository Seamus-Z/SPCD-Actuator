// Application state machine: gate driver bring-up, CAN boot, telemetry attach.
#pragma once

#include "HAL/fdcan.h"
#include "HAL/millisecond_timer.h"
#include "HAL/phase_current_adc.h"
#include "HAL/phase_pwm.h"
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
  bool pwm_output_on() const { return pwm_output_on_; }

 private:
  bool Init();
  void RunOnce();
  void DriverFault();
  void EnterBootloaderMode();
  void PollCan();
  void StopOutput();

  static bool DiagCommandThunk(void* context, std::string_view verb,
                               mjlib::base::Tokenizer& tokenizer,
                               mjlib::base::BufferWriteStream& writer);
  bool HandleDiagCommand(std::string_view verb,
                         mjlib::base::Tokenizer& tokenizer,
                         mjlib::base::BufferWriteStream& writer);
  bool HandleRawCommand(mjlib::base::Tokenizer& tokenizer,
                        mjlib::base::BufferWriteStream& writer);

  State state_ = State::INIT;
  ::pool::Pool* pool_ = nullptr;
  bool pwm_output_on_ = false;

  // Construction order = lifetime / dependency order.
  ::pool::PoolPtr<hal::MillisecondTimer> timer_;
  ::pool::PoolPtr<hal::FDCan> can_;
  ::pool::PoolPtr<device::Drv8353s> gate_driver_;
  ::pool::PoolPtr<hal::PhaseCurrentAdc> current_adc_;
  ::pool::PoolPtr<hal::PhasePwm> phase_pwm_;
  ::pool::PoolPtr<telemetry::StatusRegistry> registry_;
  ::pool::PoolPtr<telemetry::DiagnosticServer> diagnostic_;
  // Last: observes modules above; does not own them.
  AppTelemetry telemetry_;
};

}  // namespace app
