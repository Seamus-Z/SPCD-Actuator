// Application state machine: gate driver bring-up, CAN boot, telemetry attach.
#pragma once

#include "HAL/fdcan.h"
#include "HAL/millisecond_timer.h"
#include "HAL/phase_current_adc.h"
#include "HAL/phase_pwm.h"
#include "app_state.h"
#include "app_telemetry.h"
#include "control/current_loop.h"
#include "control/voltage_foc.h"
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
  const control::VoltageFoc* voltage_foc() const { return voltage_foc_.get(); }
  const control::CurrentLoop* current_loop() const
  {
    return current_loop_.get();
  }
  // Last Park / phase snapshot (updated from PWM-rate ISR).
  bool dq_valid() const { return dq_valid_; }
  float id_A() const { return id_A_; }
  float iq_A() const { return iq_A_; }
  const hal::PhaseCurrentAdc::Sample& last_current() const
  {
    return last_current_;
  }

 private:
  bool Init();
  void RunOnce();
  void DriverFault();
  void EnterBootloaderMode();
  void PollCan();
  void StopOutput();
  void StartControlIsr();
  void ControlIsrStep();
  void ObserveDqFromSample(float theta_rad,
                           const hal::PhaseCurrentAdc::Sample& s);

  static bool DiagCommandThunk(void* context, std::string_view verb,
                               mjlib::base::Tokenizer& tokenizer,
                               mjlib::base::BufferWriteStream& writer);
  static void ControlIsrThunk(void* context);
  bool HandleDiagCommand(std::string_view verb,
                         mjlib::base::Tokenizer& tokenizer,
                         mjlib::base::BufferWriteStream& writer);
  bool HandleRawCommand(mjlib::base::Tokenizer& tokenizer,
                        mjlib::base::BufferWriteStream& writer);
  bool HandleVfocCommand(mjlib::base::Tokenizer& tokenizer,
                         mjlib::base::BufferWriteStream& writer);
  bool HandleDqCommand(mjlib::base::Tokenizer& tokenizer,
                       mjlib::base::BufferWriteStream& writer);

  State state_ = State::INIT;
  ::pool::Pool* pool_ = nullptr;
  bool pwm_output_on_ = false;
  bool dq_valid_ = false;
  float id_A_ = 0.0f;
  float iq_A_ = 0.0f;
  hal::PhaseCurrentAdc::Sample last_current_{};

  // Construction order = lifetime / dependency order.
  ::pool::PoolPtr<hal::MillisecondTimer> timer_;
  ::pool::PoolPtr<hal::FDCan> can_;
  ::pool::PoolPtr<device::Drv8353s> gate_driver_;
  ::pool::PoolPtr<hal::PhaseCurrentAdc> current_adc_;
  ::pool::PoolPtr<hal::PhasePwm> phase_pwm_;
  ::pool::PoolPtr<control::VoltageFoc> voltage_foc_;
  ::pool::PoolPtr<control::CurrentLoop> current_loop_;
  ::pool::PoolPtr<telemetry::StatusRegistry> registry_;
  ::pool::PoolPtr<telemetry::DiagnosticServer> diagnostic_;
  // Last: observes modules above; does not own them.
  AppTelemetry telemetry_;
};

}  // namespace app
