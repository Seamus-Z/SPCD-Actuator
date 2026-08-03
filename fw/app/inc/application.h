// Application: system bring-up, main loop, PWM-rate control, telemetry.
// Host command parsing lives in binary_commands.h.
#pragma once

#include "HAL/fdcan.h"
#include "HAL/millisecond_timer.h"
#include "HAL/phase_current_adc.h"
#include "HAL/phase_pwm.h"
#include "app_state.h"
#include "binary_commands.h"
#include "device/drv8353s.h"
#include "device/ma600.h"
#include "foc_ctrl/current_loop.h"
#include "foc_ctrl/voltage_foc.h"
#include "pool/pool.h"
#include "snapshot_capture.h"
#include "telemetry/binary_link.h"
#include "telemetry/xt_can.h"

namespace app
{

class Application
{
 public:
  explicit Application(::pool::Pool* pool);

  [[noreturn]] void Run();

  State state() const { return state_; }
  ::pool::Pool* pool() const { return pool_; }
  bool pwm_output_on() const { return pwm_output_on_; }
  const foc_ctrl::VoltageFoc* voltage_foc() const { return voltage_foc_.get(); }
  const foc_ctrl::CurrentLoop* current_loop() const
  {
    return current_loop_.get();
  }
  bool dq_valid() const { return dq_valid_; }
  float id_A() const { return id_A_; }
  float iq_A() const { return iq_A_; }
  const hal::PhaseCurrentAdc::Sample& last_current() const
  {
    return last_current_;
  }

 private:
  friend class BinaryCommands;

  bool Init();
  void RunOnce();
  void DriverFault();
  void EnterBootloaderMode();
  void PollCan();
  void MaybeSendTelemetry();
  void MaybeSendSnapshot();
  void StopOutput();
  void StartControlIsr();
  void ControlIsrStep();
  void ObserveDqFromSample(float theta_rad,
                           const hal::PhaseCurrentAdc::Sample& s);

  static void ControlIsrThunk(void* context);

  telemetry::xt_can::Telemetry BuildTelemetry() const;
  telemetry::xt_can::EncTelem BuildEncTelem() const;
  void SampleEncoder();

  State state_ = State::INIT;
  ::pool::Pool* pool_ = nullptr;
  bool pwm_output_on_ = false;
  bool dq_valid_ = false;
  bool encoder_ok_ = false;
  uint8_t mode_ = telemetry::xt_can::kModeStop;
  float id_A_ = 0.0f;
  float iq_A_ = 0.0f;
  float enc_theta_mech_rad_ = 0.0f;
  float enc_theta_elec_rad_ = 0.0f;
  hal::PhaseCurrentAdc::Sample last_current_{};
  hal::MillisecondTimer::TimerType telem_last_us_ = 0;
  SnapshotCapture snapshot_{};
  uint8_t snap_seq_ = 0;
  bool snap_meta_sent_ = false;

  ::pool::PoolPtr<hal::MillisecondTimer> timer_;
  ::pool::PoolPtr<hal::FDCan> can_;
  ::pool::PoolPtr<device::Drv8353s> gate_driver_;
  ::pool::PoolPtr<hal::PhaseCurrentAdc> current_adc_;
  ::pool::PoolPtr<hal::PhasePwm> phase_pwm_;
  ::pool::PoolPtr<foc_ctrl::VoltageFoc> voltage_foc_;
  ::pool::PoolPtr<foc_ctrl::CurrentLoop> current_loop_;
  ::pool::PoolPtr<device::Ma600> ma600_;
  ::pool::PoolPtr<telemetry::BinaryLink> binary_link_;
  BinaryCommands commands_;
};

}  // namespace app
