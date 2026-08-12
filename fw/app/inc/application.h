// Application: system bring-up, main loop, PWM-rate control, telemetry.
// Host command parsing is an Application adapter implemented in binary_commands.cc.
#pragma once

#include <cstddef>
#include <cstdint>

#include "HAL/fdcan.h"
#include "HAL/millisecond_timer.h"
#include "HAL/phase_current_adc.h"
#include "HAL/phase_pwm.h"
#include "app_state.h"
#include "device/drv8353s.h"
#include "math/foc/controller.h"
#include "math/servo_mode/servo_mode.h"
#include "math/servo_mode/mit_mode.h"
#include "math/foc/modulator.h"
#include "middleware/encoder/encoder_service.h"
#include "middleware/calibration/calibration_manager.h"
#include "nvs/runtime_config_store.h"
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

  // Position/velocity servo (moteus kPosition).
  uint8_t StartServo(const math::servo_mode::ServoMode::Command& command);
  // Direct Id/Iq current mode (moteus kCurrent).
  uint8_t StartCurrent(float id_A, float iq_A);
  // MIT impedance: T=Kp(Pcmd-Pfdb)+Kd(Vcmd-Vfdb)+Tff (multi-turn continuous).
  uint8_t StartMit(const math::servo_mode::MitMode::Command& command);

  State state() const { return state_; }
  ::pool::Pool* pool() const { return pool_; }
  bool pwm_output_on() const { return pwm_output_on_; }
  const math::foc::DqModulator* dq_modulator() const { return dq_modulator_.get(); }
  const math::foc::FocController* foc() const
  {
    return foc_.get();
  }
  bool dq_valid() const { return dq_valid_; }
  float id_A() const { return id_A_; }
  float iq_A() const { return iq_A_; }
  const hal::PhaseCurrentAdc::Sample& last_current() const
  {
    return last_current_;
  }

 private:
  static uint8_t CommandThunk(void* context, uint8_t cmd, uint8_t seq,
                              const uint8_t* payload, size_t payload_len);
  uint8_t HandleCommand(uint8_t cmd, uint8_t seq, const uint8_t* payload,
                        size_t payload_len);
  uint8_t HandleStop();
  uint8_t HandleQuery();
  uint8_t HandleServo(const uint8_t* payload, size_t payload_len);
  uint8_t HandleCal(const uint8_t* payload, size_t payload_len);
  uint8_t HandleInfo(uint8_t seq);
  uint8_t HandleSnap(uint8_t seq, const uint8_t* payload, size_t payload_len);
  uint8_t HandleEncComp(const uint8_t* payload, size_t payload_len);
  uint8_t HandleConf(uint8_t seq, const uint8_t* payload, size_t payload_len);

  void FillDefaultRuntimeConfig();
  void ApplyRuntimeConfig();
  // When runtime Flash is empty, copy electricals from cal Flash into Motor.
  void SeedMotorElectricalFromCalFlash();
  // Cal result → Motor table (RAM). Persist with Config Save.
  void SyncMotorConfFromResistanceCal();
  void SyncMotorConfFromInductanceCal();
  void SyncMotorConfFromBemfCal();
  bool SendConfGroup(uint8_t seq, uint8_t op, uint8_t group);
  float PolePairs() const;

  bool Init();
  void RunOnce();
  void DriverFault();
  void EnterBootloaderMode();
  void PollCan();
  void MaybeSendTelemetry();
  void MaybeCommandTimeout();
  void ReplyCtrl(uint8_t cmd, uint8_t seq, uint8_t status);
  void MaybeSendSnapshot();
  void StopOutput();
  void StartControlIsr();
  void ControlIsrStep();
  void ObserveDqFromSample(float theta_rad,
                           const hal::PhaseCurrentAdc::Sample& s);

  static void ControlIsrThunk(void* context);

  telemetry::xt_can::Telemetry BuildTelemetry() const;
  telemetry::xt_can::EncTelem BuildEncTelem() const;
  telemetry::xt_can::CalTelem BuildCalTelem() const;
  telemetry::xt_can::CtrlReply BuildCtrlReply(uint8_t cmd, uint8_t seq,
                                             uint8_t status) const;

  State state_ = State::INIT;
  ::pool::Pool* pool_ = nullptr;
  bool pwm_output_on_ = false;
  bool dq_valid_ = false;
  bool encoder_ok_ = false;
  uint8_t mode_ = telemetry::xt_can::kModeStop;
  float id_A_ = 0.0f;
  float iq_A_ = 0.0f;
  hal::PhaseCurrentAdc::Sample last_current_{};
  hal::MillisecondTimer::TimerType telem_last_us_ = 0;
  hal::MillisecondTimer::TimerType last_rx_us_ = 0;
  // Last ControlIsrStep timestamp; 0 = use PWM nominal dt next time.
  hal::MillisecondTimer::TimerType last_control_us_ = 0;
  uint8_t enc_telem_div_ = 0;
  SnapshotCapture snapshot_{};
  uint8_t snap_seq_ = 0;
  bool snap_meta_sent_ = false;

  ::pool::PoolPtr<hal::MillisecondTimer> timer_;
  ::pool::PoolPtr<hal::FDCan> can_;
  ::pool::PoolPtr<device::Drv8353s> gate_driver_;
  ::pool::PoolPtr<hal::PhaseCurrentAdc> current_adc_;
  ::pool::PoolPtr<hal::PhasePwm> phase_pwm_;
  ::pool::PoolPtr<math::foc::DqModulator> dq_modulator_;
  ::pool::PoolPtr<math::foc::FocController> foc_;
  ::pool::PoolPtr<math::servo_mode::ServoMode> servo_mode_;
  ::pool::PoolPtr<math::servo_mode::MitMode> mit_mode_;
  middleware::encoder::EncoderService encoder_;
  middleware::calibration::CalibrationManager calibration_;
  nvs::RuntimeConfig runtime_config_{};
  bool runtime_config_flash_valid_ = false;
  ::pool::PoolPtr<telemetry::BinaryLink> binary_link_;
};

}  // namespace app
