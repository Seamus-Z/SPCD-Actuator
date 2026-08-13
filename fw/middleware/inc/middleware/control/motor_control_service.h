// Owns every real-time motor-control and calibration session.
#pragma once

#include <cstdint>

#include "HAL/millisecond_timer.h"
#include "HAL/phase_current_adc.h"
#include "HAL/phase_pwm.h"
#include "HAL/vt_sense_adc.h"
#include "math/protection/safety_monitor.h"
#include "device/drv8353s.h"
#include "math/foc/controller.h"
#include "math/foc/modulator.h"
#include "math/servo_mode/mit_mode.h"
#include "math/servo_mode/servo_mode.h"
#include "middleware/calibration/calibration_manager.h"
#include "middleware/encoder/encoder_service.h"
#include "middleware/snapshot/snapshot_service.h"

namespace middleware::control
{

enum class Result : uint8_t
{
  Ok = 0,
  Failed,
  NotReady,
  InvalidArgument,
};

enum class Mode : uint8_t
{
  Stopped = 0,
  Calibration,
  Servo,
  Current,
  Mit,
};

enum class CalibrationKind : uint8_t
{
  Abort = 0,
  EncoderPhase,
  EncoderLock,
  Bemf,
  Resistance,
  Inductance,
  Cogging,
};

struct CalibrationCommand
{
  CalibrationKind kind = CalibrationKind::Abort;
  float pole_pairs = 1.0f;
  float encoder_current_A = 1.0f;
  float encoder_electrical_speed_rad_s = 40.0f;
  float resistance_max_current_A = 1.5f;
  uint8_t resistance_points = 0;
  float inductance_step_voltage_V = 0.0f;
  uint8_t inductance_trials = 0;
  float bemf_max_speed_rad_s = 60.0f;
  uint8_t bemf_points = 0;
  float cogging_velocity_rad_s = 5.0f;
  float cogging_record_revs = 0.0f;
};

class ICalibrationResultSink
{
 public:
  virtual ~ICalibrationResultSink() = default;
  virtual void AcceptResistanceCalibration() = 0;
  virtual void AcceptInductanceCalibration() = 0;
  virtual void AcceptBemfCalibration() = 0;
};

class MotorControlService
{
 public:
  struct Dependencies
  {
    hal::MillisecondTimer* timer = nullptr;
    device::Drv8353s* gate_driver = nullptr;
    hal::PhaseCurrentAdc* current_adc = nullptr;
    hal::VtSenseAdc* vt_sense = nullptr;
    hal::PhasePwm* phase_pwm = nullptr;
    math::foc::DqModulator* dq_modulator = nullptr;
    math::foc::FocController* foc = nullptr;
    math::servo_mode::ServoMode* servo = nullptr;
    math::servo_mode::MitMode* mit = nullptr;
    middleware::encoder::EncoderService* encoder = nullptr;
    middleware::calibration::CalibrationManager* calibration = nullptr;
    middleware::snapshot::SnapshotService* snapshot = nullptr;
  };

  explicit MotorControlService(const Dependencies& dependencies);

  Result StartServo(const math::servo_mode::ServoMode::Command& command);
  Result StartCurrent(float id_A, float iq_A);
  Result StartMit(const math::servo_mode::MitMode::Command& command);
  Result StartCalibration(const CalibrationCommand& command);
  void Stop();

  // Sample bus voltage / FET NTC outside the control ISR (idle telemetry).
  void SampleSlowTelemetry();

  void SetCalibrationResultSink(ICalibrationResultSink* sink)
  {
    calibration_result_sink_ = sink;
  }
  void SetOvercurrentTrip(float trip_A)
  {
    auto config = safety_.config();
    config.overcurrent_A = trip_A;
    safety_.set_config(config);
  }
  void SetProtectionConfig(const math::protection::SafetyMonitor::Config& config)
  {
    safety_.set_config(config);
  }
  math::protection::Trip last_protection_trip() const { return safety_.last_trip(); }
  bool protection_tripped() const
  {
    return safety_.last_trip() != math::protection::Trip::None;
  }
  float measured_bus_V() const { return measured_bus_V_; }
  float fet_temp_C() const { return fet_temp_C_; }
  bool fet_temp_ok() const { return fet_temp_ok_; }


  bool output_enabled() const { return output_enabled_; }
  bool dq_valid() const { return dq_valid_; }
  Mode mode() const { return mode_; }
  bool active() const { return mode_ != Mode::Stopped; }
  float id_A() const { return id_A_; }
  float iq_A() const { return iq_A_; }
  const hal::PhaseCurrentAdc::Sample& last_current() const
  {
    return last_current_;
  }
  const math::foc::DqModulator* dq_modulator() const { return dq_modulator_; }
  const math::foc::FocController* foc() const { return foc_; }
  const math::servo_mode::ServoMode* servo() const { return servo_; }
  const math::servo_mode::MitMode* mit() const { return mit_; }
  bool isr_enabled() const
  {
    return phase_pwm_ != nullptr && phase_pwm_->control_isr_on();
  }
  const hal::PhasePwm* phase_pwm() const { return phase_pwm_; }

 private:
  void StartIsr();
  static void IsrThunk(void* context);
  void StepIsr();
  void ObserveDq(float theta_rad, const hal::PhaseCurrentAdc::Sample& sample);
  void StopFinishedSession();
  Result PrimeFoc(float theta_rad, float id_A, float iq_A,
                  const float* theta_override);

  bool ApplyProtection(const hal::PhaseCurrentAdc::Sample& sample, float dt_s);

  hal::MillisecondTimer* timer_ = nullptr;
  device::Drv8353s* gate_driver_ = nullptr;
  hal::PhaseCurrentAdc* current_adc_ = nullptr;
  hal::VtSenseAdc* vt_sense_ = nullptr;
  hal::PhasePwm* phase_pwm_ = nullptr;
  math::foc::DqModulator* dq_modulator_ = nullptr;
  math::foc::FocController* foc_ = nullptr;
  math::servo_mode::ServoMode* servo_ = nullptr;
  math::servo_mode::MitMode* mit_ = nullptr;
  middleware::encoder::EncoderService* encoder_ = nullptr;
  middleware::calibration::CalibrationManager* calibration_ = nullptr;
  middleware::snapshot::SnapshotService* snapshot_ = nullptr;
  ICalibrationResultSink* calibration_result_sink_ = nullptr;

  Mode mode_ = Mode::Stopped;
  bool output_enabled_ = false;
  bool dq_valid_ = false;
  float id_A_ = 0.0f;
  float iq_A_ = 0.0f;
  hal::PhaseCurrentAdc::Sample last_current_{};
  hal::MillisecondTimer::TimerType last_control_us_ = 0;
  math::protection::SafetyMonitor safety_;
  float measured_bus_V_ = 0.0f;
  float fet_temp_C_ = 0.0f;
  bool fet_temp_ok_ = false;
};

}  // namespace middleware::control
