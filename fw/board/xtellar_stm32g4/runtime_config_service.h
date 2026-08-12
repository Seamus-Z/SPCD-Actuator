// STM32G4 board policy for runtime motor/control configuration.
#pragma once

#include "app/inc/runtime_config_service.h"
#include "math/foc/controller.h"
#include "math/foc/modulator.h"
#include "math/servo_mode/mit_mode.h"
#include "math/servo_mode/servo_mode.h"
#include "middleware/calibration/calibration_manager.h"
#include "middleware/control/motor_control_service.h"
#include "middleware/encoder/encoder_service.h"

namespace app::board
{

class RuntimeConfigService final
    : public app::IRuntimeConfigService,
      public middleware::control::ICalibrationResultSink
{
 public:
  struct Dependencies
  {
    middleware::encoder::EncoderService* encoder = nullptr;
    middleware::calibration::CalibrationManager* calibration = nullptr;
    middleware::control::MotorControlService* motor_control = nullptr;
    math::foc::DqModulator* dq_modulator = nullptr;
    math::foc::FocController* foc = nullptr;
    math::servo_mode::ServoMode* servo = nullptr;
    math::servo_mode::MitMode* mit = nullptr;
  };

  explicit RuntimeConfigService(const Dependencies& dependencies);

  bool Initialize() override;
  const middleware::config::RuntimeConfig& config() const override { return config_; }
  middleware::config::RuntimeConfig& mutable_config() override { return config_; }
  bool flash_valid() const override { return flash_valid_; }
  void Apply() override;
  bool Save() override;
  bool Load() override;
  void RestoreDefaults() override;

  void AcceptResistanceCalibration() override;
  void AcceptInductanceCalibration() override;
  void AcceptBemfCalibration() override;

 private:
  void FillDefaults();
  void SeedMotorElectricalFromCalibrationFlash();
  void ReloadPersistentCalibration();

  middleware::encoder::EncoderService* encoder_ = nullptr;
  middleware::calibration::CalibrationManager* calibration_ = nullptr;
  middleware::control::MotorControlService* motor_control_ = nullptr;
  math::foc::DqModulator* dq_modulator_ = nullptr;
  math::foc::FocController* foc_ = nullptr;
  math::servo_mode::ServoMode* servo_ = nullptr;
  math::servo_mode::MitMode* mit_ = nullptr;
  middleware::config::RuntimeConfig config_{};
  bool flash_valid_ = false;
};

}  // namespace app::board
