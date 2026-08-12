// CAN wire-protocol adapter. Decodes frames and delegates to business services.
#pragma once

#include <cstddef>
#include <cstdint>

#include "HAL/fdcan.h"
#include "application_ports.h"
#include "middleware/calibration/calibration_manager.h"
#include "middleware/communication/telemetry_publisher.h"
#include "middleware/control/motor_control_service.h"
#include "middleware/encoder/encoder_service.h"
#include "middleware/snapshot/snapshot_service.h"
#include "runtime_config_service.h"
#include "middleware/communication/binary_link.h"

namespace middleware::communication
{

class CanCommandAdapter final : public app::ICommandEndpoint
{
 public:
  struct Dependencies
  {
    hal::FDCan* can = nullptr;
    middleware::communication::BinaryLink* link = nullptr;
    control::MotorControlService* motor_control = nullptr;
    encoder::EncoderService* encoder = nullptr;
    calibration::CalibrationManager* calibration = nullptr;
    snapshot::SnapshotService* snapshot = nullptr;
    app::IRuntimeConfigService* runtime_config = nullptr;
    TelemetryPublisher* telemetry = nullptr;
    const char* motor_name = nullptr;
    uint16_t nominal_pwm_hz = 0;
  };

  explicit CanCommandAdapter(const Dependencies& dependencies);

  app::CommandPollResult Poll(
      const app::CommandPollContext& context) override;

 private:
  static uint8_t CommandThunk(void* context, uint8_t command,
                              uint8_t sequence, const uint8_t* payload,
                              size_t payload_size);
  uint8_t HandleCommand(uint8_t command, uint8_t sequence,
                        const uint8_t* payload, size_t payload_size);
  uint8_t StartServo(const math::servo_mode::ServoMode::Command& command);
  uint8_t StartCurrent(float id_A, float iq_A);
  uint8_t StartMit(const math::servo_mode::MitMode::Command& command);
  uint8_t HandleStop();
  uint8_t HandleQuery();
  uint8_t HandleServo(const uint8_t* payload, size_t payload_size);
  uint8_t HandleEncoderCompensation(const uint8_t* payload,
                                    size_t payload_size);
  uint8_t HandleCalibration(const uint8_t* payload, size_t payload_size);
  uint8_t HandleConfiguration(uint8_t sequence, const uint8_t* payload,
                              size_t payload_size);
  uint8_t HandleInfo(uint8_t sequence);
  uint8_t HandleSnapshot(uint8_t sequence, const uint8_t* payload,
                         size_t payload_size);
  bool SendConfigurationGroup(uint8_t sequence, uint8_t operation,
                              uint8_t group);

  hal::FDCan* can_ = nullptr;
  middleware::communication::BinaryLink* link_ = nullptr;
  control::MotorControlService* motor_control_ = nullptr;
  encoder::EncoderService* encoder_ = nullptr;
  calibration::CalibrationManager* calibration_ = nullptr;
  snapshot::SnapshotService* snapshot_ = nullptr;
  app::IRuntimeConfigService* runtime_config_ = nullptr;
  TelemetryPublisher* telemetry_ = nullptr;
  const char* motor_name_ = nullptr;
  uint16_t nominal_pwm_hz_ = 0;
  bool control_allowed_ = false;
  bool driver_fault_ = false;
};

}  // namespace middleware::communication
