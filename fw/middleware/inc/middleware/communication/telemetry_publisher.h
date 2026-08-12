// Protocol telemetry construction and CAN publication.
#pragma once

#include <cstddef>
#include <cstdint>

#include "application_ports.h"
#include "middleware/calibration/calibration_manager.h"
#include "middleware/control/motor_control_service.h"
#include "middleware/encoder/encoder_service.h"
#include "middleware/snapshot/snapshot_service.h"
#include "runtime_config_service.h"
#include "middleware/communication/binary_link.h"
#include "protocol/xt_can.h"

namespace middleware::communication
{

class TelemetryPublisher final : public app::ITelemetryPublisher
{
 public:
  struct Dependencies
  {
    middleware::communication::BinaryLink* link = nullptr;
    middleware::control::MotorControlService* motor_control = nullptr;
    middleware::encoder::EncoderService* encoder = nullptr;
    middleware::calibration::CalibrationManager* calibration = nullptr;
    middleware::snapshot::SnapshotService* snapshot = nullptr;
    app::IRuntimeConfigService* runtime_config = nullptr;
  };

  explicit TelemetryPublisher(const Dependencies& dependencies);

  void ReplyControl(uint8_t command, uint8_t sequence, uint8_t status,
                    bool driver_fault);
  void SendInfo(const protocol::xt_can::Info& info);
  bool SendConfig(const void* data, size_t size);

  void Poll() override;

 private:
  protocol::xt_can::CtrlReply BuildControlReply(
      uint8_t command, uint8_t sequence, uint8_t status,
      bool driver_fault) const;
  protocol::xt_can::CalTelem BuildCalibrationTelemetry() const;
  bool CalibrationActive() const;

  middleware::communication::BinaryLink* link_ = nullptr;
  middleware::control::MotorControlService* motor_control_ = nullptr;
  middleware::encoder::EncoderService* encoder_ = nullptr;
  middleware::calibration::CalibrationManager* calibration_ = nullptr;
  middleware::snapshot::SnapshotService* snapshot_ = nullptr;
  app::IRuntimeConfigService* runtime_config_ = nullptr;
};

}  // namespace middleware::communication
