// Top-level firmware business lifecycle and state machine.
#pragma once

#include <cstdint>

#include "app_state.h"
#include "application_policy.h"
#include "application_ports.h"
#include "middleware/calibration/calibration_manager.h"
#include "middleware/control/motor_control_service.h"
#include "middleware/encoder/encoder_service.h"
#include "runtime_config_service.h"

namespace app
{

class Application
{
 public:
  // Every dependency is a service boundary. Concrete STM32G4 devices and CAN
  // protocol types are assembled by FirmwareComposition, not by Application.
  struct Dependencies
  {
    IPlatformService* platform = nullptr;
    middleware::encoder::EncoderService* encoder = nullptr;
    middleware::calibration::CalibrationManager* calibration = nullptr;
    middleware::control::MotorControlService* motor_control = nullptr;
    IRuntimeConfigService* runtime_config = nullptr;
    ICommandEndpoint* commands = nullptr;
    ITelemetryPublisher* telemetry = nullptr;
  };

  explicit Application(const Dependencies& dependencies);

  [[noreturn]] void Run();
  State state() const { return lifecycle_.state(); }

 private:
  bool Initialize();
  void RunOperational();
  void RunInitializationFailure();
  void RunDriverFault();
  void PollCommandIngress(bool control_allowed, bool driver_fault);
  void PollTelemetryEgress();
  void EnforceCommandTimeout();
  void StopMotor();
  void TransitionTo(State next);
  [[noreturn]] void RunBootloaderTransition();

  LifecycleStateMachine lifecycle_{};
  uint32_t last_receive_us_ = 0;

  IPlatformService& platform_;
  middleware::encoder::EncoderService& encoder_;
  middleware::calibration::CalibrationManager& calibration_;
  middleware::control::MotorControlService& motor_control_;
  IRuntimeConfigService& runtime_config_;
  ICommandEndpoint& commands_;
  ITelemetryPublisher& telemetry_;
};

}  // namespace app
