#include "board/xtellar_stm32g4/firmware_composition.h"

#include "HAL/fdcan.h"
#include "HAL/millisecond_timer.h"
#include "HAL/phase_current_adc.h"
#include "HAL/phase_pwm.h"
#include "HAL/vt_sense_adc.h"
#include "HAL/stm32_spi.h"
#include "application.h"
#include "board/xtellar_stm32g4/board_config.h"
#include "board/xtellar_stm32g4/runtime_config_service.h"
#include "board/xtellar_stm32g4/platform_service.h"
#include "device/drv8353s.h"
#include "device/ma600.h"
#include "device/motor.h"
#include "math/foc/controller.h"
#include "math/foc/modulator.h"
#include "math/servo_mode/mit_mode.h"
#include "math/servo_mode/servo_mode.h"
#include "middleware/calibration/calibration_manager.h"
#include "middleware/encoder/encoder_service.h"
#include "middleware/control/motor_control_service.h"
#include "middleware/communication/can_command_adapter.h"
#include "middleware/communication/telemetry_publisher.h"
#include "middleware/snapshot/snapshot_service.h"
#include "middleware/communication/binary_link.h"

namespace app::board
{

class FirmwareComposition::Impl
{
 public:
  explicit Impl(::core::memory::Pool* pool)
      : timer_(pool),
        can_(pool, CanOptions()),
        gate_driver_(pool, timer_.get(), GateDriverOptions()),
        current_adc_(pool, timer_.get(), CurrentSenseOptions()),
        vt_sense_(pool, timer_.get(), VtSenseOptions()),
        phase_pwm_(pool, PhasePwmOptions()),
        dq_modulator_(pool, DqModulatorOptions()),
        foc_(pool, FocControllerOptions()),
        servo_mode_(pool, ServoModeOptions()),
        mit_mode_(pool, MitModeOptions()),
        encoder_spi_(pool, EncoderSpiOptions()),
        angle_sensor_(pool, encoder_spi_.get(), timer_.get(), Ma600Config()),
        encoder_(pool, angle_sensor_.get(), EncoderOptions()),
        calibration_(pool, encoder_.get(), foc_.get(), servo_mode_.get(),
                     phase_pwm_.get()),
        snapshot_(pool),
        motor_control_(pool, middleware::control::MotorControlService::Dependencies{
                                 timer_.get(), gate_driver_.get(), current_adc_.get(),
                                 vt_sense_.get(), phase_pwm_.get(), dq_modulator_.get(),
                                 foc_.get(), servo_mode_.get(), mit_mode_.get(),
                                 encoder_.get(), calibration_.get(), snapshot_.get()}),
        runtime_config_(pool, RuntimeConfigService::Dependencies{
                                  encoder_.get(), calibration_.get(),
                                  motor_control_.get(), dq_modulator_.get(),
                                  foc_.get(), servo_mode_.get(), mit_mode_.get()}),
        binary_link_(pool, can_.get(), 1),
        platform_(pool, PlatformService::Dependencies{
                            timer_.get(), gate_driver_.get(), current_adc_.get(),
                            vt_sense_.get(), phase_pwm_.get(), GateDriverConfig()}),
        telemetry_(pool, middleware::communication::TelemetryPublisher::Dependencies{
                             binary_link_.get(), motor_control_.get(), encoder_.get(),
                             calibration_.get(), snapshot_.get(),
                             runtime_config_.get()}),
        commands_(pool, middleware::communication::CanCommandAdapter::Dependencies{
                            can_.get(), binary_link_.get(), motor_control_.get(),
                            encoder_.get(), calibration_.get(), snapshot_.get(),
                            runtime_config_.get(), telemetry_.get(),
                            device::motor::kActiveName,
                            static_cast<uint16_t>(PhasePwmOptions().rate_hz)}),
        application_(pool, Application::Dependencies{
                               platform_.get(), encoder_.get(), calibration_.get(),
                               motor_control_.get(), runtime_config_.get(),
                               commands_.get(), telemetry_.get()})
  {
  }

  [[noreturn]] void Run() { application_->Run(); }

 private:
  ::core::memory::PoolPtr<hal::MillisecondTimer> timer_;
  ::core::memory::PoolPtr<hal::FDCan> can_;
  ::core::memory::PoolPtr<device::Drv8353s> gate_driver_;
  ::core::memory::PoolPtr<hal::PhaseCurrentAdc> current_adc_;
  ::core::memory::PoolPtr<hal::VtSenseAdc> vt_sense_;
  ::core::memory::PoolPtr<hal::PhasePwm> phase_pwm_;
  ::core::memory::PoolPtr<math::foc::DqModulator> dq_modulator_;
  ::core::memory::PoolPtr<math::foc::FocController> foc_;
  ::core::memory::PoolPtr<math::servo_mode::ServoMode> servo_mode_;
  ::core::memory::PoolPtr<math::servo_mode::MitMode> mit_mode_;
  ::core::memory::PoolPtr<hal::Stm32Spi> encoder_spi_;
  ::core::memory::PoolPtr<device::Ma600> angle_sensor_;
  ::core::memory::PoolPtr<middleware::encoder::EncoderService> encoder_;
  ::core::memory::PoolPtr<middleware::calibration::CalibrationManager> calibration_;
  ::core::memory::PoolPtr<middleware::snapshot::SnapshotService> snapshot_;
  ::core::memory::PoolPtr<middleware::control::MotorControlService> motor_control_;
  ::core::memory::PoolPtr<RuntimeConfigService> runtime_config_;
  ::core::memory::PoolPtr<middleware::communication::BinaryLink> binary_link_;
  ::core::memory::PoolPtr<PlatformService> platform_;
  ::core::memory::PoolPtr<middleware::communication::TelemetryPublisher> telemetry_;
  ::core::memory::PoolPtr<middleware::communication::CanCommandAdapter> commands_;
  ::core::memory::PoolPtr<Application> application_;
};

FirmwareComposition::FirmwareComposition(::core::memory::Pool* pool)
    : impl_(pool, pool)
{
}

[[noreturn]] void FirmwareComposition::Run()
{
  impl_->Run();
}

}  // namespace app::board
