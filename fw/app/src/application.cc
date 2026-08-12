#include "application.h"

namespace app
{
namespace
{

// The host normally streams Query/control at 50 Hz. A 500 ms grace period
// tolerates scheduler and CAN stalls while still removing torque on link loss.
constexpr uint32_t kCommandTimeoutUs = 500000u;

}  // namespace

Application::Application(const Dependencies& dependencies)
    : platform_(*dependencies.platform),
      encoder_(*dependencies.encoder),
      calibration_(*dependencies.calibration),
      motor_control_(*dependencies.motor_control),
      runtime_config_(*dependencies.runtime_config),
      commands_(*dependencies.commands),
      telemetry_(*dependencies.telemetry)
{
}

[[noreturn]] void Application::Run()
{
  // Only this main loop feeds the watchdog. A stalled main loop or runaway
  // higher-priority ISR therefore resets the MCU instead of freezing PWM.
  platform_.StartWatchdog();

  while (true)
  {
    platform_.FeedWatchdog();
    switch (lifecycle_.state())
    {
      case State::INIT:
        lifecycle_.CompleteInitialization(Initialize());
        break;
      case State::INIT_FAILED:
        RunInitializationFailure();
        break;
      case State::RUN:
        RunOperational();
        break;
      case State::DRIVER_FAULT:
        RunDriverFault();
        break;
      case State::ENTER_BOOTLOADER:
        RunBootloaderTransition();
        break;
    }
  }
}

bool Application::Initialize()
{
  // Initialization order is electrical: keep the power stage controlled before
  // starting synchronized sampling, then initialize feedback and configuration.
  platform_.InitializeStatusIndicator();
  if (!platform_.InitializeGateDriver()) return false;
  if (!platform_.InitializeCurrentSense()) return false;
  if (!platform_.CalibrateCurrentOffset()) return false;
  if (!platform_.InitializePwm()) return false;
  if (!platform_.StartSynchronizedCurrentSampling()) return false;

  // Sensor faults are non-fatal at boot. Encoder-dependent control sessions
  // reject their own start requests until the sensor becomes valid.
  (void)encoder_.Init();

  // Configuration selects defaults/Flash, applies all algorithm parameters,
  // and restores persistent calibration before commands are accepted.
  if (!runtime_config_.Initialize()) return false;

  last_receive_us_ = platform_.MonotonicUs();
  platform_.SetStatusIndicator(true);
  return true;
}

void Application::RunOperational()
{
  if (platform_.DriverFaulted())
  {
    lifecycle_.DetectDriverFault();
    StopMotor();
    platform_.DisablePowerStage();
    return;
  }

  // Flash erase stalls the CPU while PWM hardware keeps its previous duty.
  // Every calibration write is therefore preceded by an explicit motor stop.
  if (calibration_.has_pending_persist())
  {
    StopMotor();
  }
  calibration_.PersistPending();

  PollCommandIngress(true, false);
  if (lifecycle_.state() != State::RUN)
  {
    return;
  }
  EnforceCommandTimeout();
  PollTelemetryEgress();
  platform_.SetStatusIndicator(true);
}

void Application::RunInitializationFailure()
{
  // Never treat a generic bring-up failure as a recoverable gate-driver fault.
  // Latch safe shutdown and remain bootloader-accessible. Re-running partial
  // ADC/PWM initialization in place is unsafe because sampling may already be
  // hardware-triggered; only reset/bootloader entry may leave this state.
  StopMotor();
  platform_.DisablePowerStage();
  PollCommandIngress(false, true);
  if (lifecycle_.state() == State::ENTER_BOOTLOADER)
  {
    return;
  }
  platform_.IndicateDriverFault();
}

void Application::RunDriverFault()
{
  StopMotor();
  platform_.DisablePowerStage();
  PollCommandIngress(false, true);
  if (lifecycle_.state() == State::ENTER_BOOTLOADER)
  {
    return;
  }

  if (platform_.TryRecoverDriverFault())
  {
    platform_.SetStatusIndicator(true);
    lifecycle_.CompleteDriverRecovery(true);
    return;
  }
  platform_.IndicateDriverFault();
}

void Application::PollCommandIngress(bool control_allowed, bool driver_fault)
{
  const CommandPollResult result =
      commands_.Poll(CommandPollContext{control_allowed, driver_fault});
  if (IsCommandActivity(result.bootloader_requested, result.received_frame))
  {
    last_receive_us_ = platform_.MonotonicUs();
  }
  if (result.bootloader_requested)
  {
    lifecycle_.RequestBootloader();
    StopMotor();
    platform_.DisablePowerStage();
  }
}

void Application::PollTelemetryEgress()
{
  telemetry_.Poll();
}

void Application::EnforceCommandTimeout()
{
  const uint32_t now_us = platform_.MonotonicUs();
  if (CommandDeadlineExpired(motor_control_.active(), now_us,
                             last_receive_us_, kCommandTimeoutUs))
  {
    StopMotor();
  }
}

void Application::StopMotor()
{
  motor_control_.Stop();
}


[[noreturn]] void Application::RunBootloaderTransition()
{
  StopMotor();
  platform_.EnterBootloader();
  __builtin_unreachable();
}

}  // namespace app
