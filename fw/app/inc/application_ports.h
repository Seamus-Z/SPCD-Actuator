// Stable ports consumed by the top-level application state machine.
#pragma once

#include <cstdint>

namespace app
{

struct CommandPollContext
{
  bool control_allowed = false;
  bool driver_fault = false;
};

struct CommandPollResult
{
  bool received_frame = false;
  bool bootloader_requested = false;
};

class ICommandEndpoint
{
 public:
  virtual ~ICommandEndpoint() = default;
  virtual CommandPollResult Poll(const CommandPollContext& context) = 0;
};

class ITelemetryPublisher
{
 public:
  virtual ~ITelemetryPublisher() = default;
  virtual void Poll() = 0;
};

class IPlatformService
{
 public:
  virtual ~IPlatformService() = default;

  virtual void StartWatchdog() = 0;
  virtual void FeedWatchdog() = 0;

  virtual void InitializeStatusIndicator() = 0;
  virtual bool InitializeGateDriver() = 0;
  virtual bool InitializeCurrentSense() = 0;
  virtual bool CalibrateCurrentOffset() = 0;
  virtual bool InitializePwm() = 0;
  virtual bool StartSynchronizedCurrentSampling() = 0;

  virtual bool DriverFaulted() = 0;
  virtual bool TryRecoverDriverFault() = 0;
  virtual void DisablePowerStage() = 0;
  virtual void SetStatusIndicator(bool on) = 0;
  virtual void IndicateDriverFault() = 0;

  // Monotonic main-loop time. The implementation extends a wrapping hardware
  // timer; callers must poll it continuously while the application is running.
  virtual uint32_t MonotonicUs() = 0;

  [[noreturn]] virtual void EnterBootloader() = 0;
};

}  // namespace app
