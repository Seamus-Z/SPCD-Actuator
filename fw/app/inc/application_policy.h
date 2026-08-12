// Pure top-level lifecycle and command-safety policy.
#pragma once

#include <cstdint>

#include "app_state.h"

namespace app
{

class LifecycleStateMachine
{
 public:
  State state() const { return state_; }

  void CompleteInitialization(bool success)
  {
    if (state_ == State::INIT)
    {
      state_ = success ? State::RUN : State::INIT_FAILED;
    }
  }

  void DetectDriverFault()
  {
    if (state_ == State::RUN)
    {
      state_ = State::DRIVER_FAULT;
    }
  }

  void CompleteDriverRecovery(bool success)
  {
    // Recovery is valid only for a fault observed from the running power stage.
    // In particular, it can never unlock a partial initialization failure.
    if (state_ == State::DRIVER_FAULT && success)
    {
      state_ = State::RUN;
    }
  }

  void RequestBootloader()
  {
    state_ = State::ENTER_BOOTLOADER;
  }

 private:
  State state_ = State::INIT;
};

constexpr bool IsCommandActivity(bool valid_boot_request,
                                 bool addressed_protocol_frame)
{
  return valid_boot_request || addressed_protocol_frame;
}

constexpr bool CommandDeadlineExpired(bool motor_active, uint32_t now_us,
                                      uint32_t last_receive_us,
                                      uint32_t timeout_us)
{
  return motor_active &&
         static_cast<uint32_t>(now_us - last_receive_us) >= timeout_us;
}

}  // namespace app
