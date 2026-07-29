// Application FSM states (shared by Application and AppTelemetry).
#pragma once

namespace app
{

enum class State
{
  INIT,
  RUN,
  DRIVER_FAULT,
  ENTER_BOOTLOADER,
};

inline const char* StateName(State state)
{
  switch (state)
  {
    case State::INIT: return "INIT";
    case State::RUN: return "RUN";
    case State::DRIVER_FAULT: return "DRIVER_FAULT";
    case State::ENTER_BOOTLOADER: return "ENTER_BOOTLOADER";
  }
  return "?";
}

}  // namespace app
