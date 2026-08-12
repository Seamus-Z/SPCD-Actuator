// Top-level application lifecycle states.
#pragma once

namespace app
{

enum class State
{
  INIT,
  INIT_FAILED,
  RUN,
  DRIVER_FAULT,
  ENTER_BOOTLOADER,
};

inline const char* StateName(State state)
{
  switch (state)
  {
    case State::INIT: return "INIT";
    case State::INIT_FAILED: return "INIT_FAILED";
    case State::RUN: return "RUN";
    case State::DRIVER_FAULT: return "DRIVER_FAULT";
    case State::ENTER_BOOTLOADER: return "ENTER_BOOTLOADER";
  }
  return "?";
}

}  // namespace app
