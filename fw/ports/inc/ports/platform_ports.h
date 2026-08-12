// Platform-neutral primitive ports used by device drivers.
#pragma once

#include <cstdint>

namespace ports
{

class ISpiBus
{
 public:
  virtual ~ISpiBus() = default;

  virtual uint16_t Transfer16(uint16_t value) = 0;
  virtual void BeginTransfer16(uint16_t value) = 0;
  virtual uint16_t FinishTransfer16() = 0;
};

class IDelay
{
 public:
  virtual ~IDelay() = default;

  virtual void WaitUs(uint32_t delay_us) = 0;
};

}  // namespace ports
