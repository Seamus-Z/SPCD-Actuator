// Motor-angle sensor capability consumed by EncoderService.
// Counts and transfer timing are device concerns; motor-angle semantics are not.
#pragma once

#include <cstdint>

namespace ports
{

class IAngleSensor
{
 public:
  virtual ~IAngleSensor() = default;

  virtual bool Init() = 0;
  virtual uint32_t Sample() = 0;
  virtual void StartSample() = 0;
  virtual uint32_t FinishSample() = 0;
  virtual uint32_t raw() const = 0;
  virtual uint32_t counts_per_turn() const = 0;
  virtual bool ok() const = 0;
  virtual bool SetFilterUs(uint16_t filter_us) = 0;
};

}  // namespace ports
