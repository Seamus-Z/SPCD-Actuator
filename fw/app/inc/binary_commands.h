// CAN binary command dispatch (payload parse + control actions).
// Application owns system timing; this class owns host command semantics.
#pragma once

#include <cstddef>
#include <cstdint>

namespace app
{

class Application;

class BinaryCommands
{
 public:
  explicit BinaryCommands(Application* app) : app_(app) {}

  static uint8_t Thunk(void* context, uint8_t cmd, uint8_t seq,
                       const uint8_t* payload, size_t payload_len);

  uint8_t Handle(uint8_t cmd, uint8_t seq, const uint8_t* payload,
                 size_t payload_len);

 private:
  uint8_t HandleStop();
  uint8_t HandleQuery();
  uint8_t HandleDq(const uint8_t* payload, size_t payload_len);
  uint8_t HandleVel(const uint8_t* payload, size_t payload_len);
  uint8_t HandleCal(const uint8_t* payload, size_t payload_len);
  uint8_t HandleVfoc(const uint8_t* payload, size_t payload_len);
  uint8_t HandleRaw(const uint8_t* payload, size_t payload_len);
  uint8_t HandleInfo(uint8_t seq);
  uint8_t HandleSnap(uint8_t seq, const uint8_t* payload, size_t payload_len);

  Application* app_ = nullptr;
};

}  // namespace app
