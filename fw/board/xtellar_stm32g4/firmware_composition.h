// Concrete xtellar STM32G4 firmware object graph.
// Application owns business orchestration only; this composition root owns
// concrete platform, device, algorithm, and service instances.
#pragma once

#include "core/memory/pool.h"

namespace app::board
{

class FirmwareComposition
{
 public:
  explicit FirmwareComposition(::core::memory::Pool* pool);

  [[noreturn]] void Run();

 private:
  class Impl;
  ::core::memory::PoolPtr<Impl> impl_;
};

}  // namespace app::board
