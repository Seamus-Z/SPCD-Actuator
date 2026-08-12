// Business-facing runtime configuration service.
#pragma once

#include "middleware/config/runtime_config.h"

namespace app
{

class IRuntimeConfigService
{
 public:
  virtual ~IRuntimeConfigService() = default;

  virtual bool Initialize() = 0;
  virtual const middleware::config::RuntimeConfig& config() const = 0;
  virtual middleware::config::RuntimeConfig& mutable_config() = 0;
  virtual bool flash_valid() const = 0;

  virtual void Apply() = 0;
  virtual bool Save() = 0;
  virtual bool Load() = 0;
  virtual void RestoreDefaults() = 0;
};

}  // namespace app
