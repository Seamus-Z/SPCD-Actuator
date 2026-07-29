#include "app_telemetry.h"

#include "HAL/fdcan.h"
#include "HAL/millisecond_timer.h"
#include "application.h"
#include "telemetry/diagnostic_server.h"
#include "telemetry/text_format.h"

namespace app
{

AppTelemetry::AppTelemetry(telemetry::StatusRegistry* registry,
                           const State* state,
                           device::Drv8353s* gate_driver,
                           ::pool::Pool* pool)
    : registry_(registry),
      state_(state),
      gate_driver_(gate_driver),
      pool_(pool)
{
}

void AppTelemetry::Register()
{
  if (registry_ == nullptr)
  {
    return;
  }

  if (gate_driver_ != nullptr)
  {
    registry_->Register(device::Drv8353s::kTelemetryChannel,
                        &device::Drv8353s::TelemetryExport,
                        gate_driver_);
  }
  registry_->Register(kStatusChannel, &AppTelemetry::StatusExport, this);
  registry_->Register(kMemChannel, &AppTelemetry::MemExport, this);
}

size_t AppTelemetry::StatusExport(void* context, char* out, size_t out_capacity)
{
  if (context == nullptr)
  {
    return 0;
  }
  return static_cast<AppTelemetry*>(context)->FormatStatus(out, out_capacity);
}

size_t AppTelemetry::MemExport(void* context, char* out, size_t out_capacity)
{
  if (context == nullptr)
  {
    return 0;
  }
  return static_cast<AppTelemetry*>(context)->FormatMem(out, out_capacity);
}

size_t AppTelemetry::FormatStatus(char* out, size_t out_capacity) const
{
  if (out == nullptr || out_capacity == 0 || state_ == nullptr ||
      gate_driver_ == nullptr)
  {
    return 0;
  }

  using telemetry::text::AppendKeyUInt;
  using telemetry::text::AppendStr;
  size_t pos = 0;
  pos = AppendStr(out, out_capacity, pos, "state=");
  pos = AppendStr(out, out_capacity, pos, StateName(*state_));
  pos = AppendKeyUInt(out, out_capacity, pos, "driver_ok",
                      gate_driver_->init_ok() ? 1u : 0u, true);
  return pos;
}

size_t AppTelemetry::FormatMem(char* out, size_t out_capacity) const
{
  if (out == nullptr || out_capacity == 0 || pool_ == nullptr)
  {
    return 0;
  }

  using telemetry::text::AppendKeyUInt;
  size_t pos = 0;
  pos = AppendKeyUInt(out, out_capacity, pos, "pool",
                      static_cast<unsigned>(pool_->size()), false);
  pos = AppendKeyUInt(out, out_capacity, pos, "used",
                      static_cast<unsigned>(pool_->used()), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "avail",
                      static_cast<unsigned>(pool_->available()), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "app",
                      static_cast<unsigned>(sizeof(Application)), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "timer",
                      static_cast<unsigned>(sizeof(hal::MillisecondTimer)),
                      true);
  pos = AppendKeyUInt(out, out_capacity, pos, "can",
                      static_cast<unsigned>(sizeof(hal::FDCan)), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "drv",
                      static_cast<unsigned>(sizeof(device::Drv8353s)), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "registry",
                      static_cast<unsigned>(sizeof(telemetry::StatusRegistry)),
                      true);
  pos = AppendKeyUInt(out, out_capacity, pos, "diag",
                      static_cast<unsigned>(sizeof(telemetry::DiagnosticServer)),
                      true);
  pos = AppendKeyUInt(out, out_capacity, pos, "telem",
                      static_cast<unsigned>(sizeof(AppTelemetry)), true);
  return pos;
}

}  // namespace app
