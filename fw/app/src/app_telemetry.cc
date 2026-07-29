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
                           hal::PhaseCurrentAdc* current_adc,
                           hal::PhasePwm* phase_pwm,
                           const Application* application,
                           ::pool::Pool* pool)
    : registry_(registry),
      state_(state),
      gate_driver_(gate_driver),
      current_adc_(current_adc),
      phase_pwm_(phase_pwm),
      application_(application),
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
  registry_->Register(hal::PhaseCurrentAdc::kTelemetryChannel,
                      &AppTelemetry::CurExport, this);
  registry_->Register(hal::PhasePwm::kTelemetryChannel,
                      &AppTelemetry::PwmExport, this);
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

size_t AppTelemetry::CurExport(void* context, char* out, size_t out_capacity)
{
  if (context == nullptr)
  {
    return 0;
  }
  return static_cast<AppTelemetry*>(context)->FormatCur(out, out_capacity);
}

size_t AppTelemetry::PwmExport(void* context, char* out, size_t out_capacity)
{
  if (context == nullptr)
  {
    return 0;
  }
  return static_cast<AppTelemetry*>(context)->FormatPwm(out, out_capacity);
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
  pos = AppendKeyUInt(out, out_capacity, pos, "pwr",
                      gate_driver_->power_on() ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "out",
                      (application_ != nullptr && application_->pwm_output_on())
                          ? 1u
                          : 0u,
                      true);
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
  pos = AppendKeyUInt(out, out_capacity, pos, "curadc",
                      static_cast<unsigned>(sizeof(hal::PhaseCurrentAdc)),
                      true);
  pos = AppendKeyUInt(out, out_capacity, pos, "pwm",
                      static_cast<unsigned>(sizeof(hal::PhasePwm)), true);
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

size_t AppTelemetry::FormatCur(char* out, size_t out_capacity) const
{
  if (out == nullptr || out_capacity == 0 || current_adc_ == nullptr)
  {
    return 0;
  }

  const auto s = current_adc_->Read();
  using telemetry::text::AppendKeyUInt;
  size_t pos = 0;
  // err: 0=ok 1=BadConfig 2=EnableFail 3=SampleTimeout 4=OffsetOutOfRange
  //      5=SyncFail
  pos = AppendKeyUInt(out, out_capacity, pos, "err",
                      static_cast<unsigned>(current_adc_->last_error()), false);
  pos = AppendKeyUInt(out, out_capacity, pos, "ok",
                      current_adc_->init_ok() ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "cal",
                      current_adc_->offset_ok() ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "sync",
                      current_adc_->sync_on() ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "seq",
                      current_adc_->seq(), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "samp", s.ok ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "raw1", s.raw1, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "raw2", s.raw2, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "raw3", s.raw3, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "off1",
                      current_adc_->offset1(), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "off2",
                      current_adc_->offset2(), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "off3",
                      current_adc_->offset3(), true);
  // i*_mA_p100k: milliamp + 100000 bias so 100000 == 0 A (unsigned formatter).
  const auto mA = [](float a) -> unsigned {
    const int v = static_cast<int>(a * 1000.0f) + 100000;
    return v < 0 ? 0u : static_cast<unsigned>(v);
  };
  pos = AppendKeyUInt(out, out_capacity, pos, "i1_mA_p100k", mA(s.i1_A), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "i2_mA_p100k", mA(s.i2_A), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "i3_mA_p100k", mA(s.i3_A), true);
  return pos;
}

size_t AppTelemetry::FormatPwm(char* out, size_t out_capacity) const
{
  if (out == nullptr || out_capacity == 0 || phase_pwm_ == nullptr)
  {
    return 0;
  }

  using telemetry::text::AppendKeyUInt;
  size_t pos = 0;
  pos = AppendKeyUInt(out, out_capacity, pos, "ok",
                      phase_pwm_->init_ok() ? 1u : 0u, false);
  pos = AppendKeyUInt(out, out_capacity, pos, "on",
                      (application_ != nullptr && application_->pwm_output_on())
                          ? 1u
                          : 0u,
                      true);
  pos = AppendKeyUInt(out, out_capacity, pos, "trig",
                      phase_pwm_->adc_trigger_on() ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "hz", phase_pwm_->rate_hz(), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "arr",
                      static_cast<unsigned>(phase_pwm_->arr()), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "a", phase_pwm_->duty_a(), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "b", phase_pwm_->duty_b(), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "c", phase_pwm_->duty_c(), true);
  return pos;
}

}  // namespace app
