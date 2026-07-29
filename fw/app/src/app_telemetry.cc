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

  // Prefer ISR-cached sample while control is running (avoid racing Read()).
  const bool control_on =
      application_ != nullptr &&
      ((application_->voltage_foc() != nullptr &&
        application_->voltage_foc()->active()) ||
       (application_->current_loop() != nullptr &&
        application_->current_loop()->active()));
  const auto s =
      control_on ? application_->last_current() : current_adc_->Read();
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

  // dq snapshot from Application (vfoc observe or current-loop).
  // *_mA_p100k: milliamp + 100000 bias (100000 == 0 A).
  if (application_ != nullptr)
  {
    pos = AppendKeyUInt(out, out_capacity, pos, "dq",
                        application_->dq_valid() ? 1u : 0u, true);
    pos = AppendKeyUInt(out, out_capacity, pos, "id_mA_p100k",
                        mA(application_->id_A()), true);
    pos = AppendKeyUInt(out, out_capacity, pos, "iq_mA_p100k",
                        mA(application_->iq_A()), true);
    const control::CurrentLoop* loop = application_->current_loop();
    pos = AppendKeyUInt(out, out_capacity, pos, "iloop",
                        (loop != nullptr && loop->active()) ? 1u : 0u, true);
    if (loop != nullptr)
    {
      pos = AppendKeyUInt(out, out_capacity, pos, "idref_mA_p100k",
                          mA(loop->id_ref_A()), true);
      pos = AppendKeyUInt(out, out_capacity, pos, "iqref_mA_p100k",
                          mA(loop->iq_ref_A()), true);
    }
  }
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
  pos = AppendKeyUInt(out, out_capacity, pos, "cisr",
                      phase_pwm_->control_isr_on() ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "hz", phase_pwm_->rate_hz(), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "arr",
                      static_cast<unsigned>(phase_pwm_->arr()), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "a", phase_pwm_->duty_a(), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "b", phase_pwm_->duty_b(), true);
  pos = AppendKeyUInt(out, out_capacity, pos, "c", phase_pwm_->duty_c(), true);

  // Open-loop voltage FOC / current-loop snapshot (milli-units; signed via bias).
  const control::VoltageFoc* foc =
      (application_ != nullptr) ? application_->voltage_foc() : nullptr;
  const control::CurrentLoop* loop =
      (application_ != nullptr) ? application_->current_loop() : nullptr;
  const bool foc_on = foc != nullptr && foc->active();
  const bool loop_on = loop != nullptr && loop->active();
  pos = AppendKeyUInt(out, out_capacity, pos, "foc", foc_on ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "iloop", loop_on ? 1u : 0u, true);
  const auto milli_biased = [](float x) -> unsigned {
    const int v = static_cast<int>(x * 1000.0f) + 1000000;
    return v < 0 ? 0u : static_cast<unsigned>(v);
  };
  if (foc_on)
  {
    pos = AppendKeyUInt(out, out_capacity, pos, "th_mrad_p1M",
                        milli_biased(foc->theta_rad()), true);
    pos = AppendKeyUInt(out, out_capacity, pos, "v_mV_p1M",
                        milli_biased(foc->voltage_V()), true);
    pos = AppendKeyUInt(out, out_capacity, pos, "w_mrad_s_p1M",
                        milli_biased(foc->theta_rate_rad_s()), true);
    pos = AppendKeyUInt(out, out_capacity, pos, "bus_mV",
                        static_cast<unsigned>(foc->bus_V() * 1000.0f + 0.5f),
                        true);
  }
  else if (loop_on)
  {
    pos = AppendKeyUInt(out, out_capacity, pos, "th_mrad_p1M",
                        milli_biased(loop->theta_rad()), true);
    pos = AppendKeyUInt(out, out_capacity, pos, "vd_mV_p1M",
                        milli_biased(loop->vd_V()), true);
    pos = AppendKeyUInt(out, out_capacity, pos, "vq_mV_p1M",
                        milli_biased(loop->vq_V()), true);
    pos = AppendKeyUInt(out, out_capacity, pos, "w_mrad_s_p1M",
                        milli_biased(loop->theta_rate_rad_s()), true);
    pos = AppendKeyUInt(out, out_capacity, pos, "bus_mV",
                        static_cast<unsigned>(loop->bus_V() * 1000.0f + 0.5f),
                        true);
  }
  return pos;
}

}  // namespace app
