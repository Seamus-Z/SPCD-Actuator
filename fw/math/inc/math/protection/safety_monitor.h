// ISR-safe current / bus / FET / I²t trip policy. No IO.
#pragma once

#include <cstdint>

namespace math {
namespace protection {

enum class Trip : uint8_t
{
  None = 0,
  CurrentSenseInvalid,
  PeakOvercurrent,
  I2t,
  BusUndervoltage,
  BusOvervoltage,
  FetOvertemperature,
};

class SafetyMonitor
{
 public:
  struct Config
  {
    float overcurrent_A = 0.0f;
    uint8_t overcurrent_count = 5;
    float i2t_rated_A = 0.0f;
    float i2t_tau_s = 2.0f;
    float bus_min_V = 8.0f;
    float bus_max_V = 54.0f;
    float fet_fault_C = 100.0f;
  };

  struct Input
  {
    bool current_ok = false;
    float i1_A = 0.0f;
    float i2_A = 0.0f;
    float i3_A = 0.0f;
    bool bus_ok = false;
    float bus_V = 0.0f;
    bool fet_ok = false;
    float fet_temp_C = 0.0f;
    float dt_s = 0.0f;
  };

  void set_config(const Config& config) { config_ = config; }
  const Config& config() const { return config_; }

  void Reset()
  {
    overcurrent_count_ = 0;
    i2t_energy_A2s_ = 0.0f;
    last_trip_ = Trip::None;
  }

  Trip last_trip() const { return last_trip_; }
  float i2t_energy_A2s() const { return i2t_energy_A2s_; }

  Trip Evaluate(const Input& in)
  {
    if (!in.current_ok)
    {
      last_trip_ = Trip::CurrentSenseInvalid;
      return last_trip_;
    }

    const float ia = in.i1_A >= 0.0f ? in.i1_A : -in.i1_A;
    const float ib = in.i2_A >= 0.0f ? in.i2_A : -in.i2_A;
    const float ic = in.i3_A >= 0.0f ? in.i3_A : -in.i3_A;
    float i_abs = ia;
    if (ib > i_abs) i_abs = ib;
    if (ic > i_abs) i_abs = ic;

    if (config_.overcurrent_A > 0.0f && i_abs > config_.overcurrent_A)
    {
      if (++overcurrent_count_ >= config_.overcurrent_count)
      {
        last_trip_ = Trip::PeakOvercurrent;
        return last_trip_;
      }
    }
    else
    {
      overcurrent_count_ = 0;
    }

    if (config_.i2t_rated_A > 0.0f && config_.i2t_tau_s > 0.0f &&
        in.dt_s > 0.0f)
    {
      const float rated2 = config_.i2t_rated_A * config_.i2t_rated_A;
      i2t_energy_A2s_ += (i_abs * i_abs - rated2) * in.dt_s;
      if (i2t_energy_A2s_ < 0.0f)
      {
        i2t_energy_A2s_ = 0.0f;
      }
      const float limit = rated2 * config_.i2t_tau_s;
      if (i2t_energy_A2s_ > limit)
      {
        last_trip_ = Trip::I2t;
        return last_trip_;
      }
    }

    if (in.bus_ok)
    {
      if (in.bus_V < config_.bus_min_V)
      {
        last_trip_ = Trip::BusUndervoltage;
        return last_trip_;
      }
      if (in.bus_V > config_.bus_max_V)
      {
        last_trip_ = Trip::BusOvervoltage;
        return last_trip_;
      }
    }

    if (in.fet_ok && in.fet_temp_C > config_.fet_fault_C)
    {
      last_trip_ = Trip::FetOvertemperature;
      return last_trip_;
    }

    last_trip_ = Trip::None;
    return last_trip_;
  }

 private:
  Config config_{};
  uint8_t overcurrent_count_ = 0;
  float i2t_energy_A2s_ = 0.0f;
  Trip last_trip_ = Trip::None;
};

}  // namespace protection
}  // namespace math
