// Open-loop voltage FOC (moteus kVoltageFoc / "d pwm").
// Electrical θ is integrated; Vd=V, Vq=0 → InverseDq → BalancedPwm.
#pragma once

#include <cstdint>

#include "math/constants.h"
#include "math/foc.h"

namespace control
{

class VoltageFoc
{
 public:
  struct Options
  {
    float bus_V = 24.0f;
    float min_duty = 0.05f;  // match PhasePwm::kDutyMinMilli / 1000
    float max_duty = 0.95f;
  };

  struct Command
  {
    float theta_rad = 0.0f;
    float voltage_V = 0.0f;
    float theta_rate_rad_s = 0.0f;
  };

  struct Duties
  {
    uint16_t a_milli = 0;
    uint16_t b_milli = 0;
    uint16_t c_milli = 0;
  };

  explicit VoltageFoc(const Options& options) : options_(options) {}

  void Start(const Command& cmd)
  {
    theta_ = math::WrapZeroToTwoPi(cmd.theta_rad);
    voltage_ = cmd.voltage_V;
    theta_rate_ = cmd.theta_rate_rad_s;
    active_ = true;
  }

  void Stop() { active_ = false; }

  bool active() const { return active_; }
  float theta_rad() const { return theta_; }
  float voltage_V() const { return voltage_; }
  float theta_rate_rad_s() const { return theta_rate_; }
  float bus_V() const { return options_.bus_V; }

  // Park measured currents into caller-provided id/iq using current θ.
  void Observe(float ia, float ib, float ic, float* id_A, float* iq_A) const
  {
    if (id_A == nullptr || iq_A == nullptr)
    {
      return;
    }
    const math::SinCos sc = math::SinCosFromRadians(theta_);
    const math::DqTransform dq(sc, ia, ib, ic);
    *id_A = dq.d;
    *iq_A = dq.q;
  }

  // Integrate θ and map Vd=V, Vq=0 to PWM thousandths.
  // |dt_s| should be measured; large stalls should be capped by the caller.
  bool Step(float dt_s, Duties* out)
  {
    if (!active_ || out == nullptr)
    {
      return false;
    }

    theta_ = math::WrapZeroToTwoPi(theta_ + theta_rate_ * dt_s);

    const float max_voltage =
        (0.5f - options_.min_duty) * options_.bus_V * math::kSvpwmRatio;
    float vd = voltage_;
    if (vd > max_voltage)
    {
      vd = max_voltage;
    }
    else if (vd < -max_voltage)
    {
      vd = -max_voltage;
    }

    const math::SinCos sc = math::SinCosFromRadians(theta_);
    const math::InverseDqTransform idt(sc, vd, 0.0f);

    math::BalancedPwm pwm;
    if (!math::BalancedPwm::FromPhaseVolts(idt.a, idt.b, idt.c, options_.bus_V,
                                          options_.min_duty, options_.max_duty,
                                          &pwm))
    {
      return false;
    }

    out->a_milli = DutyToMilli(pwm.duty_a);
    out->b_milli = DutyToMilli(pwm.duty_b);
    out->c_milli = DutyToMilli(pwm.duty_c);
    return true;
  }

 private:
  static uint16_t DutyToMilli(float duty)
  {
    float x = duty * 1000.0f + 0.5f;
    if (x < 0.0f)
    {
      x = 0.0f;
    }
    if (x > 1000.0f)
    {
      x = 1000.0f;
    }
    return static_cast<uint16_t>(x);
  }

  Options options_;
  bool active_ = false;
  float theta_ = 0.0f;
  float voltage_ = 0.0f;
  float theta_rate_ = 0.0f;
};

}  // namespace control
