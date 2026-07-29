// Open-loop-angle current FOC (main-loop bring-up; later move to PWM ISR).
// θ is integrated; Id/Iq PI → Vd/Vq → InverseDq → BalancedPwm.
#pragma once

#include <cstdint>

#include "math/constants.h"
#include "math/foc.h"

namespace control
{

class CurrentLoop
{
 public:
  struct Options
  {
    float bus_V = 48.0f;
    float min_duty = 0.05f;
    float max_duty = 0.95f;
    // PI: V = kp*err + integral; integral += ki*err*dt
    // DM4310 table I-gains look per-PWM-cycle; convert with pwm_hz.
    float kp_d = 0.35f;
    float ki_d = 0.015f * 15000.0f;
    float kp_q = 0.35f;
    float ki_q = 0.015f * 15000.0f;
    float max_current_A = 2.0f;  // command clamp (below motor max for bring-up)
  };

  struct Command
  {
    float theta_rad = 0.0f;
    float id_A = 0.0f;
    float iq_A = 0.0f;
    float theta_rate_rad_s = 0.0f;
  };

  struct Duties
  {
    uint16_t a_milli = 0;
    uint16_t b_milli = 0;
    uint16_t c_milli = 0;
  };

  explicit CurrentLoop(const Options& options) : options_(options) {}

  void Start(const Command& cmd)
  {
    theta_ = math::WrapZeroToTwoPi(cmd.theta_rad);
    id_ref_ = ClampCurrent(cmd.id_A);
    iq_ref_ = ClampCurrent(cmd.iq_A);
    theta_rate_ = cmd.theta_rate_rad_s;
    integral_d_ = 0.0f;
    integral_q_ = 0.0f;
    id_A_ = 0.0f;
    iq_A_ = 0.0f;
    vd_V_ = 0.0f;
    vq_V_ = 0.0f;
    active_ = true;
  }

  void Stop()
  {
    active_ = false;
    integral_d_ = 0.0f;
    integral_q_ = 0.0f;
  }

  bool active() const { return active_; }
  float theta_rad() const { return theta_; }
  float theta_rate_rad_s() const { return theta_rate_; }
  float id_ref_A() const { return id_ref_; }
  float iq_ref_A() const { return iq_ref_; }
  float id_A() const { return id_A_; }
  float iq_A() const { return iq_A_; }
  float vd_V() const { return vd_V_; }
  float vq_V() const { return vq_V_; }
  float bus_V() const { return options_.bus_V; }

  // Park measured currents into id_/iq_ using current θ (no PWM update).
  void Observe(float ia, float ib, float ic)
  {
    const math::SinCos sc = math::SinCosFromRadians(theta_);
    const math::DqTransform dq(sc, ia, ib, ic);
    id_A_ = dq.d;
    iq_A_ = dq.q;
  }

  // Integrate θ, current PI, write duties. Updates id_/iq_ from |ia,ib,ic|.
  bool Step(float dt_s, float ia, float ib, float ic, Duties* out)
  {
    if (!active_ || out == nullptr)
    {
      return false;
    }
    if (dt_s < 0.0f)
    {
      dt_s = 0.0f;
    }

    theta_ = math::WrapZeroToTwoPi(theta_ + theta_rate_ * dt_s);
    Observe(ia, ib, ic);

    const float max_voltage =
        (0.5f - options_.min_duty) * options_.bus_V * math::kSvpwmRatio;

    const float err_d = id_ref_ - id_A_;
    const float err_q = iq_ref_ - iq_A_;

    integral_d_ += options_.ki_d * err_d * dt_s;
    integral_q_ += options_.ki_q * err_q * dt_s;
    // Anti-windup: keep integrator inside voltage budget.
    integral_d_ = Clamp(integral_d_, -max_voltage, max_voltage);
    integral_q_ = Clamp(integral_q_, -max_voltage, max_voltage);

    vd_V_ = Clamp(options_.kp_d * err_d + integral_d_, -max_voltage,
                  max_voltage);
    vq_V_ = Clamp(options_.kp_q * err_q + integral_q_, -max_voltage,
                  max_voltage);

    const math::SinCos sc = math::SinCosFromRadians(theta_);
    const math::InverseDqTransform idt(sc, vd_V_, vq_V_);

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
  float ClampCurrent(float i) const
  {
    return Clamp(i, -options_.max_current_A, options_.max_current_A);
  }

  static float Clamp(float x, float lo, float hi)
  {
    if (x < lo)
    {
      return lo;
    }
    if (x > hi)
    {
      return hi;
    }
    return x;
  }

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
  float theta_rate_ = 0.0f;
  float id_ref_ = 0.0f;
  float iq_ref_ = 0.0f;
  float integral_d_ = 0.0f;
  float integral_q_ = 0.0f;
  float id_A_ = 0.0f;
  float iq_A_ = 0.0f;
  float vd_V_ = 0.0f;
  float vq_V_ = 0.0f;
};

}  // namespace control
