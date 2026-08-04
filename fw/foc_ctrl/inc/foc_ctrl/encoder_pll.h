// Single-source encoder PLL (moteus MotorPosition SPI subset).
// Units: mechanical radians; velocity in mech rad/s.
#pragma once

#include "math/constants.h"

namespace foc_ctrl
{

class EncoderPll
{
 public:
  struct Options
  {
    float pll_filter_hz = 400.0f;  // moteus default (3dB-ish via /2.48)
    float pole_pairs = 14.0f;
  };

  explicit EncoderPll(const Options& options) : options_(options)
  {
    RecomputeGains();
  }

  void SetOptions(const Options& options)
  {
    options_ = options;
    RecomputeGains();
  }

  void Reset(float theta_mech_rad)
  {
    filtered_ = math::WrapZeroToTwoPi(theta_mech_rad);
    integral_ = 0.0f;
    velocity_ = 0.0f;
    position_ = filtered_;
    electrical_theta_ =
        math::WrapZeroToTwoPi(position_ * options_.pole_pairs);
    theta_valid_ = true;
    have_sample_ = true;
  }

  // Call every control period with calibrated mechanical angle [0, 2π).
  void Update(float theta_mech_rad, float dt_s)
  {
    if (dt_s < 1.0e-7f)
    {
      dt_s = 1.0e-7f;
    }
    const float measured = math::WrapZeroToTwoPi(theta_mech_rad);

    if (!have_sample_)
    {
      Reset(measured);
      return;
    }

    // moteus: error = Wrap(compensated - filtered)
    const float error = math::WrapNegPiToPi(measured - filtered_);
    integral_ += dt_s * ki_ * error;
    velocity_ = integral_ + kp_ * error;

    // Limit to < 1 rev per 8 samples (moteus-style).
    const float max_vel = math::k2Pi * 0.125f / dt_s;
    integral_ = LimitVel(integral_, max_vel);
    velocity_ = LimitVel(velocity_, max_vel);

    filtered_ = math::WrapZeroToTwoPi(filtered_ + dt_s * velocity_);
    // Unwrapped output position: integrate only. Do NOT snap to wrapped
    // filtered_ (that collapses position into [0,2π) while control_position
    // keeps integrating → runaway Iq / supply collapse).
    position_ += velocity_ * dt_s;

    // Commutation θ from continuous mech position (not wrapped filtered_*pp),
    // so electrical wrap is a pure 2π discontinuity for sin/cos only.
    electrical_theta_ =
        math::WrapZeroToTwoPi(position_ * options_.pole_pairs);
    theta_valid_ = true;
  }

  bool theta_valid() const { return theta_valid_; }
  float filtered_mech_rad() const { return filtered_; }
  float position_rad() const { return position_; }       // unwrapped mech
  float velocity_mech() const { return velocity_; }      // rad/s mech
  float electrical_theta() const { return electrical_theta_; }
  float omega_elec() const { return velocity_ * options_.pole_pairs; }

 private:
  void RecomputeGains()
  {
    // w_n from 3dB cutoff: r(zeta=1)≈2.48 (moteus motor_position.h)
    const float w_n = (options_.pll_filter_hz / 2.48f) * math::k2Pi;
    constexpr float kZeta = 1.0f;
    kp_ = 2.0f * kZeta * w_n;
    ki_ = w_n * w_n;
  }

  static float LimitVel(float v, float max_v)
  {
    if (v > max_v)
    {
      return max_v;
    }
    if (v < -max_v)
    {
      return -max_v;
    }
    return v;
  }

  Options options_;
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float filtered_ = 0.0f;
  float integral_ = 0.0f;
  float velocity_ = 0.0f;
  float position_ = 0.0f;
  float electrical_theta_ = 0.0f;
  bool theta_valid_ = false;
  bool have_sample_ = false;
};

}  // namespace foc_ctrl
