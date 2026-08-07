// Single-source encoder PLL (moteus MotorPosition SPI subset).
// Units: mechanical radians; velocity in mech rad/s.
//
// Aligned with moteus motor_position.h:
//  - PLL tracks compensated mechanical angle
//  - electrical_theta is derived from filtered_ (not pure velocity integral)
#pragma once

#include <cstdint>

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
    // Absolute mechanical velocity clamp. Zero keeps only the moteus
    // "1 rev / 8 samples" limit.
    float max_velocity_mech_rad_s = 0.0f;
    // Count a sample as a glitch when |wrap(error)| exceeds this (rad).
    // At 15 kHz / 20 rad/s the per-sample angle is ~0.0013 rad, so 0.15 rad
    // (~8.6 deg) is far above normal tracking error.
    float spike_error_rad = 0.15f;
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

  void Reset(float theta_mech_rad, float electrical_correction_rad = 0.0f)
  {
    filtered_ = math::WrapZeroToTwoPi(theta_mech_rad);
    integral_ = 0.0f;
    velocity_ = 0.0f;
    position_ = filtered_;
    electrical_correction_rad_ = electrical_correction_rad;
    // moteus: electrical_theta from filtered ratio, not free-running integral.
    electrical_theta_ = math::WrapZeroToTwoPi(
        filtered_ * options_.pole_pairs + electrical_correction_rad_);
    theta_valid_ = true;
    have_sample_ = true;
  }

  // Call every control period with compensated mechanical angle [0, 2π).
  // Prefer applying commutation/sign/offset before this call (moteus order:
  // compensated_value -> PLL -> electrical_theta from filtered).
  void Update(float theta_mech_rad, float dt_s,
              float electrical_correction_rad = 0.0f)
  {
    if (dt_s < 1.0e-7f)
    {
      dt_s = 1.0e-7f;
    }
    const float measured = math::WrapZeroToTwoPi(theta_mech_rad);

    if (!have_sample_)
    {
      Reset(measured, electrical_correction_rad);
      return;
    }

    // moteus: error = Wrap(compensated - filtered)
    const float error = math::WrapNegPiToPi(measured - filtered_);
    integral_ += dt_s * ki_ * error;
    velocity_ = integral_ + kp_ * error;

    // Limit to < 1 rev per 8 samples (moteus-style), then apply the absolute
    // board ceiling so a one-sample angle jump cannot look like 10k rad/s.
    float max_vel = math::k2Pi * 0.125f / dt_s;
    if (options_.max_velocity_mech_rad_s > 0.0f &&
        options_.max_velocity_mech_rad_s < max_vel)
    {
      max_vel = options_.max_velocity_mech_rad_s;
    }
    const float abs_error = (error >= 0.0f) ? error : -error;
    const bool error_spike =
        options_.spike_error_rad > 0.0f && abs_error >= options_.spike_error_rad;
    const bool vel_before_clamp = (velocity_ > max_vel) || (velocity_ < -max_vel);
    if (error_spike || vel_before_clamp)
    {
      NoteSpike(abs_error, velocity_);
    }
    integral_ = LimitVel(integral_, max_vel);
    velocity_ = LimitVel(velocity_, max_vel);

    filtered_ = math::WrapZeroToTwoPi(filtered_ + dt_s * velocity_);
    // Unwrapped output position: integrate only. Used by position/velocity
    // control; commutation uses filtered_ below (moteus-aligned).
    position_ += velocity_ * dt_s;

    electrical_correction_rad_ = electrical_correction_rad;
    electrical_theta_ = math::WrapZeroToTwoPi(
        filtered_ * options_.pole_pairs + electrical_correction_rad_);
    theta_valid_ = true;
  }

  bool theta_valid() const { return theta_valid_; }
  float filtered_mech_rad() const { return filtered_; }
  float position_rad() const { return position_; }       // unwrapped mech
  float velocity_mech() const { return velocity_; }      // rad/s mech
  float electrical_theta() const { return electrical_theta_; }
  float omega_elec() const { return velocity_ * options_.pole_pairs; }

  // Sticky lifetime counter (saturates). Useful in Live logs while hunting
  // sudden surge/reverse: if this climbs, encoder/SPI glitches are real.
  uint32_t spike_count() const { return spike_count_; }
  float last_spike_error_rad() const { return last_spike_error_rad_; }
  float last_spike_velocity_rad_s() const { return last_spike_velocity_rad_s_; }
  void ClearSpikeCount()
  {
    spike_count_ = 0;
    last_spike_error_rad_ = 0.0f;
    last_spike_velocity_rad_s_ = 0.0f;
  }

 private:
  void NoteSpike(float abs_error_rad, float velocity_rad_s)
  {
    if (spike_count_ < 0xFFFFFFFFu)
    {
      ++spike_count_;
    }
    last_spike_error_rad_ = abs_error_rad;
    last_spike_velocity_rad_s_ = velocity_rad_s;
  }

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
  float electrical_correction_rad_ = 0.0f;
  bool theta_valid_ = false;
  bool have_sample_ = false;
  uint32_t spike_count_ = 0;
  float last_spike_error_rad_ = 0.0f;
  float last_spike_velocity_rad_s_ = 0.0f;
};

}  // namespace foc_ctrl
