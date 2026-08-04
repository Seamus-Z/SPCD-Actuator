// Encoder electrical-phase calibration.
// - Spin: moteus-like open-loop both ways (samples gated on speed match)
// - Lock: hold fixed Vd (ω=0), rotor aligns to d-axis → offset (more robust)
#pragma once

#include <cstdint>

extern "C" {
float atan2f(float, float);
float sqrtf(float);
}

#include "math/constants.h"
#include "math/foc.h"

namespace calibration
{

class EncoderPhaseCal
{
 public:
  enum class Method : uint8_t
  {
    Spin = 0,
    Lock = 1,
  };

  enum class State : uint8_t
  {
    Idle = 0,
    SenseSign = 1,
    Fwd = 2,
    Rev = 3,
    Locking = 6,
    Done = 4,
    Failed = 5,
  };

  struct Options
  {
    Method method = Method::Lock;
    float voltage_V = 1.5f;
    float omega_elec_rad_s = 40.0f;  // |ω| for Spin
    float pole_pairs = 14.0f;
    float mech_revs_each_way = 1.0f;
    uint16_t min_samples = 200;
    float sense_timeout_s = 4.0f;
    // Accept sample if |ω_enc - ω_cmd_mech| < rel*|ω_cmd| + floor.
    float track_rel = 0.35f;
    float track_floor_rad_s = 1.0f;
    // Lock method timing.
    float lock_total_s = 2.0f;
    float lock_settle_s = 1.2f;  // start averaging after this
  };

  struct Result
  {
    float offset_rad = 0.0f;
    float sign = 1.0f;
    float residual_rad_rms = 0.0f;  // electrical
    uint16_t samples = 0;
    bool ok = false;
  };

  void Start(const Options& options)
  {
    options_ = options;
    if (options_.omega_elec_rad_s < 1.0f)
    {
      options_.omega_elec_rad_s = 1.0f;
    }
    if (options_.pole_pairs < 1.0f)
    {
      options_.pole_pairs = 1.0f;
    }
    if (options_.voltage_V < 0.2f)
    {
      options_.voltage_V = 0.2f;
    }

    enc_unwrapped_ = 0.0f;
    phase_travel_ = 0.0f;
    have_prev_ = false;
    sign_ = 1.0f;
    sum_sin_ = 0.0f;
    sum_cos_ = 0.0f;
    sum_err2_ = 0.0f;
    samples_ = 0;
    gated_samples_ = 0;
    elapsed_s_ = 0.0f;
    result_ = Result{};

    if (options_.method == Method::Lock)
    {
      state_ = State::Locking;
      omega_cmd_ = 0.0f;
      lock_sum_sin_ = 0.0f;
      lock_sum_cos_ = 0.0f;
      lock_n_ = 0;
    }
    else
    {
      state_ = State::SenseSign;
      omega_cmd_ = options_.omega_elec_rad_s;
    }
  }

  void Stop()
  {
    if (active())
    {
      state_ = State::Failed;
      result_.ok = false;
    }
    omega_cmd_ = 0.0f;
  }

  State state() const { return state_; }
  Method method() const { return options_.method; }
  bool active() const
  {
    return state_ == State::SenseSign || state_ == State::Fwd ||
           state_ == State::Rev || state_ == State::Locking;
  }
  float voltage_V() const { return options_.voltage_V; }
  float omega_cmd_elec() const { return omega_cmd_; }
  const Result& result() const { return result_; }

  uint16_t progress_permille() const
  {
    if (state_ == State::Done)
    {
      return 1000;
    }
    if (state_ == State::Failed || state_ == State::Idle)
    {
      return 0;
    }
    if (state_ == State::Locking)
    {
      float p = elapsed_s_ / options_.lock_total_s;
      if (p > 1.0f)
      {
        p = 1.0f;
      }
      return static_cast<uint16_t>(p * 1000.0f);
    }
    const float target = options_.mech_revs_each_way * math::k2Pi;
    const float travel =
        (phase_travel_ >= 0.0f) ? phase_travel_ : -phase_travel_;
    float local = (target > 1.0e-3f) ? (travel / target) : 0.0f;
    if (local > 1.0f)
    {
      local = 1.0f;
    }
    if (state_ == State::SenseSign)
    {
      return static_cast<uint16_t>(local * 100.0f);
    }
    if (state_ == State::Fwd)
    {
      return static_cast<uint16_t>(100.0f + local * 450.0f);
    }
    return static_cast<uint16_t>(550.0f + local * 450.0f);
  }

  // theta_cmd_elec: open-loop FOC angle. enc_counts_rad: raw counts angle.
  bool Step(float dt_s, float theta_cmd_elec, float enc_counts_rad)
  {
    if (!active())
    {
      return false;
    }
    if (dt_s < 1.0e-7f)
    {
      dt_s = 1.0e-7f;
    }
    elapsed_s_ += dt_s;

    if (!have_prev_)
    {
      prev_enc_ = enc_counts_rad;
      have_prev_ = true;
      return false;
    }

    const float denc = math::WrapNegPiToPi(enc_counts_rad - prev_enc_);
    prev_enc_ = enc_counts_rad;
    const float omega_enc = denc / dt_s;
    enc_unwrapped_ += denc;

    if (state_ == State::Locking)
    {
      return StepLock(dt_s, enc_counts_rad);
    }

    phase_travel_ += denc;

    if (state_ == State::SenseSign)
    {
      const float traveled =
          (enc_unwrapped_ >= 0.0f) ? enc_unwrapped_ : -enc_unwrapped_;
      if (traveled > 0.5f)
      {
        sign_ = (enc_unwrapped_ > 0.0f) ? 1.0f : -1.0f;
        state_ = State::Fwd;
        phase_travel_ = 0.0f;
        omega_cmd_ = options_.omega_elec_rad_s;
        // Discard sense motion from fit.
        sum_sin_ = 0.0f;
        sum_cos_ = 0.0f;
        sum_err2_ = 0.0f;
        samples_ = 0;
        gated_samples_ = 0;
      }
      else if (elapsed_s_ > options_.sense_timeout_s)
      {
        Fail();
        return true;
      }
      return false;
    }

    // Gate: only fit when encoder speed tracks commanded mechanical ω.
    const float omega_cmd_mech = omega_cmd_ / options_.pole_pairs;
    const float abs_cmd =
        (omega_cmd_mech >= 0.0f) ? omega_cmd_mech : -omega_cmd_mech;
    const float abs_err = omega_enc - omega_cmd_mech;
    const float abs_err_a = (abs_err >= 0.0f) ? abs_err : -abs_err;
    const bool tracking =
        abs_err_a <= (options_.track_rel * abs_cmd + options_.track_floor_rad_s);

    if (tracking)
    {
      const float enc_elec = sign_ * enc_counts_rad * options_.pole_pairs;
      const float err = math::WrapNegPiToPi(theta_cmd_elec - enc_elec);
      const math::SinCos sc = math::SinCosFromRadians(err);
      sum_sin_ += sc.s;
      sum_cos_ += sc.c;
      sum_err2_ += err * err;
      gated_samples_++;
    }
    samples_++;

    const float target = options_.mech_revs_each_way * math::k2Pi;
    const float travel_abs =
        (phase_travel_ >= 0.0f) ? phase_travel_ : -phase_travel_;

    if (state_ == State::Fwd && travel_abs >= target)
    {
      state_ = State::Rev;
      phase_travel_ = 0.0f;
      omega_cmd_ = -options_.omega_elec_rad_s;
      return false;
    }

    if (state_ == State::Rev && travel_abs >= target)
    {
      FinishSpin();
      return true;
    }
    return false;
  }

 private:
  bool StepLock(float /*dt_s*/, float enc_counts_rad)
  {
    // After settle, average encoder angle on the circle.
    if (elapsed_s_ >= options_.lock_settle_s)
    {
      const math::SinCos sc = math::SinCosFromRadians(enc_counts_rad);
      lock_sum_sin_ += sc.s;
      lock_sum_cos_ += sc.c;
      lock_n_++;
    }

    if (elapsed_s_ < options_.lock_total_s)
    {
      return false;
    }

    if (lock_n_ < 32)
    {
      Fail();
      return true;
    }

    // Locked to θ_cmd≈0 ⇒ want (sign*counts + offset)*pp ≈ 0.
    const float mean_counts =
        math::WrapZeroToTwoPi(atan2f(lock_sum_sin_, lock_sum_cos_));
    // Keep sign=+1 for lock; user can flip if torque direction is wrong.
    sign_ = 1.0f;
    result_.sign = sign_;
    result_.offset_rad = math::WrapNegPiToPi(-sign_ * mean_counts);
    result_.samples = static_cast<uint16_t>(lock_n_ > 65535 ? 65535 : lock_n_);

    // Residual proxy: spread of counts during settle → electrical.
    // Re-walk not available; use 1 - |R| concentration of circular mean.
    const float r = sqrtf(lock_sum_sin_ * lock_sum_sin_ +
                          lock_sum_cos_ * lock_sum_cos_) /
                    static_cast<float>(lock_n_);
    // For wrapped Gaussian, residual≈sqrt(2(1-R)); scale to electrical.
    float mech_rms = sqrtf(2.0f * (1.0f - r));
    if (mech_rms > math::kPi)
    {
      mech_rms = math::kPi;
    }
    result_.residual_rad_rms = mech_rms * options_.pole_pairs;
    result_.ok = true;  // finished; GUI grades quality via residual
    omega_cmd_ = 0.0f;
    state_ = State::Done;
    return true;
  }

  void FinishSpin()
  {
    omega_cmd_ = 0.0f;
    // Require enough *gated* samples and decent track ratio.
    const float ratio =
        (samples_ > 0)
            ? (static_cast<float>(gated_samples_) / static_cast<float>(samples_))
            : 0.0f;
    if (gated_samples_ < options_.min_samples || ratio < 0.25f)
    {
      Fail();
      return;
    }
    const float offset_elec =
        math::WrapNegPiToPi(atan2f(sum_sin_, sum_cos_));
    result_.sign = sign_;
    result_.offset_rad = offset_elec / options_.pole_pairs;
    result_.samples = gated_samples_;
    result_.residual_rad_rms =
        sqrtf(sum_err2_ / static_cast<float>(gated_samples_));
    result_.ok = true;
    state_ = State::Done;
  }

  void Fail()
  {
    omega_cmd_ = 0.0f;
    state_ = State::Failed;
    result_.ok = false;
    result_.samples = gated_samples_;
    if (gated_samples_ > 0)
    {
      result_.residual_rad_rms =
          sqrtf(sum_err2_ / static_cast<float>(gated_samples_));
    }
  }

  Options options_{};
  State state_ = State::Idle;
  float omega_cmd_ = 0.0f;
  float sign_ = 1.0f;
  float prev_enc_ = 0.0f;
  float enc_unwrapped_ = 0.0f;
  float phase_travel_ = 0.0f;
  bool have_prev_ = false;
  float sum_sin_ = 0.0f;
  float sum_cos_ = 0.0f;
  float sum_err2_ = 0.0f;
  uint16_t samples_ = 0;
  uint16_t gated_samples_ = 0;
  float elapsed_s_ = 0.0f;
  float lock_sum_sin_ = 0.0f;
  float lock_sum_cos_ = 0.0f;
  uint16_t lock_n_ = 0;
  Result result_{};
};

}  // namespace calibration
