// Encoder electrical-phase calibration.
// - Spin: moteus-like open-loop both ways (samples gated on speed match)
// - Lock: hold fixed Vd (ω=0), rotor aligns to d-axis → offset (more robust)
#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

extern "C" {
float atan2f(float, float);
float sqrtf(float);
}

#include "math/constants.h"
#include "math/commutation.h"
#include "math/foc/transform.h"

namespace math { namespace calibration
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
    // Regulated D-axis alignment current for lock and spin calibration.
    float current_A = 1.0f;
    float omega_elec_rad_s = 40.0f;  // |ω| for Spin
    // Slew starts and reversals so the rotor remains synchronized.
    float omega_accel_elec_rad_s2 = 200.0f;
    float pole_pairs = 14.0f;
    float mech_revs_each_way = 1.0f;
    uint16_t min_samples = 200;
    float sense_timeout_s = 4.0f;
    float align_settle_s = 0.5f;
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
    // Scatter after removing the opposite constant phase lags of each sweep.
    float residual_rad_rms = 0.0f;
    math::CommutationTable commutation_offset_rad{};
    uint16_t samples = 0;
    bool commutation_valid = false;
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
    if (options_.current_A < 0.1f)
    {
      options_.current_A = 0.1f;
    }

    enc_unwrapped_ = 0.0f;
    phase_travel_ = 0.0f;
    have_prev_ = false;
    sign_ = 1.0f;
    sum_err2_ = 0.0f;
    samples_ = 0;
    gated_samples_ = 0;
    direction_sum_sin_.fill(0.0f);
    direction_sum_cos_.fill(0.0f);
    direction_samples_.fill(0u);
    for (size_t direction = 0; direction < 2; ++direction)
    {
      bin_sum_sin_[direction].fill(0.0f);
      bin_sum_cos_[direction].fill(0.0f);
      bin_samples_[direction].fill(0u);
    }
    elapsed_s_ = 0.0f;
    omega_cmd_ = 0.0f;
    omega_target_ = 0.0f;
    result_ = Result{};

    if (options_.method == Method::Lock)
    {
      state_ = State::Locking;
      lock_sum_sin_ = 0.0f;
      lock_sum_cos_ = 0.0f;
      lock_n_ = 0;
    }
    else
    {
      state_ = State::SenseSign;
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
    omega_target_ = 0.0f;
  }

  State state() const { return state_; }
  Method method() const { return options_.method; }
  bool active() const
  {
    return state_ == State::SenseSign || state_ == State::Fwd ||
           state_ == State::Rev || state_ == State::Locking;
  }
  float current_A() const { return options_.current_A; }
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
    if (state_ == State::SenseSign &&
        elapsed_s_ >= options_.align_settle_s)
    {
      omega_target_ = options_.omega_elec_rad_s;
    }
    UpdateOmegaCommand(dt_s);

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
        // Discard sense motion from fit.
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
    // Raw encoder velocity is opposite the field when sign_ == -1; compare in
    // the command frame or almost no samples gate and the midpoint biases one
    // direction (exactly the "+ works / - dies, then flips after recal" bug).
    const float omega_cmd_mech = omega_cmd_ / options_.pole_pairs;
    const float abs_cmd =
        (omega_cmd_mech >= 0.0f) ? omega_cmd_mech : -omega_cmd_mech;
    const float omega_enc_cmd_frame = sign_ * omega_enc;
    const float abs_err = omega_enc_cmd_frame - omega_cmd_mech;
    const float abs_err_a = (abs_err >= 0.0f) ? abs_err : -abs_err;
    const float target_error = omega_cmd_ - omega_target_;
    const float target_error_abs =
        (target_error >= 0.0f) ? target_error : -target_error;
    const bool at_sweep_speed = target_error_abs < 1.0e-3f;
    const bool tracking =
        at_sweep_speed &&
        abs_err_a <= (options_.track_rel * abs_cmd +
                      options_.track_floor_rad_s);
    if (tracking)
    {
      const float enc_elec = sign_ * enc_counts_rad * options_.pole_pairs;
      const float err = math::WrapNegPiToPi(theta_cmd_elec - enc_elec);
      const math::SinCos sc = math::SinCosFromRadians(err);
      sum_err2_ += err * err;
      const size_t direction = state_ == State::Fwd ? 0u : 1u;
      direction_sum_sin_[direction] += sc.s;
      direction_sum_cos_[direction] += sc.c;
      direction_samples_[direction]++;
      const float scaled =
          math::WrapZeroToTwoPi(enc_counts_rad) *
          (static_cast<float>(math::kCommutationTableSize) / math::k2Pi);
      const size_t bin =
          static_cast<size_t>(scaled) % math::kCommutationTableSize;
      bin_sum_sin_[direction][bin] += sc.s;
      bin_sum_cos_[direction][bin] += sc.c;
      bin_samples_[direction][bin]++;
      gated_samples_++;
    }
    samples_++;

    const float target = options_.mech_revs_each_way * math::k2Pi;
    const float signed_travel = sign_ * phase_travel_;

    if (state_ == State::Fwd && signed_travel >= target)
    {
      state_ = State::Rev;
      phase_travel_ = 0.0f;
      omega_target_ = -options_.omega_elec_rad_s;
      return false;
    }

    if (state_ == State::Rev && signed_travel <= -target)
    {
      FinishSpin();
      return true;
    }
    return false;
  }

 private:
  void UpdateOmegaCommand(float dt_s)
  {
    const float accel = options_.omega_accel_elec_rad_s2;
    if (accel <= 0.0f)
    {
      omega_cmd_ = omega_target_;
      return;
    }
    const float max_step = accel * dt_s;
    const float delta = omega_target_ - omega_cmd_;
    if (delta > max_step)
    {
      omega_cmd_ += max_step;
    }
    else if (delta < -max_step)
    {
      omega_cmd_ -= max_step;
    }
    else
    {
      omega_cmd_ = omega_target_;
    }
  }

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
    FinishWithQuality(false);
    return true;
  }

  void FinishSpin()
  {
    omega_cmd_ = 0.0f;
    const float ratio =
        (samples_ > 0)
            ? (static_cast<float>(gated_samples_) / static_cast<float>(samples_))
            : 0.0f;
    if (gated_samples_ < options_.min_samples || ratio < 0.25f ||
        direction_samples_[0] == 0u || direction_samples_[1] == 0u)
    {
      Fail();
      return;
    }

    constexpr uint32_t kMinSamplesPerDirectionBin = 2;
    for (size_t direction = 0; direction < 2; ++direction)
    {
      for (size_t bin = 0; bin < math::kCommutationTableSize; ++bin)
      {
        if (bin_samples_[direction][bin] < kMinSamplesPerDirectionBin)
        {
          Fail();
          return;
        }
      }
    }

    const float direction_mean[2] = {
        math::WrapNegPiToPi(
            atan2f(direction_sum_sin_[0], direction_sum_cos_[0])),
        math::WrapNegPiToPi(
            atan2f(direction_sum_sin_[1], direction_sum_cos_[1])),
    };
    // Open-loop rotation produces opposite, nearly constant torque-angle
    // lags in each direction. Their circular midpoint is the zero-torque
    // electrical offset; retaining either lag biases commutation.
    const float offset_elec =
        CircularMidpoint(direction_mean[0], direction_mean[1]);
    // Half the fwd/rev mean gap is the open-loop torque lag we removed. Large
    // values are OK; unequal coverage still leaves a static bias in offset_elec.
    const float direction_lag_elec =
        0.5f * math::WrapNegPiToPi(direction_mean[0] - direction_mean[1]);
    (void)direction_lag_elec;

    float residual_sum = 0.0f;
    for (size_t bin = 0; bin < math::kCommutationTableSize; ++bin)
    {
      float normalized[2]{};
      for (size_t direction = 0; direction < 2; ++direction)
      {
        const float mean_err = math::WrapNegPiToPi(
            atan2f(bin_sum_sin_[direction][bin],
                   bin_sum_cos_[direction][bin]));
        normalized[direction] =
            math::WrapNegPiToPi(mean_err - direction_mean[direction]);
        const float n = static_cast<float>(bin_samples_[direction][bin]);
        float concentration =
            sqrtf(bin_sum_sin_[direction][bin] *
                      bin_sum_sin_[direction][bin] +
                  bin_sum_cos_[direction][bin] *
                      bin_sum_cos_[direction][bin]) /
            n;
        if (concentration > 1.0f)
        {
          concentration = 1.0f;
        }
        residual_sum += 2.0f * (1.0f - concentration) * n;
      }

      result_.commutation_offset_rad[bin] =
          CircularMidpoint(normalized[0], normalized[1]);
      const float mismatch =
          math::WrapNegPiToPi(normalized[0] - normalized[1]);
      const float bin_total = static_cast<float>(
          bin_samples_[0][bin] + bin_samples_[1][bin]);
      residual_sum += 0.25f * mismatch * mismatch * bin_total;
    }

    result_.sign = sign_;
    result_.offset_rad = offset_elec / options_.pole_pairs;
    result_.samples = static_cast<uint16_t>(
        gated_samples_ > 65535u ? 65535u : gated_samples_);
    result_.residual_rad_rms =
        sqrtf(residual_sum / static_cast<float>(gated_samples_));
    FinishWithQuality(true);
  }

  void FinishWithQuality(bool commutation_valid)
  {
    const bool quality_ok =
        result_.residual_rad_rms >= 0.0f &&
        result_.residual_rad_rms <= math::kMaxCommutationResidualRad;
    result_.commutation_valid = commutation_valid && quality_ok;
    result_.ok = quality_ok;
    omega_cmd_ = 0.0f;
    omega_target_ = 0.0f;
    state_ = quality_ok ? State::Done : State::Failed;
  }
  void Fail()
  {
    omega_cmd_ = 0.0f;
    omega_target_ = 0.0f;
    result_.ok = false;
    state_ = State::Failed;
    result_.samples = static_cast<uint16_t>(
        gated_samples_ > 65535u ? 65535u : gated_samples_);
    if (gated_samples_ > 0)
    {
      result_.residual_rad_rms =
          sqrtf(sum_err2_ / static_cast<float>(gated_samples_));
    }
  }

  static float CircularMidpoint(float a, float b)
  {
    return math::WrapNegPiToPi(
        a + 0.5f * math::WrapNegPiToPi(b - a));
  }

  Options options_{};
  State state_ = State::Idle;
  float omega_cmd_ = 0.0f;
  float omega_target_ = 0.0f;
  float sign_ = 1.0f;
  float prev_enc_ = 0.0f;
  float enc_unwrapped_ = 0.0f;
  float phase_travel_ = 0.0f;
  bool have_prev_ = false;
  float sum_err2_ = 0.0f;
  uint32_t samples_ = 0;
  uint32_t gated_samples_ = 0;
  std::array<float, 2> direction_sum_sin_{};
  std::array<float, 2> direction_sum_cos_{};
  std::array<uint32_t, 2> direction_samples_{};
  std::array<std::array<float, math::kCommutationTableSize>, 2> bin_sum_sin_{};
  std::array<std::array<float, math::kCommutationTableSize>, 2> bin_sum_cos_{};
  std::array<std::array<uint32_t, math::kCommutationTableSize>, 2> bin_samples_{};
  float elapsed_s_ = 0.0f;
  float lock_sum_sin_ = 0.0f;
  float lock_sum_cos_ = 0.0f;
  uint16_t lock_n_ = 0;
  Result result_{};
};

}  // namespace math { namespace calibration
}  // namespace math
