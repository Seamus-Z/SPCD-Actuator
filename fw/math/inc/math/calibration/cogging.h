// Cogging-torque compensation measurement (moteus utils/compensate_cogging.py
// ported on-device). Spins the rotor at a constant velocity forward then
// reverse on top of the running velocity loop, and records the torque current
// the loop applies at each mechanical rotor position. Averaging forward and
// reverse cancels the direction-dependent terms (friction, torque-angle lag,
// viscous damping) and leaves the position-periodic cogging current, stored as
// a moteus-style int8 table + float scale for feed-forward.
//
// Phase progression is gated by MEASURED motion, not by wall-clock time:
//   settle  -> wait until the encoder actually reaches the commanded speed
//   record  -> accumulate until the rotor has traveled the requested number of
//              revolutions (so it always covers a full turn regardless of how
//              fast it ends up spinning or how long the reversal takes).
#pragma once

#include <array>
#include <cstdint>

#include "math/cogging.h"
#include "math/constants.h"

namespace math { namespace calibration
{

class CoggingCal
{
 public:
  enum class State : uint8_t
  {
    Idle = 0,
    Running = 1,
    Done = 2,
    Failed = 3,
  };

  struct Options
  {
    float velocity_mech_rad_s = 5.0f;  // constant sweep speed (mech)
    float record_revs = 4.0f;          // mechanical revs recorded per direction
    uint16_t min_count_per_bin = 2;    // per direction, before gap-fill
    float settle_timeout_s = 6.0f;     // max time to reach commanded speed
    float record_timeout_s = 30.0f;    // max time to cover the revolutions
    float speed_gate = 0.7f;           // fraction of commanded speed = "settled"
  };

  struct Result
  {
    math::CoggingTable table{};
    float scale = 0.0f;  // A per LSB (max|avg| / 127)
    float peak_A = 0.0f;
    bool ok = false;
  };

  void Start(const Options& options)
  {
    options_ = options;
    if (options_.velocity_mech_rad_s < 0.1f)
    {
      options_.velocity_mech_rad_s = 0.1f;
    }
    if (options_.record_revs < 0.5f)
    {
      options_.record_revs = 0.5f;
    }
    if (options_.min_count_per_bin < 1u)
    {
      options_.min_count_per_bin = 1u;
    }
    phase_ = Phase::FwdSettle;
    phase_elapsed_s_ = 0.0f;
    travel_ = 0.0f;
    prev_enc_ = 0.0f;
    have_prev_ = false;
    for (size_t d = 0; d < 2; ++d)
    {
      bin_sum_[d].fill(0.0f);
      bin_count_[d].fill(0u);
    }
    result_ = Result{};
    state_ = State::Running;
  }

  void Stop()
  {
    if (active())
    {
      state_ = State::Failed;
      result_.ok = false;
    }
  }

  State state() const { return state_; }
  bool active() const { return state_ == State::Running; }
  const Result& result() const { return result_; }

  // Velocity setpoint the ISR feeds to the velocity loop. Always commanded so
  // the position loop's own acceleration limit shapes the ramp.
  float velocity_cmd_mech_rad_s() const
  {
    if (!active())
    {
      return 0.0f;
    }
    const bool forward =
        phase_ == Phase::FwdSettle || phase_ == Phase::FwdRecord;
    return forward ? options_.velocity_mech_rad_s
                   : -options_.velocity_mech_rad_s;
  }

  uint16_t progress_permille() const
  {
    if (state_ == State::Done)
    {
      return 1000;
    }
    if (!active())
    {
      return 0;
    }
    const float target = options_.record_revs * math::k2Pi;
    const float frac = (target > 1.0e-6f) ? (travel_ / target) : 0.0f;
    float p = 0.0f;
    switch (phase_)
    {
      case Phase::FwdSettle:
        p = 0.0f;
        break;
      case Phase::FwdRecord:
        p = 0.5f * frac;
        break;
      case Phase::RevSettle:
        p = 0.5f;
        break;
      case Phase::RevRecord:
        p = 0.5f + 0.5f * frac;
        break;
    }
    if (p > 1.0f)
    {
      p = 1.0f;
    }
    return static_cast<uint16_t>(p * 1000.0f);
  }

  // Called each control tick while active. enc_mech_rad is the calibrated
  // mechanical rotor angle [0, 2π); iq_cmd_A is the torque current the velocity
  // loop is applying this tick. Returns true the tick it finishes.
  bool Step(float dt_s, float enc_mech_rad, float iq_cmd_A)
  {
    if (!active())
    {
      return false;
    }
    if (dt_s < 1.0e-7f)
    {
      dt_s = 1.0e-7f;
    }

    float dtheta = 0.0f;
    if (have_prev_)
    {
      dtheta = math::WrapNegPiToPi(enc_mech_rad - prev_enc_);
    }
    prev_enc_ = enc_mech_rad;
    have_prev_ = true;

    const float speed = dtheta / dt_s;
    const float abs_dtheta = (dtheta >= 0.0f) ? dtheta : -dtheta;
    phase_elapsed_s_ += dt_s;

    const float v = options_.velocity_mech_rad_s;
    const float gate = options_.speed_gate * v;
    const float target = options_.record_revs * math::k2Pi;

    switch (phase_)
    {
      case Phase::FwdSettle:
        if (speed >= gate)
        {
          phase_ = Phase::FwdRecord;
          phase_elapsed_s_ = 0.0f;
          travel_ = 0.0f;
        }
        else if (phase_elapsed_s_ >= options_.settle_timeout_s)
        {
          Fail();
          return true;
        }
        return false;

      case Phase::FwdRecord:
        Accumulate(0u, enc_mech_rad, iq_cmd_A);
        travel_ += abs_dtheta;
        if (travel_ >= target)
        {
          phase_ = Phase::RevSettle;
          phase_elapsed_s_ = 0.0f;
        }
        else if (phase_elapsed_s_ >= options_.record_timeout_s)
        {
          Fail();
          return true;
        }
        return false;

      case Phase::RevSettle:
        if (speed <= -gate)
        {
          phase_ = Phase::RevRecord;
          phase_elapsed_s_ = 0.0f;
          travel_ = 0.0f;
        }
        else if (phase_elapsed_s_ >= options_.settle_timeout_s)
        {
          Fail();
          return true;
        }
        return false;

      case Phase::RevRecord:
        Accumulate(1u, enc_mech_rad, iq_cmd_A);
        travel_ += abs_dtheta;
        if (travel_ >= target)
        {
          Finish();
          return true;
        }
        if (phase_elapsed_s_ >= options_.record_timeout_s)
        {
          Fail();
          return true;
        }
        return false;
    }
    return false;
  }

 private:
  enum class Phase : uint8_t
  {
    FwdSettle,
    FwdRecord,
    RevSettle,
    RevRecord,
  };

  static size_t BinOf(float enc_mech_rad)
  {
    const float wrapped = math::WrapZeroToTwoPi(enc_mech_rad);
    const float scaled =
        wrapped * (static_cast<float>(math::kCoggingTableSize) / math::k2Pi);
    return static_cast<size_t>(scaled) % math::kCoggingTableSize;
  }

  void Accumulate(size_t dir, float enc_mech_rad, float iq_cmd_A)
  {
    const size_t bin = BinOf(enc_mech_rad);
    if (bin_count_[dir][bin] < 0xFFFFu)
    {
      bin_sum_[dir][bin] += iq_cmd_A;
      bin_count_[dir][bin]++;
    }
  }

  void Fail()
  {
    result_.ok = false;
    state_ = State::Failed;
  }

  void Finish()
  {
    // Average fwd + rev per bin (cogging survives, direction terms cancel).
    // Store the average back into bin_sum_[0] and reuse bin_count_[0] as a
    // per-bin validity flag — a full-size local array would blow the ISR stack.
    constexpr size_t N = math::kCoggingTableSize;
    uint32_t valid_count = 0;
    for (size_t bin = 0; bin < N; ++bin)
    {
      const bool ok = bin_count_[0][bin] >= options_.min_count_per_bin &&
                      bin_count_[1][bin] >= options_.min_count_per_bin;
      float a = 0.0f;
      if (ok)
      {
        const float m0 =
            bin_sum_[0][bin] / static_cast<float>(bin_count_[0][bin]);
        const float m1 =
            bin_sum_[1][bin] / static_cast<float>(bin_count_[1][bin]);
        a = 0.5f * (m0 + m1);
        valid_count++;
      }
      bin_sum_[0][bin] = a;
      bin_count_[0][bin] = ok ? 1u : 0u;  // reuse as validity flag
    }

    // Need most bins covered; otherwise the rotor never completed a clean turn
    // in both directions (too fast for the bin width, or it stalled).
    if (valid_count < (3u * N) / 4u)
    {
      Fail();
      return;
    }

    // Fill the few undersampled bins from their nearest valid neighbour.
    // Immediate marking makes a run fill from its leading edge in one pass;
    // a handful of passes also covers wrap-around at the array ends.
    for (int pass = 0; pass < 4 && valid_count < N; ++pass)
    {
      for (size_t bin = 0; bin < N; ++bin)
      {
        if (bin_count_[0][bin] != 0u)
        {
          continue;
        }
        const size_t left = (bin + N - 1u) % N;
        const size_t right = (bin + 1u) % N;
        if (bin_count_[0][left] != 0u)
        {
          bin_sum_[0][bin] = bin_sum_[0][left];
          bin_count_[0][bin] = 1u;
          valid_count++;
        }
        else if (bin_count_[0][right] != 0u)
        {
          bin_sum_[0][bin] = bin_sum_[0][right];
          bin_count_[0][bin] = 1u;
          valid_count++;
        }
      }
    }
    if (valid_count < N)
    {
      Fail();
      return;
    }

    float peak = 0.0f;
    for (size_t bin = 0; bin < N; ++bin)
    {
      const float a = bin_sum_[0][bin];
      const float abs_a = (a >= 0.0f) ? a : -a;
      if (abs_a > peak)
      {
        peak = abs_a;
      }
    }

    result_.peak_A = peak;
    if (peak < 1.0e-4f)
    {
      // No measurable cogging: valid result, empty (zero) table.
      result_.table.fill(0);
      result_.scale = 0.0f;
      result_.ok = true;
      state_ = State::Done;
      return;
    }

    const float scale = peak / 127.0f;
    const float inv_scale = 1.0f / scale;
    for (size_t bin = 0; bin < N; ++bin)
    {
      float q = bin_sum_[0][bin] * inv_scale;
      q = (q >= 0.0f) ? (q + 0.5f) : (q - 0.5f);  // round to nearest
      int v = static_cast<int>(q);
      if (v > 127)
      {
        v = 127;
      }
      if (v < -127)
      {
        v = -127;
      }
      result_.table[bin] = static_cast<int8_t>(v);
    }
    result_.scale = scale;
    result_.ok = true;
    state_ = State::Done;
  }

  Options options_{};
  State state_ = State::Idle;
  Phase phase_ = Phase::FwdSettle;
  float phase_elapsed_s_ = 0.0f;
  float travel_ = 0.0f;
  float prev_enc_ = 0.0f;
  bool have_prev_ = false;
  std::array<float, math::kCoggingTableSize> bin_sum_[2]{};
  std::array<uint16_t, math::kCoggingTableSize> bin_count_[2]{};
  Result result_{};
};

}  // namespace math { namespace calibration
}  // namespace math
