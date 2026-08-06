// Phase resistance (R) identification.
// Closed-loop Id sweep at a fixed electrical angle (theta=0): a nonzero Id
// pulls the rotor into d-axis alignment via reluctance/magnet attraction,
// same physics as EncoderPhaseCal::Lock, so this needs no encoder. At
// steady state (Iq=0, omega=0) there is no BEMF and no cross-coupling:
// Vd = R*Id. Sweeps several Id levels and regresses through the origin.
#pragma once

#include <cstdint>

namespace calibration
{

class RIdentCal
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
    float max_current_A = 2.0f;    // top of the Id sweep
    uint8_t n_points = 5;          // evenly spaced: max/n .. max
    float settle_s = 0.5f;         // per-point settle before sampling
    float sample_s = 0.3f;         // per-point averaging window
    float current_tol_A = 0.15f;   // reject samples off the target current
  };

  struct Result
  {
    float resistance_ohm = 0.0f;
    float r2 = 0.0f;
    uint8_t points_used = 0;
    bool ok = false;
  };

  static constexpr uint8_t kMaxPoints = 8;

  void Start(const Options& options)
  {
    options_ = options;
    if (options_.n_points < 3)
    {
      options_.n_points = 3;
    }
    if (options_.n_points > kMaxPoints)
    {
      options_.n_points = kMaxPoints;
    }
    if (options_.max_current_A < 0.05f)
    {
      options_.max_current_A = 0.05f;
    }
    if (options_.settle_s < 0.05f)
    {
      options_.settle_s = 0.05f;
    }
    if (options_.sample_s < 0.05f)
    {
      options_.sample_s = 0.05f;
    }
    point_idx_ = 0;
    point_elapsed_s_ = 0.0f;
    ResetPointAccum();
    sxy_ = 0.0f;
    sxx_ = 0.0f;
    syy_ = 0.0f;
    points_used_ = 0;
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

  // Current Id setpoint for the host loop to command; 0 when idle.
  float id_cmd_A() const
  {
    if (state_ != State::Running)
    {
      return 0.0f;
    }
    return TargetCurrent(point_idx_);
  }

  uint16_t progress_permille() const
  {
    if (state_ == State::Done)
    {
      return 1000;
    }
    if (state_ != State::Running)
    {
      return 0;
    }
    const float per_point = options_.settle_s + options_.sample_s;
    const float done =
        static_cast<float>(point_idx_) * per_point + point_elapsed_s_;
    const float total = static_cast<float>(options_.n_points) * per_point;
    float p = (total > 1.0e-6f) ? (done / total) : 0.0f;
    if (p > 1.0f)
    {
      p = 1.0f;
    }
    return static_cast<uint16_t>(p * 1000.0f);
  }

  // Called every control tick while active with the live plant signals.
  bool Step(float dt_s, float id_meas_A, float vd_V)
  {
    if (!active())
    {
      return false;
    }
    if (dt_s < 1.0e-7f)
    {
      dt_s = 1.0e-7f;
    }
    point_elapsed_s_ += dt_s;

    if (point_elapsed_s_ >= options_.settle_s)
    {
      const float target = TargetCurrent(point_idx_);
      const float err = id_meas_A - target;
      const float abs_err = (err >= 0.0f) ? err : -err;
      if (abs_err <= options_.current_tol_A)
      {
        pt_sum_id_ += id_meas_A;
        pt_sum_vd_ += vd_V;
        pt_n_++;
      }
    }

    if (point_elapsed_s_ < options_.settle_s + options_.sample_s)
    {
      return false;
    }

    FinishPoint();
    point_idx_++;
    point_elapsed_s_ = 0.0f;
    ResetPointAccum();

    if (point_idx_ >= options_.n_points)
    {
      Finish();
      return true;
    }
    return false;
  }

 private:
  float TargetCurrent(uint8_t point_idx) const
  {
    const float n = static_cast<float>(options_.n_points);
    const float i1 = static_cast<float>(point_idx + 1);
    return options_.max_current_A * (i1 / n);
  }

  void ResetPointAccum()
  {
    pt_sum_id_ = 0.0f;
    pt_sum_vd_ = 0.0f;
    pt_n_ = 0;
  }

  void FinishPoint()
  {
    constexpr uint16_t kMinSamplesPerPoint = 20;
    if (pt_n_ < kMinSamplesPerPoint)
    {
      return;  // dropped: never tracked the target current well enough
    }
    const float n = static_cast<float>(pt_n_);
    const float id_avg = pt_sum_id_ / n;
    const float vd_avg = pt_sum_vd_ / n;
    sxy_ += id_avg * vd_avg;
    sxx_ += id_avg * id_avg;
    syy_ += vd_avg * vd_avg;
    points_used_++;
  }

  void Finish()
  {
    if (points_used_ < 3 || sxx_ < 1.0e-6f)
    {
      result_.ok = false;
      result_.points_used = points_used_;
      state_ = State::Failed;
      return;
    }
    const float r = sxy_ / sxx_;
    float r2 = (syy_ > 1.0e-9f) ? (sxy_ * sxy_) / (sxx_ * syy_) : 0.0f;
    if (r2 < 0.0f)
    {
      r2 = 0.0f;
    }
    if (r2 > 1.0f)
    {
      r2 = 1.0f;
    }
    result_.resistance_ohm = r;
    result_.r2 = r2;
    result_.points_used = points_used_;
    result_.ok = true;
    state_ = State::Done;
  }

  Options options_{};
  State state_ = State::Idle;
  uint8_t point_idx_ = 0;
  float point_elapsed_s_ = 0.0f;
  float pt_sum_id_ = 0.0f;
  float pt_sum_vd_ = 0.0f;
  uint16_t pt_n_ = 0;
  float sxy_ = 0.0f;
  float sxx_ = 0.0f;
  float syy_ = 0.0f;
  uint8_t points_used_ = 0;
  Result result_{};
};

}  // namespace calibration
