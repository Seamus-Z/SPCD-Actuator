// Back-EMF constant (Ke) identification.
// Closed-loop velocity sweep: Id=0, hold N steady mech-speed points, and at
// each point regress the current loop's own Vq/Iq feedforward-corrected
// output against speed. Steady state (no dI/dt, cross term drops with
// Id=0): Vq = R*Iq + Ke*omega_mech -> Ke = (Vq - R*Iq) / omega_mech.
// Runs on top of an already-working velocity loop (position_loop/
// current_loop); this class only supplies the speed setpoint schedule and
// does the least-squares fit — it does not touch PWM/FOC itself.
#pragma once

#include <cstdint>

namespace calibration
{

class BemfIdentCal
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
    float resistance_ohm = 0.0f;     // known-good R, for IR-drop correction
    float max_speed_rad_s = 60.0f;   // mech rad/s, top of the sweep
    uint8_t n_points = 5;            // evenly spaced: max/n .. max
    float settle_s = 0.6f;           // per-point ramp/settle before sampling
    float sample_s = 0.4f;           // per-point averaging window
    float speed_tol_rad_s = 1.5f;    // reject samples off the target speed
  };

  struct Result
  {
    float ke_v_s_per_rad = 0.0f;
    float r2 = 0.0f;          // fit quality, 0..1 (through-origin R^2)
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
    if (options_.max_speed_rad_s < 1.0f)
    {
      options_.max_speed_rad_s = 1.0f;
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

  // Current velocity setpoint for the host loop to command; 0 when idle.
  float velocity_cmd_mech_rad_s() const
  {
    if (state_ != State::Running)
    {
      return 0.0f;
    }
    return TargetSpeed(point_idx_);
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
    const float done = static_cast<float>(point_idx_) * per_point +
                       point_elapsed_s_;
    const float total = static_cast<float>(options_.n_points) * per_point;
    float p = (total > 1.0e-6f) ? (done / total) : 0.0f;
    if (p > 1.0f)
    {
      p = 1.0f;
    }
    return static_cast<uint16_t>(p * 1000.0f);
  }

  // Called every control tick while active with the live plant signals.
  // Returns true the tick the identification finishes (Done or Failed).
  bool Step(float dt_s, float omega_meas_mech_rad_s, float vq_V, float iq_A)
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
      const float target = TargetSpeed(point_idx_);
      const float err = omega_meas_mech_rad_s - target;
      const float abs_err = (err >= 0.0f) ? err : -err;
      if (abs_err <= options_.speed_tol_rad_s)
      {
        pt_sum_omega_ += omega_meas_mech_rad_s;
        pt_sum_vq_ += vq_V;
        pt_sum_iq_ += iq_A;
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
  float TargetSpeed(uint8_t point_idx) const
  {
    const float n = static_cast<float>(options_.n_points);
    const float i1 = static_cast<float>(point_idx + 1);
    return options_.max_speed_rad_s * (i1 / n);
  }

  void ResetPointAccum()
  {
    pt_sum_omega_ = 0.0f;
    pt_sum_vq_ = 0.0f;
    pt_sum_iq_ = 0.0f;
    pt_n_ = 0;
  }

  void FinishPoint()
  {
    constexpr uint16_t kMinSamplesPerPoint = 20;
    if (pt_n_ < kMinSamplesPerPoint)
    {
      return;  // dropped: never tracked the target speed well enough
    }
    const float n = static_cast<float>(pt_n_);
    const float omega_avg = pt_sum_omega_ / n;
    const float vq_avg = pt_sum_vq_ / n;
    const float iq_avg = pt_sum_iq_ / n;
    const float y = vq_avg - options_.resistance_ohm * iq_avg;
    sxy_ += omega_avg * y;
    sxx_ += omega_avg * omega_avg;
    syy_ += y * y;
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
    const float ke = sxy_ / sxx_;
    float r2 = (syy_ > 1.0e-9f) ? (sxy_ * sxy_) / (sxx_ * syy_) : 0.0f;
    if (r2 < 0.0f)
    {
      r2 = 0.0f;
    }
    if (r2 > 1.0f)
    {
      r2 = 1.0f;
    }
    result_.ke_v_s_per_rad = ke;
    result_.r2 = r2;
    result_.points_used = points_used_;
    result_.ok = true;
    state_ = State::Done;
  }

  Options options_{};
  State state_ = State::Idle;
  uint8_t point_idx_ = 0;
  float point_elapsed_s_ = 0.0f;
  float pt_sum_omega_ = 0.0f;
  float pt_sum_vq_ = 0.0f;
  float pt_sum_iq_ = 0.0f;
  uint16_t pt_n_ = 0;
  float sxy_ = 0.0f;
  float sxx_ = 0.0f;
  float syy_ = 0.0f;
  uint8_t points_used_ = 0;
  Result result_{};
};

}  // namespace calibration
