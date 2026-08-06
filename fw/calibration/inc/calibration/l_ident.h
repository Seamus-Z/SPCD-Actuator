// D/Q phase inductance identification from locked-rotor voltage steps.
#pragma once

#include <array>
#include <cstdint>

extern "C" {
float sqrtf(float);
}

namespace calibration
{

class LIdentCal
{
 public:
  enum class State : uint8_t
  {
    Idle = 0,
    Running = 1,
    Done = 2,
    Failed = 3,
  };

  enum class Axis : uint8_t
  {
    D = 0,
    Q = 1,
  };

  struct Options
  {
    float resistance_ohm = 0.5f;
    float hold_voltage_V = 1.0f;   // continuously locks the rotor on d-axis
    float step_voltage_V = 3.0f;
    uint8_t n_trials = 5;
    float hold_s = 0.4f;
    float step_timeout_s = 0.01f;
    float cooldown_s = 0.15f;
  };

  struct Result
  {
    float inductance_d_H = 0.0f;
    float inductance_q_H = 0.0f;
    float tau_d_s = 0.0f;
    float tau_q_s = 0.0f;
    float tau_std_d_s = 0.0f;
    float tau_std_q_s = 0.0f;
    uint8_t trials_d_used = 0;
    uint8_t trials_q_used = 0;
    bool ok = false;
  };

  static constexpr uint8_t kMaxTrials = 8;

  void Start(const Options& options)
  {
    options_ = options;
    if (options_.resistance_ohm < 0.01f)
    {
      options_.resistance_ohm = 0.01f;
    }
    if (options_.n_trials < 3)
    {
      options_.n_trials = 3;
    }
    if (options_.n_trials > kMaxTrials)
    {
      options_.n_trials = kMaxTrials;
    }
    if (options_.step_voltage_V < 0.1f)
    {
      options_.step_voltage_V = 0.1f;
    }
    if (options_.hold_s < 0.05f)
    {
      options_.hold_s = 0.05f;
    }
    if (options_.step_timeout_s < 0.002f)
    {
      options_.step_timeout_s = 0.002f;
    }
    if (options_.cooldown_s < 0.02f)
    {
      options_.cooldown_s = 0.02f;
    }
    axis_ = Axis::D;
    phase_ = Phase::InitialHold;
    phase_elapsed_s_ = 0.0f;
    elapsed_s_ = 0.0f;
    trial_idx_ = 0;
    step_current0_ = 0.0f;
    step_target_ = 0.0f;
    sum_tau_.fill(0.0f);
    sum_tau2_.fill(0.0f);
    valid_trials_.fill(0u);
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
  Axis axis() const { return axis_; }
  bool active() const { return state_ == State::Running; }
  const Result& result() const { return result_; }

  float voltage_d_cmd_V() const
  {
    if (!active())
    {
      return 0.0f;
    }
    return options_.hold_voltage_V +
           ((axis_ == Axis::D && phase_ == Phase::Step)
                ? options_.step_voltage_V
                : 0.0f);
  }

  float voltage_q_cmd_V() const
  {
    if (!active())
    {
      return 0.0f;
    }
    return (axis_ == Axis::Q && phase_ == Phase::Step)
               ? options_.step_voltage_V
               : 0.0f;
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
    const float per_axis = options_.hold_s +
                           static_cast<float>(options_.n_trials) *
                               (options_.step_timeout_s + options_.cooldown_s);
    const float axis_base = (axis_ == Axis::Q) ? per_axis : 0.0f;
    const float local = static_cast<float>(trial_idx_) *
                            (options_.step_timeout_s + options_.cooldown_s) +
                        phase_elapsed_s_;
    float progress = (axis_base + local) / (2.0f * per_axis);
    if (progress > 1.0f)
    {
      progress = 1.0f;
    }
    return static_cast<uint16_t>(progress * 1000.0f);
  }

  bool Step(float dt_s, float id_meas_A, float iq_meas_A)
  {
    if (!active())
    {
      return false;
    }
    if (dt_s < 1.0e-7f)
    {
      dt_s = 1.0e-7f;
    }
    phase_elapsed_s_ += dt_s;
    elapsed_s_ += dt_s;

    if (phase_ == Phase::InitialHold)
    {
      if (phase_elapsed_s_ >= options_.hold_s)
      {
        BeginStep(id_meas_A, iq_meas_A);
      }
      return false;
    }

    if (phase_ == Phase::Step)
    {
      const float measured = (axis_ == Axis::D) ? id_meas_A : iq_meas_A;
      if (measured >= step_target_)
      {
        RecordTrial(phase_elapsed_s_, true);
        phase_ = Phase::Cooldown;
        phase_elapsed_s_ = 0.0f;
      }
      else if (phase_elapsed_s_ >= options_.step_timeout_s)
      {
        RecordTrial(phase_elapsed_s_, false);
        phase_ = Phase::Cooldown;
        phase_elapsed_s_ = 0.0f;
      }
      return false;
    }

    if (phase_elapsed_s_ < options_.cooldown_s)
    {
      return false;
    }

    trial_idx_++;
    if (trial_idx_ < options_.n_trials)
    {
      BeginStep(id_meas_A, iq_meas_A);
      return false;
    }

    if (axis_ == Axis::D)
    {
      axis_ = Axis::Q;
      trial_idx_ = 0;
      phase_ = Phase::InitialHold;
      phase_elapsed_s_ = 0.0f;
      return false;
    }

    Finish();
    return true;
  }

 private:
  enum class Phase : uint8_t
  {
    InitialHold,
    Step,
    Cooldown,
  };

  static size_t AxisIndex(Axis axis)
  {
    return axis == Axis::D ? 0u : 1u;
  }

  void BeginStep(float id_now_A, float iq_now_A)
  {
    phase_ = Phase::Step;
    phase_elapsed_s_ = 0.0f;
    step_current0_ = (axis_ == Axis::D) ? id_now_A : iq_now_A;
    step_target_ = step_current0_ +
                   0.632f * (options_.step_voltage_V /
                             options_.resistance_ohm);
  }

  void RecordTrial(float tau_s, bool valid)
  {
    if (!valid)
    {
      return;
    }
    const size_t index = AxisIndex(axis_);
    sum_tau_[index] += tau_s;
    sum_tau2_[index] += tau_s * tau_s;
    valid_trials_[index]++;
  }

  void Finish()
  {
    constexpr uint8_t kMinValidTrials = 2;
    if (valid_trials_[0] < kMinValidTrials ||
        valid_trials_[1] < kMinValidTrials)
    {
      result_.trials_d_used = valid_trials_[0];
      result_.trials_q_used = valid_trials_[1];
      result_.ok = false;
      state_ = State::Failed;
      return;
    }

    const float nd = static_cast<float>(valid_trials_[0]);
    const float nq = static_cast<float>(valid_trials_[1]);
    result_.tau_d_s = sum_tau_[0] / nd;
    result_.tau_q_s = sum_tau_[1] / nq;
    float var_d = sum_tau2_[0] / nd - result_.tau_d_s * result_.tau_d_s;
    float var_q = sum_tau2_[1] / nq - result_.tau_q_s * result_.tau_q_s;
    if (var_d < 0.0f)
    {
      var_d = 0.0f;
    }
    if (var_q < 0.0f)
    {
      var_q = 0.0f;
    }
    result_.tau_std_d_s = sqrtf(var_d);
    result_.tau_std_q_s = sqrtf(var_q);
    result_.inductance_d_H = options_.resistance_ohm * result_.tau_d_s;
    result_.inductance_q_H = options_.resistance_ohm * result_.tau_q_s;
    result_.trials_d_used = valid_trials_[0];
    result_.trials_q_used = valid_trials_[1];
    result_.ok = true;
    state_ = State::Done;
  }

  Options options_{};
  State state_ = State::Idle;
  Axis axis_ = Axis::D;
  Phase phase_ = Phase::InitialHold;
  float phase_elapsed_s_ = 0.0f;
  float elapsed_s_ = 0.0f;
  uint8_t trial_idx_ = 0;
  float step_current0_ = 0.0f;
  float step_target_ = 0.0f;
  std::array<float, 2> sum_tau_{};
  std::array<float, 2> sum_tau2_{};
  std::array<uint8_t, 2> valid_trials_{};
  Result result_{};
};

}  // namespace calibration
