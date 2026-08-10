// SimplePI (moteus-compatible polarity, no mjlib).
// error = measured - desired; command = -(kp*error + integral).
#pragma once

namespace math { namespace foc
{

inline float Limit(float a, float lo, float hi)
{
  if (a < lo)
  {
    return lo;
  }
  if (a > hi)
  {
    return hi;
  }
  return a;
}

// Freestanding-safe NaN / isfinite (avoid <cmath>/<limits>).
inline float QuietNan()
{
  union
  {
    unsigned u;
    float f;
  } v{0x7fc00000u};
  return v.f;
}

inline bool IsFinite(float x) { return __builtin_isfinite(x) != 0; }

class SimplePI
{
 public:
  struct Config
  {
    float kp = 0.0f;
    float ki = 0.0f;
    float max_desired_rate = 0.0f;  // 0 = unlimited
  };

  struct State
  {
    float integral = 0.0f;
    float desired = QuietNan();
    float error = 0.0f;
    float p = 0.0f;
    float command = 0.0f;

    void Clear()
    {
      integral = 0.0f;
      desired = QuietNan();
      error = 0.0f;
      p = 0.0f;
      command = 0.0f;
    }
  };

  SimplePI(const Config* config, State* state) : config_(config), state_(state) {}

  float Apply(float measured, float input_desired, float period_s)
  {
    float desired = input_desired;
    if (config_->max_desired_rate != 0.0f && IsFinite(state_->desired))
    {
      const float max_step = config_->max_desired_rate * period_s;
      desired = state_->desired +
                Limit(input_desired - state_->desired, -max_step, max_step);
    }

    state_->desired = desired;
    state_->error = measured - desired;

    state_->integral += state_->error * config_->ki * period_s;
    state_->p = config_->kp * state_->error;
    state_->command = -1.0f * (state_->p + state_->integral);
    return state_->command;
  }

  void SetIntegralForOutput(float target_command)
  {
    state_->integral = -target_command - config_->kp * state_->error;
  }

 private:
  const Config* const config_;
  State* const state_;
};

}  // namespace foc
}  // namespace math
