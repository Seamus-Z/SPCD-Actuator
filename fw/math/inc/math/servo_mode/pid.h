// Position-style PID (moteus-compatible, no mjlib).
#pragma once

#include <cstdint>

#include "math/foc/pi.h"

namespace math { namespace servo_mode
{
using foc::IsFinite;
using foc::Limit;
using foc::QuietNan;


class PID
{
 public:
  struct Config
  {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float iratelimit = -1.0f;
    float ilimit = 0.0f;
    float max_desired_rate = 0.0f;
    int8_t sign = 1;
  };

  struct State
  {
    float integral = 0.0f;
    float desired = QuietNan();
    float error = 0.0f;
    float error_rate = 0.0f;
    float p = 0.0f;
    float d = 0.0f;
    float pd = 0.0f;
    float command = 0.0f;

    void Clear()
    {
      integral = 0.0f;
      desired = QuietNan();
      error = 0.0f;
      error_rate = 0.0f;
      p = 0.0f;
      d = 0.0f;
      pd = 0.0f;
      command = 0.0f;
    }
  };

  struct ApplyOptions
  {
    float kp_scale = 1.0f;
    float kd_scale = 1.0f;
    float ilimit_scale = 1.0f;
  };

  PID(const Config* config, State* state) : config_(config), state_(state) {}

  float Apply(float measured, float input_desired, float measured_rate,
              float input_desired_rate, float period_s)
  {
    return Apply(measured, input_desired, measured_rate, input_desired_rate,
                 period_s, ApplyOptions{});
  }

  float Apply(float measured, float input_desired, float measured_rate,
              float input_desired_rate, float period_s,
              ApplyOptions apply_options)
  {
    float desired = input_desired;
    float desired_rate = input_desired_rate;
    if (config_->max_desired_rate != 0.0f && IsFinite(state_->desired))
    {
      const float max_step = config_->max_desired_rate * period_s;
      desired = state_->desired +
                Limit(input_desired - state_->desired, -max_step, max_step);
      desired_rate = Limit(input_desired_rate, -config_->max_desired_rate,
                           config_->max_desired_rate);
    }

    state_->desired = desired;
    state_->error = measured - desired;
    state_->error_rate = measured_rate - desired_rate;

    float to_update_i = state_->error * config_->ki * period_s;
    if (config_->iratelimit > 0.0f)
    {
      const float max_i_update = config_->iratelimit * period_s;
      to_update_i = Limit(to_update_i, -max_i_update, max_i_update);
    }
    state_->integral += to_update_i;

    const float ilimit = config_->ilimit * apply_options.ilimit_scale;
    state_->integral = Limit(state_->integral, -ilimit, ilimit);

    state_->p = apply_options.kp_scale * config_->kp * state_->error;
    state_->d = apply_options.kd_scale * config_->kd * state_->error_rate;
    state_->pd = state_->p + state_->d;
    state_->command =
        static_cast<float>(config_->sign) * (state_->pd + state_->integral);
    return state_->command;
  }

 private:
  const Config* const config_;
  State* const state_;
};

}  // namespace servo_mode
}  // namespace math
