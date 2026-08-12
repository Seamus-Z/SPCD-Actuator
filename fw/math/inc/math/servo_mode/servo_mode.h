// moteus-style kPosition: trajectory → PID → torque → Iq.
// Units at the public API: mechanical radians / rad/s / Nm.
// Internally PID matches moteus (revolutions / rev/s).
#pragma once

#include "math/servo_mode/pid.h"
#include "math/constants.h"

namespace math { namespace servo_mode
{
using foc::IsFinite;
using foc::Limit;
using foc::QuietNan;

class ServoMode
{
 public:
  struct Command
  {
    float position_rad = QuietNan();
    float velocity_rad_s = 0.0f;
    float stop_position_rad = QuietNan();
    float max_torque_Nm = QuietNan();
    float feedforward_Nm = 0.0f;
    float velocity_limit_rad_s = QuietNan();
    float accel_limit_rad_s2 = QuietNan();
    float kp_scale = 1.0f;
    float kd_scale = 1.0f;
    float ilimit_scale = 1.0f;
    float id_ref_A = 0.0f;
  };

  struct Observation
  {
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float dt_s = 0.0f;
  };

  struct Output
  {
    float id_ref_A = 0.0f;
    float iq_ref_A = 0.0f;
    float torque_Nm = 0.0f;
    bool faulted = false;
    bool trajectory_done = false;
  };

  struct Options
  {
    PID::Config pid{};
    float max_torque_Nm = 1.0f;
    float torque_constant_Nm_A = 0.1f;
    float max_iq_A = 2.0f;
    float default_velocity_limit_rad_s = QuietNan();
    float default_accel_limit_rad_s2 = QuietNan();
    float max_velocity_cmd_rad_s = 0.0f;
    float velocity_threshold = 0.0f;
    float max_position_slip_rad = 0.0f;
    float max_velocity_error_rad_s = 0.0f;
    float velocity_zero_capture_threshold = 0.01f;
  };

  explicit ServoMode(const Options& options)
      : options_(options), pid_(&options_.pid, &pid_state_)
  {
  }

  void Start(const Command& command)
  {
    command_ = command;
    NormalizeCommandLimits(&command_);
    control_velocity_hz_ = QuietNan();
    control_position_turns_ = QuietNan();
    control_accel_hz_s_ = 0.0f;
    trajectory_done_ = false;
    pid_state_.Clear();
    iq_ref_ = 0.0f;
    torque_Nm_ = 0.0f;
    faulted_ = false;
    active_ = true;
  }

  void Start(float position_cmd_rad, float velocity_cmd_mech_rad_s,
             float id_ref_A = 0.0f)
  {
    Command command{};
    command.position_rad = position_cmd_rad;
    command.velocity_rad_s = velocity_cmd_mech_rad_s;
    command.id_ref_A = id_ref_A;
    Start(command);
  }

  void Stop()
  {
    active_ = false;
    control_velocity_hz_ = QuietNan();
    control_position_turns_ = QuietNan();
    control_accel_hz_s_ = 0.0f;
    trajectory_done_ = false;
    iq_ref_ = 0.0f;
    torque_Nm_ = 0.0f;
    faulted_ = false;
    pid_state_.Clear();
  }

  void SetCommand(const Command& command)
  {
    command_ = command;
    NormalizeCommandLimits(&command_);
    trajectory_done_ = false;
  }

  void SetVelocity(float velocity_cmd_mech_rad_s)
  {
    command_.velocity_rad_s = velocity_cmd_mech_rad_s;
    trajectory_done_ = false;
  }

  void SetPosition(float position_cmd_rad)
  {
    command_.position_rad = position_cmd_rad;
    trajectory_done_ = false;
  }

  void SetIdRef(float id_A) { command_.id_ref_A = id_A; }

  void SetTorqueConstant(float nm_per_a)
  {
    options_.torque_constant_Nm_A = nm_per_a;
    options_.max_torque_Nm = nm_per_a * options_.max_iq_A;
  }

  void SetOptions(const Options& options) { options_ = options; }
  const Options& options() const { return options_; }

  bool active() const { return active_; }
  float iq_ref_A() const { return iq_ref_; }
  float id_ref_A() const { return command_.id_ref_A; }
  float torque_Nm() const { return torque_Nm_; }
  bool faulted() const { return faulted_; }
  bool trajectory_done() const { return trajectory_done_; }
  float control_position_rad() const
  {
    return IsFinite(control_position_turns_)
               ? (control_position_turns_ * math::k2Pi)
               : QuietNan();
  }
  float velocity_cmd() const { return command_.velocity_rad_s; }
  float control_velocity() const
  {
    return IsFinite(control_velocity_hz_) ? (control_velocity_hz_ * math::k2Pi)
                                          : 0.0f;
  }

  Output Step(const Observation& observation)
  {
    const float iq_ref = Step(observation.dt_s, observation.position_rad,
                              observation.velocity_rad_s);
    Output output{};
    output.id_ref_A = command_.id_ref_A;
    output.iq_ref_A = iq_ref;
    output.torque_Nm = torque_Nm_;
    output.faulted = faulted_;
    output.trajectory_done = trajectory_done_;
    return output;
  }

  float Step(float dt_s, float measured_position_rad,
             float measured_velocity_rad_s)
  {
    if (!active_ || faulted_)
    {
      return 0.0f;
    }
    if (dt_s < 1.0e-7f)
    {
      dt_s = 1.0e-7f;
    }

    const float measured_turns = measured_position_rad / math::k2Pi;
    const float measured_hz = measured_velocity_rad_s / math::k2Pi;
    UpdateTrajectory(dt_s, measured_turns, measured_hz);

    float vel_for_pid = measured_hz;
    if (options_.velocity_threshold > 0.0f && IsFinite(control_velocity_hz_))
    {
      const float thr_hz = options_.velocity_threshold / math::k2Pi;
      const float dv = measured_hz - control_velocity_hz_;
      if (dv > -thr_hz && dv < thr_hz)
      {
        vel_for_pid = control_velocity_hz_;
      }
    }

    PID::ApplyOptions apply_options;
    apply_options.kp_scale = command_.kp_scale;
    apply_options.kd_scale = command_.kd_scale;
    apply_options.ilimit_scale = command_.ilimit_scale;
    if (!IsFinite(command_.position_rad) && command_.ilimit_scale == 1.0f &&
        options_.pid.ilimit <= 0.0f)
    {
      apply_options.ilimit_scale = 0.0f;
    }

    const float control_pos =
        IsFinite(control_position_turns_) ? control_position_turns_
                                          : measured_turns;
    const float control_vel =
        IsFinite(control_velocity_hz_) ? control_velocity_hz_ : 0.0f;
    const float raw_torque =
        pid_.Apply(measured_turns, control_pos, vel_for_pid, control_vel, dt_s,
                   apply_options) +
        command_.feedforward_Nm;

    const float max_torque = ActiveMaxTorqueNm();
    float torque = Limit(raw_torque, -max_torque, max_torque);
    if (torque != raw_torque)
    {
      const float sign = options_.pid.sign < 0 ? -1.0f : 1.0f;
      float integral = torque / sign - pid_state_.pd;
      const float ilimit = options_.pid.ilimit * apply_options.ilimit_scale;
      integral = ilimit > 0.0f ? Limit(integral, -ilimit, ilimit) : 0.0f;
      pid_state_.integral = integral;
      pid_state_.command = sign * (pid_state_.pd + pid_state_.integral);
      torque = Limit(pid_state_.command, -max_torque, max_torque);
    }
    torque_Nm_ = torque;

    const float kt = options_.torque_constant_Nm_A;
    iq_ref_ = (kt > 1.0e-6f) ? (torque / kt) : 0.0f;
    iq_ref_ = Limit(iq_ref_, -options_.max_iq_A, options_.max_iq_A);
    return iq_ref_;
  }

 private:
  Options options_;
  Command command_{};
  PID::State pid_state_{};
  PID pid_;
  bool active_ = false;
  bool faulted_ = false;
  bool trajectory_done_ = false;
  float control_velocity_hz_ = QuietNan();
  float control_position_turns_ = QuietNan();
  float control_accel_hz_s_ = 0.0f;
  float iq_ref_ = 0.0f;
  float torque_Nm_ = 0.0f;

  static float Abs(float value) { return value >= 0.0f ? value : -value; }

  float ActiveMaxTorqueNm() const
  {
    if (IsFinite(command_.max_torque_Nm) && command_.max_torque_Nm > 0.0f)
    {
      return command_.max_torque_Nm;
    }
    return options_.max_torque_Nm;
  }

  void NormalizeCommandLimits(Command* command) const
  {
    if (command == nullptr)
    {
      return;
    }
    if (!IsFinite(command->velocity_limit_rad_s))
    {
      command->velocity_limit_rad_s = options_.default_velocity_limit_rad_s;
    }
    else if (command->velocity_limit_rad_s < 0.0f)
    {
      command->velocity_limit_rad_s = QuietNan();
    }
    if (!IsFinite(command->accel_limit_rad_s2))
    {
      command->accel_limit_rad_s2 = options_.default_accel_limit_rad_s2;
    }
    else if (command->accel_limit_rad_s2 < 0.0f)
    {
      command->accel_limit_rad_s2 = QuietNan();
    }
    if (IsFinite(command->velocity_limit_rad_s) ||
        IsFinite(command->accel_limit_rad_s2))
    {
      if (!IsFinite(command->velocity_limit_rad_s))
      {
        command->velocity_limit_rad_s = options_.max_velocity_cmd_rad_s;
      }
      else if (options_.max_velocity_cmd_rad_s > 0.0f)
      {
        command->velocity_limit_rad_s = Abs(Limit(
            command->velocity_limit_rad_s, -options_.max_velocity_cmd_rad_s,
            options_.max_velocity_cmd_rad_s));
      }
    }
    if (IsFinite(command->velocity_limit_rad_s))
    {
      command->velocity_rad_s =
          Limit(command->velocity_rad_s, -command->velocity_limit_rad_s,
                command->velocity_limit_rad_s);
    }
    else if (options_.max_velocity_cmd_rad_s > 0.0f)
    {
      command->velocity_rad_s =
          Limit(command->velocity_rad_s, -options_.max_velocity_cmd_rad_s,
                options_.max_velocity_cmd_rad_s);
    }
    if (!IsFinite(command->kp_scale)) command->kp_scale = 1.0f;
    if (!IsFinite(command->kd_scale)) command->kd_scale = 1.0f;
    if (!IsFinite(command->ilimit_scale)) command->ilimit_scale = 1.0f;
  }

  void EnsureControlState(float measured_turns, float measured_hz)
  {
    if (!IsFinite(control_position_turns_))
    {
      control_position_turns_ = measured_turns;
      if (Abs(measured_hz) <
          options_.velocity_zero_capture_threshold / math::k2Pi)
      {
        control_velocity_hz_ = 0.0f;
      }
      else
      {
        control_velocity_hz_ = measured_hz;
      }
      control_accel_hz_s_ = 0.0f;
    }
    if (!IsFinite(control_velocity_hz_))
    {
      control_velocity_hz_ = 0.0f;
    }
  }

  float CalculateAcceleration(float a, float v0, float vf, float dx, float dt,
                              float velocity_limit_hz) const
  {
    const float v0_abs = Abs(v0);
    if (IsFinite(velocity_limit_hz) && v0_abs > velocity_limit_hz)
    {
      return (v0 > 0.0f) ? -a : a;
    }
    const float v_frame = v0 - vf;
    const float v_frame_abs = Abs(v_frame);
    if ((v_frame * dx) >= 0.0f && dx != 0.0f)
    {
      const float inv_2a = 1.0f / (2.0f * a);
      const float stop_distance = (v_frame * v_frame) * inv_2a;
      const float dx_abs = Abs(dx);
      const float inv_2dx = 1.0f / (2.0f * dx_abs);
      if (dx_abs > stop_distance)
      {
        const float v_next = v_frame_abs + a * dt;
        const float dx_step = (v_frame_abs + v_next) * 0.5f * dt;
        const float dx_after = dx_abs - dx_step;
        const float stop_distance_next = (v_next * v_next) * inv_2a;
        if (dx_after < stop_distance_next)
        {
          float required = (v_frame_abs * v_frame_abs) * inv_2dx;
          if (required > a) required = a;
          return (v_frame > 0.0f) ? -required : required;
        }
        if (!IsFinite(velocity_limit_hz) || v0_abs < velocity_limit_hz)
        {
          return (dx > 0.0f) ? a : -a;
        }
        return 0.0f;
      }
      float required = (v_frame_abs * v_frame_abs) * inv_2dx;
      if (required > a) required = a;
      return (v_frame > 0.0f) ? -required : required;
    }
    return (v_frame > 0.0f) ? -a : a;
  }

  void UpdateTrajectory(float dt_s, float measured_turns, float measured_hz)
  {
    float velocity_cmd_hz = command_.velocity_rad_s / math::k2Pi;
    const bool has_position = IsFinite(command_.position_rad);
    const bool has_vel_limit = IsFinite(command_.velocity_limit_rad_s);
    const bool has_accel_limit = IsFinite(command_.accel_limit_rad_s2);
    float position_cmd_turns =
        has_position ? (command_.position_rad / math::k2Pi) : QuietNan();

    if (!has_vel_limit && !has_accel_limit)
    {
      trajectory_done_ = true;
      control_accel_hz_s_ = 0.0f;
      control_velocity_hz_ = velocity_cmd_hz;
      if (has_position)
      {
        control_position_turns_ = position_cmd_turns;
        command_.position_rad = QuietNan();
      }
      else
      {
        EnsureControlState(measured_turns, measured_hz);
        control_position_turns_ += control_velocity_hz_ * dt_s;
      }
      FinishTrajectoryGuards(dt_s, measured_turns, measured_hz);
      return;
    }

    if (has_position || Abs(velocity_cmd_hz) > 0.0f)
    {
      trajectory_done_ = false;
    }
    EnsureControlState(measured_turns, measured_hz);

    const float v0 = control_velocity_hz_;
    if (!trajectory_done_)
    {
      if (!has_position)
      {
        if (has_accel_limit)
        {
          const float accel_hz_s = command_.accel_limit_rad_s2 / math::k2Pi;
          const float dv = velocity_cmd_hz - control_velocity_hz_;
          const float initial_sign = (dv > 0.0f) ? 1.0f : -1.0f;
          control_accel_hz_s_ = accel_hz_s * initial_sign;
          control_velocity_hz_ += control_accel_hz_s_ * dt_s;
          const float final_sign =
              (velocity_cmd_hz > control_velocity_hz_) ? 1.0f : -1.0f;
          if (final_sign != initial_sign)
          {
            control_accel_hz_s_ = 0.0f;
            control_velocity_hz_ = velocity_cmd_hz;
            trajectory_done_ = true;
          }
        }
        else
        {
          control_accel_hz_s_ = 0.0f;
          control_velocity_hz_ = velocity_cmd_hz;
          trajectory_done_ = true;
        }
      }
      else if (!has_accel_limit && has_vel_limit)
      {
        const float dx = position_cmd_turns - control_position_turns_;
        const float initial_sign = dx < 0.0f ? 1.0f : -1.0f;
        control_accel_hz_s_ = 0.0f;
        control_velocity_hz_ =
            -initial_sign * (command_.velocity_limit_rad_s / math::k2Pi);
        const float next_dx = dx - control_velocity_hz_ * dt_s;
        const float final_sign = (next_dx < 0.0f) ? 1.0f : -1.0f;
        if (final_sign != initial_sign)
        {
          command_.position_rad = QuietNan();
          control_velocity_hz_ = velocity_cmd_hz;
          trajectory_done_ = true;
        }
      }
      else
      {
        const float a = command_.accel_limit_rad_s2 / math::k2Pi;
        const float vf = velocity_cmd_hz;
        const float dx = position_cmd_turns - control_position_turns_;
        const float vmax_hz = has_vel_limit
                                  ? (command_.velocity_limit_rad_s / math::k2Pi)
                                  : QuietNan();
        const float acceleration =
            CalculateAcceleration(a, v0, vf, dx, dt_s, vmax_hz);
        control_accel_hz_s_ = acceleration;
        control_velocity_hz_ += acceleration * dt_s;
        const float v1 = control_velocity_hz_;
        if (has_vel_limit)
        {
          const float vel_lower = (Abs(v0) < Abs(v1)) ? Abs(v0) : Abs(v1);
          const float vel_upper = (Abs(v0) > Abs(v1)) ? Abs(v0) : Abs(v1);
          if (vel_lower < vmax_hz && vel_upper > vmax_hz)
          {
            control_accel_hz_s_ = 0.0f;
            control_velocity_hz_ = (v1 >= 0.0f) ? vmax_hz : -vmax_hz;
          }
        }
        const float v1_final = control_velocity_hz_;
        const float signed_lower = (v0 < v1_final) ? v0 : v1_final;
        const float signed_upper = (v0 > v1_final) ? v0 : v1_final;
        const float dx_abs = Abs(dx);
        const float v_frame_final_abs = Abs(v1_final - vf);
        const bool target_cross = signed_lower <= vf && signed_upper >= vf;
        const bool target_near = v_frame_final_abs < (a * 0.5f * dt_s);
        const float v_for_threshold =
            (v_frame_final_abs > Abs(vf)) ? v_frame_final_abs : Abs(vf);
        const bool position_near = dx_abs <= v_for_threshold * 10.0f * dt_s;
        if ((target_cross || target_near) && position_near)
        {
          command_.position_rad = QuietNan();
          control_accel_hz_s_ = 0.0f;
          control_velocity_hz_ = vf;
          trajectory_done_ = true;
        }
      }
    }

    // If a stop_position is armed and we are already on/beyond it in the
    // travel direction, kill velocity before integrating this sample.
    if (IsFinite(command_.stop_position_rad) &&
        IsFinite(control_position_turns_) &&
        IsFinite(control_velocity_hz_))
    {
      const float stop_turns = command_.stop_position_rad / math::k2Pi;
      if ((control_velocity_hz_ > 0.0f &&
           control_position_turns_ >= stop_turns) ||
          (control_velocity_hz_ < 0.0f &&
           control_position_turns_ <= stop_turns))
      {
        control_accel_hz_s_ = 0.0f;
        control_velocity_hz_ = 0.0f;
        control_position_turns_ = stop_turns;
        command_.velocity_rad_s = 0.0f;
        trajectory_done_ = true;
      }
    }

    const float v1 = control_velocity_hz_;
    const float integrate_hz =
        (control_accel_hz_s_ != 0.0f) ? (0.5f * (v0 + v1)) : v1;
    control_position_turns_ += integrate_hz * dt_s;
    FinishTrajectoryGuards(dt_s, measured_turns, measured_hz);
  }

  void FinishTrajectoryGuards(float dt_s, float measured_turns,
                              float measured_hz)
  {
    if (IsFinite(command_.position_rad) && Abs(command_.velocity_rad_s) > 0.0f)
    {
      command_.position_rad += command_.velocity_rad_s * dt_s;
    }
    if (options_.max_position_slip_rad > 0.0f &&
        IsFinite(control_position_turns_))
    {
      const float slip_turns = options_.max_position_slip_rad / math::k2Pi;
      const float error = measured_turns - control_position_turns_;
      if (error < -slip_turns)
      {
        control_position_turns_ = measured_turns + slip_turns;
      }
      else if (error > slip_turns)
      {
        control_position_turns_ = measured_turns - slip_turns;
      }
    }
    if (options_.max_velocity_error_rad_s > 0.0f &&
        IsFinite(control_velocity_hz_))
    {
      const float slip_hz = options_.max_velocity_error_rad_s / math::k2Pi;
      const float error = measured_hz - control_velocity_hz_;
      if (error < -slip_hz)
      {
        control_accel_hz_s_ = 0.0f;
        control_velocity_hz_ = measured_hz + slip_hz;
      }
      else if (error > slip_hz)
      {
        control_accel_hz_s_ = 0.0f;
        control_velocity_hz_ = measured_hz - slip_hz;
      }
    }
    if (IsFinite(command_.stop_position_rad) &&
        IsFinite(control_position_turns_) && IsFinite(control_velocity_hz_))
    {
      const float stop_turns = command_.stop_position_rad / math::k2Pi;
      // Reach/cross test (not just strict overshoot). Needed because the host
      // may keep streaming the same velocity+stop command at 50 Hz after
      // arrival; a ">"-only check leaves a one-sample poke past the stop.
      const bool reached =
          (control_velocity_hz_ > 0.0f &&
           control_position_turns_ >= stop_turns) ||
          (control_velocity_hz_ < 0.0f &&
           control_position_turns_ <= stop_turns) ||
          (control_velocity_hz_ == 0.0f &&
           command_.velocity_rad_s != 0.0f &&
           Abs(control_position_turns_ - stop_turns) <=
               (Abs(command_.velocity_rad_s) / math::k2Pi) * dt_s);
      if (reached)
      {
        control_position_turns_ = stop_turns;
        control_accel_hz_s_ = 0.0f;
        control_velocity_hz_ = 0.0f;
        command_.position_rad = QuietNan();
        command_.velocity_rad_s = 0.0f;
        trajectory_done_ = true;
      }
    }
  }
};

}  // namespace servo_mode
}  // namespace math
