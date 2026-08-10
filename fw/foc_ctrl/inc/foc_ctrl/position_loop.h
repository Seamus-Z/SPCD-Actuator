// moteus-style position/velocity PID → torque → Iq.
// Units at the public API: mechanical radians / rad/s.
// Internally PID matches moteus (revolutions / rev/s).
// Velocity mode: position=NaN, integrate velocity command (moteus kPosition).
#pragma once

#include "foc_ctrl/pid.h"
#include "math/constants.h"

namespace foc_ctrl
{

class PositionLoop
{
 public:
  struct Options
  {
    PID::Config pid{};
    float max_torque_Nm = 1.0f;
    float torque_constant_Nm_A = 0.1f;
    float max_iq_A = 2.0f;
    float id_ref_A = 0.0f;
    float acceleration_limit_rad_s2 = 0.0f;
    // Absolute mechanical command limit. Zero disables clamping.
    float max_velocity_cmd_rad_s = 0.0f;
    // Soften D when |ω_meas - ω_cmd| is below this (mech rad/s).
    float velocity_threshold = 0.0f;
    // Stop torque generation when the virtual trajectory loses the rotor.
    // Zero disables each guard.
    float max_position_slip_rad = 0.0f;
    float max_velocity_error_rad_s = 0.0f;
  };

  explicit PositionLoop(const Options& options)
      : options_(options), pid_(&options_.pid, &pid_state_)
  {
  }

  // position_cmd_rad = QuietNan() → velocity tracking (moteus velocity mode).
  void Start(float position_cmd_rad, float velocity_cmd_mech_rad_s,
             float id_ref_A = 0.0f)
  {
    position_cmd_turns_ = IsFinite(position_cmd_rad)
                              ? (position_cmd_rad / math::k2Pi)
                              : QuietNan();
    velocity_cmd_hz_ =
        ClampVelocityCommand(velocity_cmd_mech_rad_s) / math::k2Pi;
    options_.id_ref_A = id_ref_A;
    control_velocity_hz_ = 0.0f;
    control_position_turns_ = QuietNan();
    velocity_position_error_turns_ = 0.0f;
    pid_state_.Clear();
    iq_ref_ = 0.0f;
    torque_Nm_ = 0.0f;
    faulted_ = false;
    active_ = true;
  }

  void Stop()
  {
    active_ = false;
    control_velocity_hz_ = 0.0f;
    velocity_position_error_turns_ = 0.0f;
    iq_ref_ = 0.0f;
    torque_Nm_ = 0.0f;
    faulted_ = false;
    pid_state_.Clear();
  }

  void SetVelocity(float velocity_cmd_mech_rad_s)
  {
    velocity_cmd_hz_ =
        ClampVelocityCommand(velocity_cmd_mech_rad_s) / math::k2Pi;
  }

  void SetIdRef(float id_A) { options_.id_ref_A = id_A; }

  // Runtime Kt≈Ke override (e.g. from an on-device identification result).
  void SetTorqueConstant(float nm_per_a)
  {
    options_.torque_constant_Nm_A = nm_per_a;
    options_.max_torque_Nm = nm_per_a * options_.max_iq_A;
  }

  bool active() const { return active_; }
  float iq_ref_A() const { return iq_ref_; }
  float id_ref_A() const { return options_.id_ref_A; }
  float torque_Nm() const { return torque_Nm_; }
  bool faulted() const { return faulted_; }
  float control_position_rad() const
  {
    return control_position_turns_ * math::k2Pi;
  }
  float velocity_cmd() const { return velocity_cmd_hz_ * math::k2Pi; }
  float control_velocity() const { return control_velocity_hz_ * math::k2Pi; }

  // Returns iq_ref [A]. measured_* are mechanical rad / rad/s (unwrapped OK).
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

    if (!IsFinite(control_position_turns_))
    {
      control_position_turns_ = measured_turns;
      control_velocity_hz_ = measured_hz;
      if (IsFinite(position_cmd_turns_))
      {
        control_position_turns_ = position_cmd_turns_;
      }
    }
    const float accel_hz_s = options_.acceleration_limit_rad_s2 / math::k2Pi;
    if (accel_hz_s > 0.0f)
    {
      const float max_step_hz = accel_hz_s * dt_s;
      control_velocity_hz_ +=
          Limit(velocity_cmd_hz_ - control_velocity_hz_,
                -max_step_hz, max_step_hz);
    }
    else
    {
      control_velocity_hz_ = velocity_cmd_hz_;
    }

    // Velocity slip, underspeed-only. If the rotor cannot keep up with the
    // virtual trajectory, pull the trajectory down toward measured so a
    // torque/voltage-limited command rides at the achievable speed.
    //
    // Do NOT pull the trajectory *up* toward measured. That branch glued
    // control_velocity to ω_meas during deceleration and made +ω → −ω
    // reversals fail (accel stepped down, slip yanked it back every cycle).
    // Overspeed is handled by the command-side clamp below; BEMF/phase-lead
    // already use measured ω, so the old up-yank is no longer needed.
    if (!IsFinite(position_cmd_turns_) &&
        options_.max_velocity_error_rad_s > 0.0f)
    {
      const float max_cmd_hz = options_.max_velocity_cmd_rad_s / math::k2Pi;
      const bool measured_plausible =
          max_cmd_hz <= 0.0f ||
          (measured_hz <= max_cmd_hz * 1.25f &&
           measured_hz >= -max_cmd_hz * 1.25f);
      if (measured_plausible)
      {
        const float slip_hz = options_.max_velocity_error_rad_s / math::k2Pi;
        const float err_hz = measured_hz - control_velocity_hz_;
        if (err_hz < -slip_hz)
        {
          control_velocity_hz_ = measured_hz + slip_hz;
        }
      }
    }
    // Never let the virtual trajectory exceed the host command. This stops
    // overspeed lock without blocking deceleration/reversal.
    if (!IsFinite(position_cmd_turns_))
    {
      if (velocity_cmd_hz_ >= 0.0f)
      {
        if (control_velocity_hz_ > velocity_cmd_hz_)
        {
          control_velocity_hz_ = velocity_cmd_hz_;
        }
      }
      else if (control_velocity_hz_ < velocity_cmd_hz_)
      {
        control_velocity_hz_ = velocity_cmd_hz_;
      }
    }
    // Absolute board ceiling (invalid/host-limit guard).
    if (options_.max_velocity_cmd_rad_s > 0.0f)
    {
      const float max_hz = options_.max_velocity_cmd_rad_s / math::k2Pi;
      control_velocity_hz_ = Limit(control_velocity_hz_, -max_hz, max_hz);
    }

    if (IsFinite(position_cmd_turns_))
    {
      control_position_turns_ = position_cmd_turns_;
    }
    else
    {
      // The position-P term is the integral term of velocity mode.  Integrate
      // relative speed error directly so the state stays bounded and does not
      // lose precision after long runs.  Cap only where its P contribution
      // alone reaches the configured torque limit.
      velocity_position_error_turns_ +=
          (measured_hz - control_velocity_hz_) * dt_s;
      const float kp_abs = Abs(options_.pid.kp);
      if (kp_abs > 1.0e-9f && options_.max_torque_Nm > 0.0f)
      {
        const float max_error_turns = options_.max_torque_Nm / kp_abs;
        velocity_position_error_turns_ =
            Limit(velocity_position_error_turns_,
                  -max_error_turns, max_error_turns);
      }
      control_position_turns_ =
          measured_turns - velocity_position_error_turns_;
    }

    const float position_error_rad =
        (measured_turns - control_position_turns_) * math::k2Pi;
    // Fixed-position commands retain the position-slip fault. Velocity mode no
    // longer faults/resets — it is bounded by the torque-capped relative
    // integrator and the max_velocity_slip clamp above (moteus behaviour).
    if (IsFinite(position_cmd_turns_) &&
        options_.max_position_slip_rad > 0.0f &&
        Abs(position_error_rad) > options_.max_position_slip_rad)
    {
      faulted_ = true;
      iq_ref_ = 0.0f;
      torque_Nm_ = 0.0f;
      pid_state_.Clear();
      return 0.0f;
    }

    float vel_for_pid = measured_hz;
    if (options_.velocity_threshold > 0.0f)
    {
      const float thr_hz = options_.velocity_threshold / math::k2Pi;
      const float dv = measured_hz - control_velocity_hz_;
      if (dv > -thr_hz && dv < thr_hz)
      {
        vel_for_pid = control_velocity_hz_;
      }
    }

    // Velocity mode already integrates speed error into its virtual position.
    // A second PID integral creates an I² controller and large breakaway
    // cycles, so reserve PID-I for finite-position commands.
    const bool velocity_mode = !IsFinite(position_cmd_turns_);
    PID::ApplyOptions apply_options;
    if (velocity_mode)
    {
      apply_options.ilimit_scale = 0.0f;
    }
    const float raw_torque =
        pid_.Apply(measured_turns, control_position_turns_,
                   vel_for_pid, control_velocity_hz_, dt_s, apply_options);
    float torque =
        Limit(raw_torque, -options_.max_torque_Nm, options_.max_torque_Nm);
    if (torque != raw_torque)
    {
      const float sign = options_.pid.sign < 0 ? -1.0f : 1.0f;
      if (velocity_mode && Abs(options_.pid.kp) > 1.0e-9f)
      {
        // Back-calculate the speed-integrator state from the actuator limit.
        // This permits immediate braking after an unreachable command.
        velocity_position_error_turns_ =
            (torque / sign - pid_state_.d) / options_.pid.kp;
        control_position_turns_ =
            measured_turns - velocity_position_error_turns_;
        pid_state_.desired = control_position_turns_;
        pid_state_.error = velocity_position_error_turns_;
        pid_state_.p = options_.pid.kp * pid_state_.error;
        pid_state_.pd = pid_state_.p + pid_state_.d;
      }
      else
      {
        float integral = torque / sign - pid_state_.pd;
        const float ilimit = options_.pid.ilimit;
        integral = ilimit > 0.0f
                       ? Limit(integral, -ilimit, ilimit)
                       : 0.0f;
        pid_state_.integral = integral;
      }
      pid_state_.command =
          sign * (pid_state_.pd + pid_state_.integral);
      torque = Limit(pid_state_.command,
                     -options_.max_torque_Nm, options_.max_torque_Nm);
    }
    torque_Nm_ = torque;

    const float kt = options_.torque_constant_Nm_A;
    iq_ref_ = (kt > 1.0e-6f) ? (torque / kt) : 0.0f;
    iq_ref_ = Limit(iq_ref_, -options_.max_iq_A, options_.max_iq_A);
    return iq_ref_;
  }

 private:
  Options options_;
  PID::State pid_state_{};
  PID pid_;
  bool active_ = false;
  static float Abs(float value) { return value >= 0.0f ? value : -value; }
  float ClampVelocityCommand(float value) const
  {
    const float limit = options_.max_velocity_cmd_rad_s;
    return limit > 0.0f ? Limit(value, -limit, limit) : value;
  }
  float position_cmd_turns_ = QuietNan();
  float velocity_cmd_hz_ = 0.0f;
  float control_velocity_hz_ = 0.0f;
  float control_position_turns_ = QuietNan();
  float velocity_position_error_turns_ = 0.0f;
  float iq_ref_ = 0.0f;
  float torque_Nm_ = 0.0f;
  bool faulted_ = false;
};

}  // namespace foc_ctrl
