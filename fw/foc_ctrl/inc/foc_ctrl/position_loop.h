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
    // Soften D when |ω_meas - ω_cmd| is below this (mech rad/s).
    float velocity_threshold = 0.0f;
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
    velocity_cmd_hz_ = velocity_cmd_mech_rad_s / math::k2Pi;
    options_.id_ref_A = id_ref_A;
    control_position_turns_ = QuietNan();
    pid_state_.Clear();
    iq_ref_ = 0.0f;
    torque_Nm_ = 0.0f;
    active_ = true;
  }

  void Stop()
  {
    active_ = false;
    iq_ref_ = 0.0f;
    torque_Nm_ = 0.0f;
    pid_state_.Clear();
  }

  void SetVelocity(float velocity_cmd_mech_rad_s)
  {
    velocity_cmd_hz_ = velocity_cmd_mech_rad_s / math::k2Pi;
  }

  void SetIdRef(float id_A) { options_.id_ref_A = id_A; }

  bool active() const { return active_; }
  float iq_ref_A() const { return iq_ref_; }
  float id_ref_A() const { return options_.id_ref_A; }
  float torque_Nm() const { return torque_Nm_; }
  float control_position_rad() const
  {
    return control_position_turns_ * math::k2Pi;
  }
  float velocity_cmd() const { return velocity_cmd_hz_ * math::k2Pi; }

  // Returns iq_ref [A]. measured_* are mechanical rad / rad/s (unwrapped OK).
  float Step(float dt_s, float measured_position_rad,
             float measured_velocity_rad_s)
  {
    if (!active_)
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
      if (IsFinite(position_cmd_turns_))
      {
        control_position_turns_ = position_cmd_turns_;
      }
    }

    if (IsFinite(position_cmd_turns_))
    {
      control_position_turns_ = position_cmd_turns_;
    }
    else
    {
      // Velocity mode: advance the virtual setpoint.
      control_position_turns_ += velocity_cmd_hz_ * dt_s;
    }

    float vel_for_pid = measured_hz;
    if (options_.velocity_threshold > 0.0f)
    {
      const float thr_hz = options_.velocity_threshold / math::k2Pi;
      const float dv = measured_hz - velocity_cmd_hz_;
      if (dv > -thr_hz && dv < thr_hz)
      {
        vel_for_pid = velocity_cmd_hz_;
      }
    }

    // moteus: Apply(measured, desired, measured_rate, desired_rate)
    float torque = pid_.Apply(measured_turns, control_position_turns_,
                              vel_for_pid, velocity_cmd_hz_, dt_s);
    torque = Limit(torque, -options_.max_torque_Nm, options_.max_torque_Nm);
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
  float position_cmd_turns_ = QuietNan();
  float velocity_cmd_hz_ = 0.0f;
  float control_position_turns_ = QuietNan();
  float iq_ref_ = 0.0f;
  float torque_Nm_ = 0.0f;
};

}  // namespace foc_ctrl
