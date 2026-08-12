// MIT-style impedance: T = Kp*(P_cmd-P_fdb) + Kd*(V_cmd-V_fdb) + T_ff.
// Position is continuous multi-turn mechanical radians (no wrap).
#pragma once

#include "math/foc/controller.h"
#include "math/constants.h"

namespace math { namespace servo_mode
{
using foc::IsFinite;
using foc::Limit;
using foc::QuietNan;

class MitMode
{
 public:
  struct Command
  {
    float position_rad = 0.0f;     // multi-turn continuous
    float velocity_rad_s = 0.0f;
    float kp_nm_per_rad = 0.0f;
    float kd_nm_s_per_rad = 0.0f;
    float feedforward_Nm = 0.0f;
    float max_torque_Nm = QuietNan();  // NaN => options.max_torque_Nm
  };

  struct Options
  {
    float max_torque_Nm = 1.0f;
    float torque_constant_Nm_A = 0.1f;
    float max_iq_A = 2.0f;
  };

  explicit MitMode(const Options& options) : options_(options) {}

  void SetOptions(const Options& options) { options_ = options; }
  const Options& options() const { return options_; }

  void Start(const Command& command)
  {
    command_ = command;
    NormalizeLimits(&command_);
    iq_ref_ = 0.0f;
    torque_Nm_ = 0.0f;
    active_ = true;
  }

  void SetCommand(const Command& command)
  {
    command_ = command;
    NormalizeLimits(&command_);
  }

  void Stop()
  {
    active_ = false;
    iq_ref_ = 0.0f;
    torque_Nm_ = 0.0f;
  }

  bool active() const { return active_; }
  float iq_ref_A() const { return iq_ref_; }
  float torque_Nm() const { return torque_Nm_; }
  float position_cmd_rad() const { return command_.position_rad; }
  float velocity_cmd_rad_s() const { return command_.velocity_rad_s; }

  // Returns Iq reference (A). Feedback must be continuous multi-turn mech.
  float Step(float /*dt_s*/, float position_fdb_rad, float velocity_fdb_rad_s)
  {
    if (!active_)
    {
      iq_ref_ = 0.0f;
      torque_Nm_ = 0.0f;
      return 0.0f;
    }

    const float torque =
        command_.kp_nm_per_rad * (command_.position_rad - position_fdb_rad) +
        command_.kd_nm_s_per_rad *
            (command_.velocity_rad_s - velocity_fdb_rad_s) +
        command_.feedforward_Nm;

    const float max_t = IsFinite(command_.max_torque_Nm)
                            ? command_.max_torque_Nm
                            : options_.max_torque_Nm;
    torque_Nm_ = Limit(torque, -max_t, max_t);

    float kt = options_.torque_constant_Nm_A;
    if (kt < 1.0e-6f)
    {
      kt = 1.0e-6f;
    }
    float iq = torque_Nm_ / kt;
    const float max_iq = options_.max_iq_A;
    iq_ref_ = Limit(iq, -max_iq, max_iq);
    return iq_ref_;
  }

 private:
  void NormalizeLimits(Command* command) const
  {
    if (command == nullptr)
    {
      return;
    }
    if (command->kp_nm_per_rad < 0.0f)
    {
      command->kp_nm_per_rad = 0.0f;
    }
    if (command->kd_nm_s_per_rad < 0.0f)
    {
      command->kd_nm_s_per_rad = 0.0f;
    }
    if (IsFinite(command->max_torque_Nm) && command->max_torque_Nm < 0.0f)
    {
      command->max_torque_Nm = 0.0f;
    }
  }

  Options options_{};
  Command command_{};
  float iq_ref_ = 0.0f;
  float torque_Nm_ = 0.0f;
  bool active_ = false;
};

}  // namespace servo_mode
}  // namespace math
