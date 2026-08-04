// Current FOC aligned with moteus ISR_DoCurrent (SimplePI + feedforward).
#pragma once

#include <cstdint>

#include "foc_ctrl/simple_pi.h"
#include "math/constants.h"
#include "math/foc.h"

extern "C" {
float sqrtf(float);
}

namespace foc_ctrl
{

class CurrentLoop
{
 public:
  struct Options
  {
    float bus_V = 48.0f;
    float min_duty = 0.05f;
    float max_duty = 0.95f;
    float kp_d = 0.35f;
    float ki_d = 0.015f * 15000.0f;
    float kp_q = 0.35f;
    float ki_q = 0.015f * 15000.0f;
    float max_current_A = 4.9f;
    float resistance_ohm = 0.65f;
    float inductance_H = 0.00034f;
    // BEMF feedforward scale: V / (mech rad/s), from Ke.
    float v_per_hz = 0.0f;  // name kept; units are V·s/rad mech
    float current_feedforward = 1.0f;
    float cross_coupling_feedforward = 1.0f;
    float bemf_feedforward = 1.0f;
  };

  struct Command
  {
    float theta_rad = 0.0f;
    float id_A = 0.0f;
    float iq_A = 0.0f;
    float theta_rate_rad_s = 0.0f;  // open-loop only when no θ override
  };

  struct Duties
  {
    uint16_t a_milli = 0;
    uint16_t b_milli = 0;
    uint16_t c_milli = 0;
  };

  explicit CurrentLoop(const Options& options)
      : options_(options),
        pi_d_(&pi_d_cfg_, &pi_d_state_),
        pi_q_(&pi_q_cfg_, &pi_q_state_)
  {
    SyncPiConfig();
  }

  void Start(const Command& cmd)
  {
    SyncPiConfig();
    theta_ = math::WrapZeroToTwoPi(cmd.theta_rad);
    id_ref_ = ClampCurrent(cmd.id_A);
    iq_ref_ = ClampCurrent(cmd.iq_A);
    theta_rate_ = cmd.theta_rate_rad_s;
    omega_elec_ = 0.0f;
    omega_rotor_ = 0.0f;
    pi_d_state_.Clear();
    pi_q_state_.Clear();
    id_A_ = 0.0f;
    iq_A_ = 0.0f;
    vd_V_ = 0.0f;
    vq_V_ = 0.0f;
    active_ = true;
  }

  void Stop()
  {
    active_ = false;
    pi_d_state_.Clear();
    pi_q_state_.Clear();
  }

  void SetRefs(float id_A, float iq_A)
  {
    id_ref_ = ClampCurrent(id_A);
    iq_ref_ = ClampCurrent(iq_A);
  }

  void SetThetaRate(float theta_rate_rad_s) { theta_rate_ = theta_rate_rad_s; }

  // Feedforward velocities (elec rad/s, rotor mech rad/s).
  void SetOmega(float omega_elec, float omega_rotor_mech)
  {
    omega_elec_ = omega_elec;
    omega_rotor_ = omega_rotor_mech;
  }

  bool active() const { return active_; }
  float theta_rad() const { return theta_; }
  float theta_rate_rad_s() const { return theta_rate_; }
  float id_ref_A() const { return id_ref_; }
  float iq_ref_A() const { return iq_ref_; }
  float id_A() const { return id_A_; }
  float iq_A() const { return iq_A_; }
  float vd_V() const { return vd_V_; }
  float vq_V() const { return vq_V_; }
  float bus_V() const { return options_.bus_V; }

  void Observe(float ia, float ib, float ic)
  {
    const math::SinCos sc = math::SinCosFromRadians(theta_);
    const math::DqTransform dq(sc, ia, ib, ic);
    id_A_ = dq.d;
    iq_A_ = dq.q;
  }

  // If theta_elec_override != nullptr: encoder commutation (moteus kCurrent).
  // Else: open-loop θ += rate (legacy; prefer Vfoc for open-loop).
  bool Step(float dt_s, float ia, float ib, float ic, Duties* out,
            const float* theta_elec_override = nullptr)
  {
    if (!active_ || out == nullptr)
    {
      return false;
    }
    if (dt_s < 0.0f)
    {
      dt_s = 0.0f;
    }

    if (theta_elec_override != nullptr)
    {
      theta_ = math::WrapZeroToTwoPi(*theta_elec_override);
    }
    else
    {
      theta_ = math::WrapZeroToTwoPi(theta_ + theta_rate_ * dt_s);
    }
    Observe(ia, ib, ic);

    const float max_V =
        (0.5f - options_.min_duty) * options_.bus_V * math::kSvpwmRatio;

    const float i_d_cmd = id_ref_;
    const float i_q_cmd = iq_ref_;

    const float cross_d =
        -omega_elec_ * options_.inductance_H * i_q_cmd *
        options_.cross_coupling_feedforward;
    const float cross_q =
        omega_elec_ * options_.inductance_H * i_d_cmd *
        options_.cross_coupling_feedforward;

    const float d_ff =
        i_d_cmd * options_.current_feedforward * options_.resistance_ohm +
        cross_d;
    const float q_ff =
        i_q_cmd * options_.current_feedforward * options_.resistance_ohm +
        cross_q +
        omega_rotor_ * options_.bemf_feedforward * options_.v_per_hz;

    float denorm_d = pi_d_.Apply(id_A_, i_d_cmd, dt_s) + d_ff;
    float denorm_q = pi_q_.Apply(iq_A_, i_q_cmd, dt_s) + q_ff;

    float d_V = denorm_d;
    float q_V = denorm_q;
    LimitToMaxVoltage(max_V, &d_V, &q_V);

    // Back-calculation anti-windup (moteus).
    if (d_V != denorm_d)
    {
      pi_d_.SetIntegralForOutput(d_V - d_ff);
    }
    if (q_V != denorm_q)
    {
      pi_q_.SetIntegralForOutput(q_V - q_ff);
    }
    pi_d_state_.integral = Limit(pi_d_state_.integral, -max_V, max_V);
    pi_q_state_.integral = Limit(pi_q_state_.integral, -max_V, max_V);

    vd_V_ = d_V;
    vq_V_ = q_V;

    const math::SinCos sc = math::SinCosFromRadians(theta_);
    const math::InverseDqTransform idt(sc, vd_V_, vq_V_);

    math::BalancedPwm pwm;
    if (!math::BalancedPwm::FromPhaseVolts(idt.a, idt.b, idt.c, options_.bus_V,
                                          options_.min_duty, options_.max_duty,
                                          &pwm))
    {
      return false;
    }

    out->a_milli = DutyToMilli(pwm.duty_a);
    out->b_milli = DutyToMilli(pwm.duty_b);
    out->c_milli = DutyToMilli(pwm.duty_c);
    return true;
  }

 private:
  void SyncPiConfig()
  {
    pi_d_cfg_.kp = options_.kp_d;
    pi_d_cfg_.ki = options_.ki_d;
    pi_d_cfg_.max_desired_rate = 0.0f;
    pi_q_cfg_.kp = options_.kp_q;
    pi_q_cfg_.ki = options_.ki_q;
    pi_q_cfg_.max_desired_rate = 0.0f;
  }

  float ClampCurrent(float i) const
  {
    return Limit(i, -options_.max_current_A, options_.max_current_A);
  }

  // D-axis priority circle limit (moteus limit_to_max_voltage).
  static void LimitToMaxVoltage(float max_V, float* d_V, float* q_V)
  {
    const float max_V_sq = max_V * max_V;
    const float len = (*d_V) * (*d_V) + (*q_V) * (*q_V);
    if (len <= max_V_sq)
    {
      return;
    }
    *d_V = Limit(*d_V, -max_V, max_V);
    float remain = max_V_sq - (*d_V) * (*d_V);
    if (remain < 0.0f)
    {
      remain = 0.0f;
    }
    const float q_max = sqrtf(remain);
    *q_V = Limit(*q_V, -q_max, q_max);
  }

  static uint16_t DutyToMilli(float duty)
  {
    float x = duty * 1000.0f + 0.5f;
    if (x < 0.0f)
    {
      x = 0.0f;
    }
    if (x > 1000.0f)
    {
      x = 1000.0f;
    }
    return static_cast<uint16_t>(x);
  }

  Options options_;
  SimplePI::Config pi_d_cfg_{};
  SimplePI::Config pi_q_cfg_{};
  SimplePI::State pi_d_state_{};
  SimplePI::State pi_q_state_{};
  SimplePI pi_d_;
  SimplePI pi_q_;
  bool active_ = false;
  float theta_ = 0.0f;
  float theta_rate_ = 0.0f;
  float omega_elec_ = 0.0f;
  float omega_rotor_ = 0.0f;
  float id_ref_ = 0.0f;
  float iq_ref_ = 0.0f;
  float id_A_ = 0.0f;
  float iq_A_ = 0.0f;
  float vd_V_ = 0.0f;
  float vq_V_ = 0.0f;
};

}  // namespace foc_ctrl
