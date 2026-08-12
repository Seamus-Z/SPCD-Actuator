// Runtime motor-control configuration shared by application services and persistence.
#pragma once

namespace middleware::config
{

// Layouts intentionally match protocol::xt_can::*Conf wire structures.
struct MotorConf
{
  float pole_pairs = 14.0f;
  float resistance_ohm = 0.65f;
  float inductance_H = 0.00034f;
  float bemf_Vpeak_per_krpm = 11.5f;
  float max_phase_current_A = 4.9f;
  float fw_speed_rad_s = 200.0f;
  float bus_V = 48.0f;
  float reserved0 = 0.0f;
} __attribute__((packed));

struct FocConf
{
  float bandwidth_hz = 200.0f;
  float bemf_feedforward = 1.0f;
  float current_feedforward = 1.0f;
  float cross_coupling_feedforward = 1.0f;
  float max_current_desired_rate_A_s = 10000.0f;
  float reserved0 = 0.0f;
} __attribute__((packed));

struct ServoConf
{
  float kp = 4.0f;
  float ki = 0.0f;
  float kd = 0.05f;
  float ilimit = 0.0f;
  float max_iq_A = 3.0f;
  float velocity_threshold = 0.5f;
  float max_position_slip_rad = 3.14159265358979323846f;
  float max_velocity_error_rad_s = 0.0f;
  float default_velocity_limit_rad_s = 200.0f;
  float default_accel_limit_rad_s2 = 0.0f;
  float sign_f = -1.0f;
  float reserved0 = 0.0f;
} __attribute__((packed));

struct EncoderConf
{
  float pll_filter_hz = 400.0f;
  float spike_error_rad = 0.15f;
  float filter_us = 160.0f;
  float reserved0 = 0.0f;
} __attribute__((packed));

struct RuntimeConfig
{
  MotorConf motor{};
  FocConf foc{};
  ServoConf servo{};
  EncoderConf encoder{};
};

static_assert(sizeof(MotorConf) == 32, "MotorConf size");
static_assert(sizeof(FocConf) == 24, "FocConf size");
static_assert(sizeof(ServoConf) == 48, "ServoConf size");
static_assert(sizeof(EncoderConf) == 16, "EncoderConf size");

}  // namespace middleware::config
