// Board-level wiring and electrical defaults for the application.
#pragma once

#include "HAL/fdcan.h"
#include "HAL/phase_current_adc.h"
#include "HAL/phase_pwm.h"
#include "foc_ctrl/current_loop.h"
#include "foc_ctrl/encoder_pll.h"
#include "foc_ctrl/position_loop.h"
#include "foc_ctrl/voltage_foc.h"
#include "math/constants.h"
#include "device/drv8353s.h"
#include "device/ma600.h"
#include "device/motor.h"

namespace app
{
namespace board
{

// CAN: FDCAN2 @ 1M/2M FD, matches moteus-x1 wiring (PB5 RX / PB6 TX).
inline hal::FDCan::Options CanOptions()
{
  hal::FDCan::Options options;
  options.instance = FDCAN2;
  options.slow_bitrate = 1000000;
  options.fast_bitrate = 2000000;
  options.fdcan_frame = true;
  options.bitrate_switch = true;
  options.automatic_retransmission = false;
  return options;
}

// Gate driver SPI / control pins (moteus-x1 / family 3).
inline device::Drv8353s::Options GateDriverOptions()
{
  device::Drv8353s::Options options;
  options.mosi = PC_13;
  options.miso = PC_11;
  options.sck = PC_10;
  options.cs = PB_0;
  options.enable = PC_14;
  options.fault = PB_13;
  options.hiz = PC_15;
  options.spi_frequency_hz = 1000000;
  return options;
}

// Gate drive / OCP / CSA register programming values.
inline device::Drv8353s::Config GateDriverConfig()
{
  device::Drv8353s::Config config;
  config.pwm_mode = device::Drv8353s::PwmMode::PWM_3X;
  config.source_current_hs_ma = 300;
  config.sink_current_hs_ma = 200;
  config.source_current_ls_ma = 850;
  config.sink_current_ls_ma = 1200;
  config.peak_drive_time_ns = 1000;
  config.dead_time_ns = 50;
  config.vds_threshold_mv = 700;
  config.csa_gain = 20;
  return config;
}

// Phase current sense (moteus-x1 / family 3).
inline hal::PhaseCurrentAdc::Options CurrentSenseOptions()
{
  hal::PhaseCurrentAdc::Options options;
  options.current1 = PA_3;   // ADC1_IN4 / FCUR1
  options.current2 = PA_6;   // ADC2_IN3 / FCUR2
  options.current3 = PB_1;   // ADC3_IN1 / FCUR3
  options.sense_ohm = 0.0005f;
  options.csa_gain = 20.0f;
  options.sample_time_code = 1;  // 6.5 cycles
  return options;
}

// Phase PWM (moteus-x1 / family 3): TIM5 @ 15 kHz center-aligned.
inline hal::PhasePwm::Options PhasePwmOptions()
{
  hal::PhasePwm::Options options;
  options.pwm1 = PA_0_ALT0;
  options.pwm2 = PA_1_ALT0;
  options.pwm3 = PA_2_ALT0;
  options.rate_hz = 15000;
  return options;
}

// Active motor parameter table (DM4310 by default).
inline const device::motor::Params& MotorParams()
{
  return device::motor::kActive;
}

// Nominal DC bus for open-loop voltage→PWM scaling (no bus ADC yet).
inline constexpr float kBusVoltage_V = 48.0f;

// AUX2 MA600. vel/dq closed-loop use encoder θ_e for commutation.
inline device::Ma600::Options Ma600Options()
{
  device::Ma600::Options options;
  options.mosi = PA_11;
  options.miso = PA_10;
  options.sck = PF_1;
  options.cs = PB_7;
  options.frequency_hz = 6000000;
  options.filter_us = 64;
  options.sign = 1.0f;
  options.offset_rad = 0.0f;
  return options;
}

inline foc_ctrl::EncoderPll::Options EncoderPllOptions()
{
  foc_ctrl::EncoderPll::Options options;
  options.pll_filter_hz = 200.0f;
  options.pole_pairs = MotorParams().pole_pairs;
  return options;
}

inline foc_ctrl::VoltageFoc::Options VoltageFocOptions()
{
  foc_ctrl::VoltageFoc::Options options;
  options.bus_V = kBusVoltage_V;
  options.min_duty = static_cast<float>(hal::PhasePwm::kDutyMinMilli) / 1000.0f;
  options.max_duty = static_cast<float>(hal::PhasePwm::kDutyMaxMilli) / 1000.0f;
  return options;
}

// Vendor Ke is line-to-line peak. CurrentLoop uses phase/dq peak voltage,
// therefore convert by 1/sqrt(3) before applying BEMF feedforward.
inline float BemfVPerMechRadS(const device::motor::Params& motor)
{
  return motor.bemf_Vpeak_per_krpm * (60.0f / 1000.0f) /
      (math::k2Pi * math::kSqrt3);
}

// For the amplitude-invariant 2/3 Park transform, Kt = 3/2 * Ke_dq.
inline float VendorTorqueConstantNmPerA(const device::motor::Params& motor)
{
  return 1.5f * BemfVPerMechRadS(motor);
}

// BemfIdentCal measures phase/dq Vq per mechanical rad/s. With the
// amplitude-invariant 2/3 Park transform, Kt = 3/2 * Ke_dq.
inline float IdentifiedTorqueConstantNmPerA(float ke_dq_v_s_per_rad)
{
  return 1.5f * ke_dq_v_s_per_rad;
}

inline foc_ctrl::CurrentLoop::Options CurrentLoopOptions()
{
  const auto& motor = MotorParams();
  foc_ctrl::CurrentLoop::Options options;
  options.bus_V = kBusVoltage_V;
  options.min_duty = static_cast<float>(hal::PhasePwm::kDutyMinMilli) / 1000.0f;
  options.max_duty = static_cast<float>(hal::PhasePwm::kDutyMaxMilli) / 1000.0f;
  constexpr float kCurrentLoopHz = 200.0f;
  const float current_loop_w = math::k2Pi * kCurrentLoopHz;
  options.kp_d = current_loop_w * motor.phase_inductance_H;
  options.kp_q = current_loop_w * motor.phase_inductance_H;
  options.ki_d = current_loop_w * motor.phase_resistance_ohm;
  options.ki_q = current_loop_w * motor.phase_resistance_ohm;
  options.max_current_desired_rate_A_s = 10000.0f;
  options.max_current_A = motor.max_phase_current_A;
  options.resistance_ohm = motor.phase_resistance_ohm;
  options.inductance_d_H = motor.phase_inductance_H;
  options.inductance_q_H = motor.phase_inductance_H;
  options.v_per_hz = BemfVPerMechRadS(motor);
  options.bemf_feedforward = 1.0f;
  // Transport delay ~= 1.5 PWM periods (sample->apply) plus the MA600 group
  // delay; advance commutation by omega_elec * this to keep Id from leaking
  // at speed. Tune by watching Id at high speed if needed.
  options.phase_lead_s =
      1.5f / static_cast<float>(PhasePwmOptions().rate_hz) + 64.0e-6f;
  return options;
}

// Position/velocity gains from the known-good moteus setup for this motor.
inline foc_ctrl::PositionLoop::Options PositionLoopOptions()
{
  const auto& motor = MotorParams();
  foc_ctrl::PositionLoop::Options options;
  options.pid.kp = 0.04f;
  options.pid.ki = 0.003f;
  options.pid.kd = 0.02f;
  options.pid.ilimit = 0.3f;
  options.pid.sign = -1;
  options.torque_constant_Nm_A = VendorTorqueConstantNmPerA(motor);
  if (options.torque_constant_Nm_A < 0.05f)
  {
    options.torque_constant_Nm_A = 0.1f;
  }
  options.max_iq_A = 3.0f;
  if (options.max_iq_A > motor.max_phase_current_A)
  {
    options.max_iq_A = motor.max_phase_current_A;
  }
  options.max_torque_Nm =
      options.torque_constant_Nm_A * options.max_iq_A;
  options.acceleration_limit_rad_s2 = 15.0f * math::k2Pi;
  // 200 rad/s is below the 48 V DQ voltage limit for this motor. Retain an
  // explicit ceiling so invalid host commands cannot accelerate without bound.
  options.max_velocity_cmd_rad_s =
      motor.fw_speed_rpm * (math::k2Pi / 60.0f);
  options.max_position_slip_rad = math::kPi;
  options.max_velocity_error_rad_s = 100.0f;
  return options;
}

}  // namespace board
}  // namespace app
