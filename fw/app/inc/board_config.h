// Board-level wiring and electrical defaults for the application.
#pragma once

#include "HAL/fdcan.h"
#include "HAL/phase_current_adc.h"
#include "HAL/phase_pwm.h"
#include "foc_ctrl/current_loop.h"
#include "foc_ctrl/voltage_foc.h"
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

// AUX2 MA600 defaults (readout only; not used for commutation yet).
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

inline foc_ctrl::VoltageFoc::Options VoltageFocOptions()
{
  foc_ctrl::VoltageFoc::Options options;
  options.bus_V = kBusVoltage_V;
  options.min_duty = static_cast<float>(hal::PhasePwm::kDutyMinMilli) / 1000.0f;
  options.max_duty = static_cast<float>(hal::PhasePwm::kDutyMaxMilli) / 1000.0f;
  return options;
}

inline foc_ctrl::CurrentLoop::Options CurrentLoopOptions()
{
  const auto& motor = MotorParams();
  foc_ctrl::CurrentLoop::Options options;
  options.bus_V = kBusVoltage_V;
  options.min_duty = static_cast<float>(hal::PhasePwm::kDutyMinMilli) / 1000.0f;
  options.max_duty = static_cast<float>(hal::PhasePwm::kDutyMaxMilli) / 1000.0f;
  options.kp_d = motor.id_kp;
  options.kp_q = motor.iq_kp;
  // Vendor I-gains are per PWM cycle @ ~15–20 kHz → continuous 1/s.
  constexpr float kPwmHz = 15000.0f;
  options.ki_d = motor.id_ki * kPwmHz;
  options.ki_q = motor.iq_ki * kPwmHz;
  // Soft command clamp = motor peak (DM4310 table).
  options.max_current_A = motor.max_phase_current_A;
  return options;
}

}  // namespace board
}  // namespace app
