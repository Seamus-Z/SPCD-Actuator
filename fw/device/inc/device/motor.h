// Motor electrical / control parameters (bring-up defaults).
// Source: DM4310.h (verify against the physical motor on the bench).
#pragma once

namespace device
{
namespace motor
{

// Generic parameter block used by open-loop voltage FOC and later loops.
struct Params
{
  float phase_resistance_ohm = 0.0f;   // per-phase R
  float phase_inductance_H = 0.0f;     // per-phase L
  // Line-to-line peak voltage per kRPM (mechanical), from vendor table.
  float bemf_Vpeak_per_krpm = 0.0f;
  float pole_pairs = 0.0f;             // electrical cycles per mechanical rev
  float max_phase_current_A = 0.0f;    // peak per phase
  float openloop_iq_A = 0.0f;          // suggested Iq when leaving open-loop
  float end_speed_rpm = 0.0f;          // open→closed transition hint
  float nominal_speed_rpm = 0.0f;
  float fw_speed_rpm = 0.0f;           // field-weakening ceiling (same as nominal here)
  float dc_bus_utilization = 0.0f;     // modulation index limit

  // Current-loop PI seeds from the vendor table (retune for our 15 kHz rate).
  float id_kp = 0.0f;
  float id_ki = 0.0f;
  float id_kc = 0.0f;
  float iq_kp = 0.0f;
  float iq_ki = 0.0f;
  float iq_kc = 0.0f;
  float speed_kp = 0.0f;
  float speed_ki = 0.0f;
  float speed_kc = 0.0f;
};

// Damiao DM4310 defaults (imported from DM4310.h).
// Note: that header's comment still says "Hurst300"; confirm on hardware.
inline constexpr Params kDm4310 = {
    .phase_resistance_ohm = 0.65f,
    .phase_inductance_H = 0.00034f,
    .bemf_Vpeak_per_krpm = 11.5f,
    .pole_pairs = 14.0f,
    .max_phase_current_A = 4.9f,
    .openloop_iq_A = 2.0f,
    .end_speed_rpm = 250.0f,
    .nominal_speed_rpm = 1200.0f,
    .fw_speed_rpm = 1909.8593f,  // 200 mechanical rad/s
    .dc_bus_utilization = 0.89f,
    .id_kp = 0.35f,
    .id_ki = 0.015f,
    .id_kc = 0.999f,
    .iq_kp = 0.35f,
    .iq_ki = 0.015f,
    .iq_kc = 0.999f,
    .speed_kp = 0.0054f,
    // Original table used ITERM = 1.56/20000 for a 20 kHz loop.
    .speed_ki = 1.56f / 20000.0f,
    .speed_kc = 1.0f,
};

// Active motor on this board (change when swapping motors).
inline constexpr const Params& kActive = kDm4310;
inline constexpr const char kActiveName[] = "DM4310";

// θ̇_elec [rad/s] from mechanical speed [rad/s].
inline float ElectricalOmegaFromMechanical(float omega_mech_rad_s,
                                           const Params& p = kActive)
{
  return omega_mech_rad_s * p.pole_pairs;
}

// θ̇_elec [rad/s] from mechanical RPM.
inline float ElectricalOmegaFromRpm(float rpm, const Params& p = kActive)
{
  constexpr float kTwoPi = 6.28318530718f;
  const float omega_mech = rpm * (kTwoPi / 60.0f);
  return ElectricalOmegaFromMechanical(omega_mech, p);
}

}  // namespace motor
}  // namespace device
