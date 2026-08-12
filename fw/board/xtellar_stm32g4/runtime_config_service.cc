#include "board/xtellar_stm32g4/runtime_config_service.h"

#include "board/xtellar_stm32g4/board_config.h"
#include "device/motor.h"
#include "math/constants.h"
#include "middleware/persistence/motor_calibration_store.h"
#include "middleware/persistence/runtime_config_store.h"

namespace app::board
{

RuntimeConfigService::RuntimeConfigService(const Dependencies& dependencies)
    : encoder_(dependencies.encoder),
      calibration_(dependencies.calibration),
      motor_control_(dependencies.motor_control),
      dq_modulator_(dependencies.dq_modulator),
      foc_(dependencies.foc),
      servo_(dependencies.servo),
      mit_(dependencies.mit)
{
  if (motor_control_ != nullptr)
  {
    motor_control_->SetCalibrationResultSink(this);
  }
}

bool RuntimeConfigService::Initialize()
{
  FillDefaults();
  flash_valid_ = middleware::persistence::RuntimeConfigStore::Load(&config_);
  if (!flash_valid_)
  {
    SeedMotorElectricalFromCalibrationFlash();
  }
  Apply();
  ReloadPersistentCalibration();
  return true;
}

bool RuntimeConfigService::Save()
{
  if (motor_control_ != nullptr)
  {
    motor_control_->Stop();
  }
  if (!middleware::persistence::RuntimeConfigStore::Save(config_))
  {
    return false;
  }
  flash_valid_ = true;
  return true;
}

bool RuntimeConfigService::Load()
{
  middleware::config::RuntimeConfig loaded{};
  if (!middleware::persistence::RuntimeConfigStore::Load(&loaded))
  {
    flash_valid_ = false;
    return false;
  }
  config_ = loaded;
  flash_valid_ = true;
  Apply();
  ReloadPersistentCalibration();
  return true;
}

void RuntimeConfigService::RestoreDefaults()
{
  FillDefaults();
  Apply();
  ReloadPersistentCalibration();
}

void RuntimeConfigService::ReloadPersistentCalibration()
{
  if (calibration_ != nullptr)
  {
    (void)calibration_->LoadPersistentConfig();
  }
}

void RuntimeConfigService::SeedMotorElectricalFromCalibrationFlash()
{
  middleware::persistence::MotorCalibration calibration{};
  if (!middleware::persistence::MotorCalibrationStore::Load(&calibration))
  {
    return;
  }
  if (calibration.resistance_valid && calibration.resistance_ohm > 0.001f)
  {
    config_.motor.resistance_ohm = calibration.resistance_ohm;
  }
  if (calibration.inductance_valid)
  {
    float inductance = 0.0f;
    if (calibration.inductance_d_H > 0.0f &&
        calibration.inductance_q_H > 0.0f)
    {
      inductance = 0.5f * (calibration.inductance_d_H +
                           calibration.inductance_q_H);
    }
    else if (calibration.inductance_d_H > 0.0f)
    {
      inductance = calibration.inductance_d_H;
    }
    else
    {
      inductance = calibration.inductance_q_H;
    }
    if (inductance > 0.0f)
    {
      config_.motor.inductance_H = inductance;
    }
  }
  if (calibration.bemf_valid && calibration.bemf_v_per_hz > 0.005f)
  {
    config_.motor.bemf_Vpeak_per_krpm =
        BemfVpeakPerKrpmFromDqKe(calibration.bemf_v_per_hz);
  }
}

void RuntimeConfigService::AcceptResistanceCalibration()
{
  if (calibration_ == nullptr) return;
  const auto& result = calibration_->resistance().result();
  if (result.ok && result.resistance_ohm > 0.001f)
  {
    config_.motor.resistance_ohm = result.resistance_ohm;
  }
}

void RuntimeConfigService::AcceptInductanceCalibration()
{
  if (calibration_ == nullptr) return;
  const auto& result = calibration_->inductance().result();
  if (result.ok && result.inductance_d_H > 0.0f &&
      result.inductance_q_H > 0.0f)
  {
    config_.motor.inductance_H =
        0.5f * (result.inductance_d_H + result.inductance_q_H);
  }
}

void RuntimeConfigService::AcceptBemfCalibration()
{
  if (calibration_ == nullptr) return;
  const auto& result = calibration_->bemf().result();
  if (result.ok && result.ke_v_s_per_rad > 0.005f)
  {
    config_.motor.bemf_Vpeak_per_krpm =
        BemfVpeakPerKrpmFromDqKe(result.ke_v_s_per_rad);
  }
}

void RuntimeConfigService::FillDefaults()
{
  const auto& motor = MotorParams();
  config_.motor.pole_pairs = motor.pole_pairs;
  config_.motor.resistance_ohm = motor.phase_resistance_ohm;
  config_.motor.inductance_H = motor.phase_inductance_H;
  config_.motor.bemf_Vpeak_per_krpm = motor.bemf_Vpeak_per_krpm;
  config_.motor.max_phase_current_A = motor.max_phase_current_A;
  config_.motor.fw_speed_rad_s =
      motor.fw_speed_rpm * (math::k2Pi / 60.0f);
  config_.motor.bus_V = kBusVoltage_V;
  config_.motor.reserved0 = 0.0f;

  config_.foc.bandwidth_hz = 200.0f;
  config_.foc.bemf_feedforward = 1.0f;
  config_.foc.current_feedforward = 1.0f;
  config_.foc.cross_coupling_feedforward = 1.0f;
  config_.foc.max_current_desired_rate_A_s = 10000.0f;
  config_.foc.reserved0 = 0.0f;

  const auto servo = ServoModeOptions();
  config_.servo.kp = servo.pid.kp;
  config_.servo.ki = servo.pid.ki;
  config_.servo.kd = servo.pid.kd;
  config_.servo.ilimit = servo.pid.ilimit;
  config_.servo.max_iq_A = servo.max_iq_A;
  config_.servo.velocity_threshold = servo.velocity_threshold;
  config_.servo.max_position_slip_rad = servo.max_position_slip_rad;
  config_.servo.max_velocity_error_rad_s = servo.max_velocity_error_rad_s;
  config_.servo.default_velocity_limit_rad_s =
      servo.default_velocity_limit_rad_s;
  config_.servo.default_accel_limit_rad_s2 =
      servo.default_accel_limit_rad_s2;
  config_.servo.sign_f = servo.pid.sign < 0 ? -1.0f : 1.0f;
  config_.servo.reserved0 = 0.0f;

  config_.encoder.pll_filter_hz = 400.0f;
  config_.encoder.spike_error_rad = 0.15f;
  config_.encoder.filter_us = static_cast<float>(Ma600Config().filter_us);
  config_.encoder.reserved0 = 0.0f;
}

void RuntimeConfigService::Apply()
{
  auto& motor = config_.motor;
  auto& foc_config = config_.foc;
  auto& servo_config = config_.servo;
  auto& encoder_config = config_.encoder;

  if (motor.pole_pairs < 1.0f) motor.pole_pairs = 1.0f;
  if (motor.resistance_ohm < 0.001f) motor.resistance_ohm = 0.001f;
  if (motor.inductance_H < 1.0e-7f) motor.inductance_H = 1.0e-7f;
  if (motor.max_phase_current_A < 0.1f) motor.max_phase_current_A = 0.1f;
  if (motor.fw_speed_rad_s < 1.0f) motor.fw_speed_rad_s = 1.0f;
  if (motor.bus_V < 1.0f) motor.bus_V = 1.0f;
  if (foc_config.bandwidth_hz < 1.0f) foc_config.bandwidth_hz = 1.0f;
  if (encoder_config.pll_filter_hz < 1.0f)
    encoder_config.pll_filter_hz = 1.0f;
  if (encoder_config.filter_us < 0.0f) encoder_config.filter_us = 0.0f;
  if (encoder_config.filter_us > 10240.0f)
    encoder_config.filter_us = 10240.0f;

  const uint16_t filter_us =
      static_cast<uint16_t>(encoder_config.filter_us + 0.5f);
  if (encoder_ != nullptr)
  {
    (void)encoder_->SetSensorFilterUs(filter_us);
    math::servo_mode::EncoderPll::Options pll = EncoderPllOptions();
    pll.pll_filter_hz = encoder_config.pll_filter_hz;
    pll.pole_pairs = motor.pole_pairs;
    pll.spike_error_rad = encoder_config.spike_error_rad;
    pll.max_velocity_mech_rad_s = motor.fw_speed_rad_s * 1.5f;
    encoder_->SetPllOptions(pll);
  }

  if (dq_modulator_ != nullptr)
  {
    dq_modulator_->SetBusVoltage(motor.bus_V);
  }
  if (motor_control_ != nullptr)
  {
    motor_control_->SetOvercurrentTrip(motor.max_phase_current_A * 1.2f);
  }

  if (foc_ != nullptr)
  {
    foc_->SetBusVoltage(motor.bus_V);
    foc_->SetMaxCurrent(motor.max_phase_current_A);
    foc_->SetFeedforwards(foc_config.bemf_feedforward,
                          foc_config.current_feedforward,
                          foc_config.cross_coupling_feedforward);
    foc_->SetMaxCurrentDesiredRate(
        foc_config.max_current_desired_rate_A_s);
    foc_->SetPhaseLead(1.5f / static_cast<float>(PhasePwmOptions().rate_hz) +
                       static_cast<float>(filter_us) * 1.0e-6f);
    foc_->SetResistanceInductance(motor.resistance_ohm, motor.inductance_H,
                                  motor.inductance_H);
    device::motor::Params parameters = MotorParams();
    parameters.bemf_Vpeak_per_krpm = motor.bemf_Vpeak_per_krpm;
    parameters.phase_resistance_ohm = motor.resistance_ohm;
    parameters.phase_inductance_H = motor.inductance_H;
    parameters.pole_pairs = motor.pole_pairs;
    parameters.max_phase_current_A = motor.max_phase_current_A;
    foc_->SetVPerHz(BemfVPerMechRadS(parameters));
    foc_->SetBandwidthHz(foc_config.bandwidth_hz);
  }

  if (servo_ != nullptr)
  {
    auto options = ServoModeOptions();
    options.pid.kp = servo_config.kp;
    options.pid.ki = servo_config.ki;
    options.pid.kd = servo_config.kd;
    options.pid.ilimit = servo_config.ilimit;
    options.pid.sign = servo_config.sign_f < 0.0f ? -1 : 1;
    options.max_iq_A = servo_config.max_iq_A;
    if (options.max_iq_A > motor.max_phase_current_A)
      options.max_iq_A = motor.max_phase_current_A;
    options.velocity_threshold = servo_config.velocity_threshold;
    options.max_position_slip_rad = servo_config.max_position_slip_rad;
    options.max_velocity_error_rad_s = servo_config.max_velocity_error_rad_s;
    options.default_velocity_limit_rad_s =
        servo_config.default_velocity_limit_rad_s;
    options.default_accel_limit_rad_s2 =
        servo_config.default_accel_limit_rad_s2;
    options.max_velocity_cmd_rad_s = motor.fw_speed_rad_s;
    device::motor::Params parameters = MotorParams();
    parameters.bemf_Vpeak_per_krpm = motor.bemf_Vpeak_per_krpm;
    options.torque_constant_Nm_A = VendorTorqueConstantNmPerA(parameters);
    if (options.torque_constant_Nm_A < 0.05f)
      options.torque_constant_Nm_A = 0.1f;
    options.max_torque_Nm = options.torque_constant_Nm_A * options.max_iq_A;
    servo_->SetOptions(options);
  }

  if (mit_ != nullptr)
  {
    auto options = MitModeOptions();
    options.max_iq_A = servo_config.max_iq_A;
    if (options.max_iq_A > motor.max_phase_current_A)
      options.max_iq_A = motor.max_phase_current_A;
    device::motor::Params parameters = MotorParams();
    parameters.bemf_Vpeak_per_krpm = motor.bemf_Vpeak_per_krpm;
    options.torque_constant_Nm_A = VendorTorqueConstantNmPerA(parameters);
    if (options.torque_constant_Nm_A < 0.05f)
      options.torque_constant_Nm_A = 0.1f;
    options.max_torque_Nm = options.torque_constant_Nm_A * options.max_iq_A;
    mit_->SetOptions(options);
  }
}

}  // namespace app::board
