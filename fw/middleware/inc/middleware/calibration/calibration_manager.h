// Calibration business service: owns calibration sessions, applies results to
// runtime services, and defers flash writes out of the PWM ISR.
#pragma once

#include <cstddef>
#include <cstdint>

#include "HAL/phase_pwm.h"
#include "math/calibration/bemf_ident.h"
#include "math/calibration/cogging.h"
#include "math/calibration/encoder_phase.h"
#include "math/calibration/l_ident.h"
#include "math/calibration/r_ident.h"
#include "math/foc/controller.h"
#include "math/servo_mode/servo_mode.h"
#include "middleware/encoder/encoder_service.h"
#include "middleware/persistence/motor_calibration_store.h"

namespace middleware { namespace calibration {

class CalibrationManager {
 public:
  CalibrationManager(encoder::EncoderService* encoder,
                     math::foc::FocController* foc,
                     math::servo_mode::ServoMode* servo,
                     hal::PhasePwm* phase_pwm)
      : encoder_(encoder), foc_(foc), servo_(servo), phase_pwm_(phase_pwm) {}

  bool LoadPersistentConfig()
  {
    middleware::persistence::MotorCalibration data{};
    if (!middleware::persistence::MotorCalibrationStore::Load(&data))
    {
      return false;
    }
    if (encoder_ != nullptr)
    {
      encoder::EncoderService::Calibration calibration{};
      calibration.sign = data.sign;
      calibration.offset_rad = data.offset_rad;
      calibration.commutation_offset_rad = data.commutation_offset_rad;
      calibration.commutation_valid = data.commutation_valid;
      encoder_->SetCalibration(calibration);
      encoder_->SetCogging(data.cogging_table, data.cogging_scale,
                           data.cogging_valid);
      encoder_->SetCompensation(data.encoder_comp_table,
                                data.encoder_comp_scale,
                                data.encoder_comp_valid);
    }
    // Electrical R/L/Ke are owned by runtime Config (Motor table). Cal flash
    // keeps them for status / first-boot seed only — do not override FOC here.
    encoder_persisted_ = true;
    bemf_persisted_ = data.bemf_valid;
    resistance_persisted_ = data.resistance_valid;
    inductance_persisted_ = data.inductance_valid;
    cogging_persisted_ = data.cogging_valid;
    compensation_persisted_ = data.encoder_comp_valid;
    return true;
  }

  math::calibration::EncoderPhaseCal& encoder_phase() { return encoder_phase_; }
  math::calibration::BemfIdentCal& bemf() { return bemf_; }
  math::calibration::RIdentCal& resistance() { return resistance_; }
  math::calibration::LIdentCal& inductance() { return inductance_; }
  math::calibration::CoggingCal& cogging() { return cogging_; }
  const math::calibration::EncoderPhaseCal& encoder_phase() const { return encoder_phase_; }
  const math::calibration::BemfIdentCal& bemf() const { return bemf_; }
  const math::calibration::RIdentCal& resistance() const { return resistance_; }
  const math::calibration::LIdentCal& inductance() const { return inductance_; }
  const math::calibration::CoggingCal& cogging() const { return cogging_; }

  void Abort()
  {
    encoder_phase_.Stop();
    bemf_.Stop();
    resistance_.Stop();
    inductance_.Stop();
    cogging_.Stop();
    if (encoder_ != nullptr)
    {
      encoder_->RestoreCalibration();
    }
  }

  bool active() const
  {
    return encoder_phase_.active() || bemf_.active() || resistance_.active() ||
           inductance_.active() || cogging_.active();
  }

  void SetLastKind(uint8_t kind) { last_kind_ = kind; }
  uint8_t last_kind() const { return last_kind_; }

  void ApplyEncoderResult()
  {
    const auto& result = encoder_phase_.result();
    if (!result.ok || encoder_ == nullptr)
    {
      return;
    }
    encoder::EncoderService::Calibration calibration{};
    calibration.sign = result.sign;
    calibration.offset_rad = result.offset_rad;
    calibration.commutation_offset_rad = result.commutation_offset_rad;
    calibration.commutation_valid = result.commutation_valid;
    encoder_->ApplyCalibration(calibration);
    encoder_dirty_ = true;
    encoder_persisted_ = false;
  }

  void ApplyBemfResult()
  {
    const auto& result = bemf_.result();
    if (!result.ok || result.ke_v_s_per_rad <= 0.005f)
    {
      return;
    }
    if (foc_ != nullptr)
    {
      foc_->SetVPerHz(result.ke_v_s_per_rad);
    }
    if (servo_ != nullptr)
    {
      servo_->SetTorqueConstant(1.5f * result.ke_v_s_per_rad);
    }
    bemf_dirty_ = true;
    bemf_persisted_ = false;
  }

  void ApplyResistanceResult()
  {
    const auto& result = resistance_.result();
    if (!result.ok || result.resistance_ohm <= 0.001f || foc_ == nullptr)
    {
      return;
    }
    foc_->SetResistanceInductance(result.resistance_ohm, foc_->inductance_d_H(),
                                  foc_->inductance_q_H());
    resistance_dirty_ = true;
    resistance_persisted_ = false;
  }

  void ApplyInductanceResult()
  {
    const auto& result = inductance_.result();
    if (!result.ok || result.inductance_d_H <= 0.0f ||
        result.inductance_q_H <= 0.0f || foc_ == nullptr)
    {
      return;
    }
    foc_->SetResistanceInductance(foc_->resistance_ohm(), result.inductance_d_H,
                                  result.inductance_q_H);
    inductance_dirty_ = true;
    inductance_persisted_ = false;
  }

  void ApplyCoggingResult()
  {
    const auto& result = cogging_.result();
    if (!result.ok || encoder_ == nullptr)
    {
      return;
    }
    encoder_->SetCogging(result.table, result.scale, result.scale > 0.0f);
    cogging_dirty_ = true;
    cogging_persisted_ = false;
  }

  void MarkCompensationDirty()
  {
    compensation_dirty_ = true;
    compensation_persisted_ = false;
  }

  bool StartEncoderPhase(const math::calibration::EncoderPhaseCal::Options& options)
  {
    if (encoder_ == nullptr) { return false; }
    encoder_->BeginCalibration();
    encoder_persisted_ = false;
    encoder_phase_.Start(options);
    return true;
  }

  void ClearCompensation()
  {
    encoder_->ClearCompensation();
    MarkCompensationDirty();
  }

  bool SetCompensationChunk(uint8_t chunk, const int8_t* data, size_t len)
  {
    return encoder_->SetCompensationChunk(chunk, data, len);
  }

  bool CommitCompensation(float scale_rad)
  {
    if (!encoder_->CommitCompensation(scale_rad)) { return false; }
    MarkCompensationDirty();
    return true;
  }

  // True when PersistPending() would erase/write flash on its next call.
  // The caller must cut motor output first: page erase stalls the CPU for
  // tens of ms while the PWM timer keeps driving the last duty cycle.
  bool has_pending_persist() const
  {
    return encoder_dirty_ || bemf_dirty_ || resistance_dirty_ ||
           inductance_dirty_ || cogging_dirty_ || compensation_dirty_;
  }

  void PersistPending()
  {
    if (!has_pending_persist())
    {
      return;
    }
    middleware::persistence::MotorCalibration data{};
    middleware::persistence::MotorCalibrationStore::Load(&data);
    if (encoder_dirty_ && encoder_ != nullptr)
    {
      const auto calibration = encoder_->calibration();
      data.offset_rad = calibration.offset_rad;
      data.sign = calibration.sign;
      data.commutation_offset_rad = calibration.commutation_offset_rad;
      data.commutation_valid = calibration.commutation_valid;
      data.commutation_residual_rad = encoder_phase_.result().residual_rad_rms;
    }
    if (bemf_dirty_)
    {
      data.bemf_v_per_hz = bemf_.result().ke_v_s_per_rad;
      data.bemf_valid = true;
    }
    if (resistance_dirty_)
    {
      data.resistance_ohm = resistance_.result().resistance_ohm;
      data.resistance_valid = true;
    }
    if (inductance_dirty_)
    {
      data.inductance_d_H = inductance_.result().inductance_d_H;
      data.inductance_q_H = inductance_.result().inductance_q_H;
      data.inductance_valid = true;
    }
    if (cogging_dirty_ && encoder_ != nullptr)
    {
      data.cogging_table = encoder_->cogging_table();
      data.cogging_scale = encoder_->cogging_scale_A();
      data.cogging_valid = encoder_->cogging_valid();
    }
    if (compensation_dirty_ && encoder_ != nullptr)
    {
      data.encoder_comp_table = encoder_->compensation_table();
      data.encoder_comp_scale = encoder_->compensation_scale_rad();
      data.encoder_comp_valid = encoder_->compensation_valid();
    }
    if (phase_pwm_ != nullptr)
    {
      phase_pwm_->DisableControlIsr();
    }
    const bool saved = middleware::persistence::MotorCalibrationStore::Save(data);
    if (encoder_dirty_) encoder_persisted_ = saved;
    if (bemf_dirty_) bemf_persisted_ = saved;
    if (resistance_dirty_) resistance_persisted_ = saved;
    if (inductance_dirty_) inductance_persisted_ = saved;
    if (cogging_dirty_) cogging_persisted_ = saved;
    if (compensation_dirty_) compensation_persisted_ = saved;
    encoder_dirty_ = false;
    bemf_dirty_ = false;
    resistance_dirty_ = false;
    inductance_dirty_ = false;
    cogging_dirty_ = false;
    compensation_dirty_ = false;
  }

  bool encoder_persisted() const { return encoder_persisted_; }
  bool bemf_persisted() const { return bemf_persisted_; }
  bool resistance_persisted() const { return resistance_persisted_; }
  bool inductance_persisted() const { return inductance_persisted_; }
  bool cogging_persisted() const { return cogging_persisted_; }
  bool compensation_persisted() const { return compensation_persisted_; }

 private:
  encoder::EncoderService* encoder_ = nullptr;
  math::foc::FocController* foc_ = nullptr;
  math::servo_mode::ServoMode* servo_ = nullptr;
  hal::PhasePwm* phase_pwm_ = nullptr;
  math::calibration::EncoderPhaseCal encoder_phase_{};
  math::calibration::BemfIdentCal bemf_{};
  math::calibration::RIdentCal resistance_{};
  math::calibration::LIdentCal inductance_{};
  math::calibration::CoggingCal cogging_{};
  bool encoder_dirty_ = false;
  bool bemf_dirty_ = false;
  bool resistance_dirty_ = false;
  bool inductance_dirty_ = false;
  bool cogging_dirty_ = false;
  bool compensation_dirty_ = false;
  bool encoder_persisted_ = false;
  bool bemf_persisted_ = false;
  bool resistance_persisted_ = false;
  bool inductance_persisted_ = false;
  bool cogging_persisted_ = false;
  bool compensation_persisted_ = false;
  uint8_t last_kind_ = 0;
};

}  // namespace calibration
}  // namespace middleware
