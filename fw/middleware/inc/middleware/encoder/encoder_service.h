// Encoder business service: calibration, compensation, sampling pipeline, and PLL.
// Device drivers expose only raw sensor transfers; motor-angle semantics live here.
#pragma once

#include <cstddef>
#include <cstdint>

#include "math/cogging.h"
#include "math/commutation.h"
#include "math/constants.h"
#include "math/encoder_comp.h"
#include "math/servo_mode/encoder_pll.h"
#include "ports/angle_sensor.h"

namespace middleware { namespace encoder {

class EncoderService {
 public:
  struct Options {
    math::servo_mode::EncoderPll::Options pll;
  };

  struct Calibration {
    float sign = 1.0f;
    float offset_rad = 0.0f;
    math::CommutationTable commutation_offset_rad{};
    bool commutation_valid = false;
  };

  struct Sample {
    uint16_t raw = 0;
    float counts_rad = 0.0f;
    float mechanical_rad = 0.0f;
    float compensated_mechanical_rad = 0.0f;
    float electrical_rad = 0.0f;
    float mechanical_velocity_rad_s = 0.0f;
    float electrical_velocity_rad_s = 0.0f;
    bool valid = false;
  };

  EncoderService(ports::IAngleSensor* sensor, const Options& options)
      : sensor_(sensor), pll_(options.pll) {}

  bool Init()
  {
    if (sensor_ == nullptr || !sensor_->Init())
    {
      return false;
    }
    SampleBlocking();
    return true;
  }

  // Non-ISR callers may synchronously acquire the angle.
  void SampleBlocking()
  {
    if (sensor_ == nullptr)
    {
      sample_.valid = false;
      return;
    }
    spi_pending_ = false;
    sensor_->Sample();
    UpdateSampleFromRaw();
    if (pll_.theta_valid())
    {
      UpdatePllOutput();
    }
    else
    {
      sample_.electrical_rad = math::WrapZeroToTwoPi(
          sample_.compensated_mechanical_rad * pole_pairs());
    }
  }

  // ISR order: Finish previous SPI transfer, update PLL, begin next transfer.
  void UpdatePwmIsr(float dt_s)
  {
    if (sensor_ == nullptr)
    {
      sample_.valid = false;
      return;
    }
    if (spi_pending_)
    {
      sensor_->FinishSample();
      spi_pending_ = false;
    }
    else
    {
      sensor_->Sample();
    }
    UpdateSampleFromRaw();
    pll_.Update(sample_.compensated_mechanical_rad, dt_s);
    UpdatePllOutput();
    sensor_->StartSample();
    spi_pending_ = true;
  }

  void ResetPll()
  {
    pll_.Reset(sample_.compensated_mechanical_rad, 0.0f);
    UpdatePllOutput();
  }

  bool valid() const { return sensor_ != nullptr && sensor_->ok(); }
  bool pll_valid() const { return pll_.theta_valid(); }
  const Sample& sample() const { return sample_; }
  const math::servo_mode::EncoderPll& pll() const { return pll_; }

  void SetPllOptions(const math::servo_mode::EncoderPll::Options& options)
  {
    pll_.SetOptions(options);
  }

  bool SetSensorFilterUs(uint16_t filter_us)
  {
    if (sensor_ == nullptr)
    {
      return false;
    }
    return sensor_->SetFilterUs(filter_us);
  }

  Calibration calibration() const { return calibration_; }

  void SetCalibration(const Calibration& calibration)
  {
    calibration_ = calibration;
    calibration_.sign = calibration_.sign >= 0.0f ? 1.0f : -1.0f;
    UpdateSampleFromRaw();
    ResetPll();
  }

  void BeginCalibration()
  {
    calibration_backup_ = calibration_;
    calibration_backup_valid_ = true;
    Calibration raw{};
    raw.sign = 1.0f;
    SetCalibration(raw);
  }

  void RestoreCalibration()
  {
    if (!calibration_backup_valid_)
    {
      return;
    }
    SetCalibration(calibration_backup_);
    calibration_backup_valid_ = false;
  }

  void ApplyCalibration(const Calibration& calibration)
  {
    SetCalibration(calibration);
    calibration_backup_valid_ = false;
  }

  void ClearCompensation()
  {
    compensation_.fill(0);
    compensation_scale_rad_ = 0.0f;
    compensation_valid_ = false;
    compensation_chunk_mask_ = 0;
  }

  bool SetCompensationChunk(uint8_t chunk, const int8_t* data, size_t len)
  {
    constexpr size_t kChunkSize = 32u;
    if (data == nullptr || chunk >= 8u || len < kChunkSize)
    {
      return false;
    }
    const size_t base = static_cast<size_t>(chunk) * kChunkSize;
    for (size_t i = 0; i < kChunkSize; ++i)
    {
      compensation_[base + i] = data[i];
    }
    compensation_chunk_mask_ = static_cast<uint8_t>(
        compensation_chunk_mask_ | static_cast<uint8_t>(1u << chunk));
    return true;
  }

  bool CommitCompensation(float scale_rad)
  {
    if (scale_rad <= 0.0f || compensation_chunk_mask_ != 0xFFu)
    {
      compensation_valid_ = false;
      compensation_scale_rad_ = 0.0f;
      return false;
    }
    compensation_scale_rad_ = scale_rad;
    compensation_valid_ = true;
    UpdateSampleFromRaw();
    return true;
  }

  void SetCompensation(const math::EncoderCompTable& table, float scale_rad,
                       bool valid)
  {
    compensation_ = table;
    compensation_scale_rad_ = scale_rad;
    compensation_valid_ = valid && scale_rad > 0.0f;
    compensation_chunk_mask_ = compensation_valid_ ? 0xFFu : 0u;
    UpdateSampleFromRaw();
  }

  const math::EncoderCompTable& compensation_table() const
  {
    return compensation_;
  }
  float compensation_scale_rad() const { return compensation_scale_rad_; }
  bool compensation_valid() const { return compensation_valid_; }

  void SetCogging(const math::CoggingTable& table, float scale_A, bool valid)
  {
    cogging_ = table;
    cogging_scale_A_ = scale_A;
    cogging_valid_ = valid && scale_A > 0.0f;
  }

  const math::CoggingTable& cogging_table() const { return cogging_; }
  float cogging_scale_A() const { return cogging_scale_A_; }
  bool cogging_valid() const { return cogging_valid_; }

  float CoggingCurrentCompensation() const
  {
    if (!cogging_valid_)
    {
      return 0.0f;
    }
    return math::InterpolateCogging(cogging_, cogging_scale_A_,
                                    sample_.mechanical_rad);
  }

 private:
  float pole_pairs() const { return pll_.pole_pairs(); }

  void UpdatePllOutput()
  {
    sample_.mechanical_velocity_rad_s = pll_.velocity_mech();
    sample_.electrical_velocity_rad_s = pll_.omega_elec();
    sample_.electrical_rad = pll_.electrical_theta();
  }

  void UpdateSampleFromRaw()
  {
    if (sensor_ == nullptr)
    {
      sample_.valid = false;
      return;
    }
    sample_.raw = static_cast<uint16_t>(sensor_->raw());
    const uint32_t counts_per_turn = sensor_->counts_per_turn();
    sample_.counts_rad = counts_per_turn > 0u
        ? math::WrapZeroToTwoPi(
              static_cast<float>(sample_.raw) *
              (math::k2Pi / static_cast<float>(counts_per_turn)))
        : 0.0f;
    sample_.mechanical_rad = math::WrapZeroToTwoPi(
        calibration_.sign * sample_.counts_rad + calibration_.offset_rad);
    const float commutation = calibration_.commutation_valid
        ? math::InterpolateCommutationOffset(
              calibration_.commutation_offset_rad, sample_.counts_rad)
        : 0.0f;
    const float pp = pole_pairs();
    sample_.compensated_mechanical_rad = math::WrapZeroToTwoPi(
        sample_.mechanical_rad + EncoderCompensationRad() +
        (pp > 1.0e-6f ? commutation / pp : 0.0f));
    sample_.valid = sensor_->ok();
  }

  float EncoderCompensationRad() const
  {
    return compensation_valid_
        ? math::InterpolateEncoderComp(compensation_, compensation_scale_rad_,
                                       sample_.counts_rad)
        : 0.0f;
  }

  ports::IAngleSensor* sensor_ = nullptr;
  math::servo_mode::EncoderPll pll_;
  Calibration calibration_{};
  Calibration calibration_backup_{};
  bool calibration_backup_valid_ = false;
  Sample sample_{};
  math::EncoderCompTable compensation_{};
  float compensation_scale_rad_ = 0.0f;
  bool compensation_valid_ = false;
  uint8_t compensation_chunk_mask_ = 0;
  math::CoggingTable cogging_{};
  float cogging_scale_A_ = 0.0f;
  bool cogging_valid_ = false;
  bool spi_pending_ = false;
};

}  // namespace encoder
}  // namespace middleware
