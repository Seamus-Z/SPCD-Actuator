// MagAlpha MA600 magnetic encoder over SPI (moteus-x1 AUX2 defaults).
#pragma once

#include <cstdint>

#include "HAL/millisecond_timer.h"
#include "HAL/stm32_spi.h"
#include "math/constants.h"
#include "math/commutation.h"
#include "PinNames.h"
#include "stm32g4xx_hal.h"

namespace device
{

class Ma600
{
 public:
  static constexpr float kCpr = 65536.0f;

  struct Options
  {
    PinName mosi = PA_11;  // AUX2.C
    PinName miso = PA_10;  // AUX2.B
    PinName sck = PF_1;    // AUX2.A
    PinName cs = PB_7;     // AUX2.D
    int frequency_hz = 6000000;
    uint16_t filter_us = 1024;
    uint8_t bct = 0;
    uint8_t enable_trim = 0;
    // Mechanical angle = sign * raw/CPR * 2π + offset.
    float sign = 1.0f;
    float offset_rad = 0.0f;
    math::CommutationTable commutation_offset_rad{};
    bool commutation_valid = false;
  };

  Ma600(hal::MillisecondTimer* timer, const Options& options)
      : timer_(timer),
        options_(options),
        spi_([&]() {
          hal::Stm32Spi::Options spi_opt;
          spi_opt.mosi = options.mosi;
          spi_opt.miso = options.miso;
          spi_opt.sck = options.sck;
          spi_opt.cs = options.cs;
          spi_opt.frequency = options.frequency_hz;
          spi_opt.width = 16;
          spi_opt.mode = 0;
          return spi_opt;
        }())
  {
  }

  // Configure filter/BCT registers. Returns true on success.
  // Note: do NOT reject angle==0 / 0xFFFF — a resting shaft can legitimately
  // sit there; gating on that previously disabled all sampling/telemetry.
  bool Init()
  {
    // Ensure AUX2 GPIO banks are clocked (PF1 SCK needs GPIOF).
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    error_ = WriteConfig();
    ok_ = !error_;
    if (ok_)
    {
      (void)Sample();
    }
    return ok_;
  }

  uint16_t Sample()
  {
    raw_ = spi_.write(0x0000);
    return raw_;
  }

  void StartSample() { spi_.start_write(0x0000); }

  uint16_t FinishSample()
  {
    raw_ = spi_.finish_write();
    return raw_;
  }

  uint16_t raw() const { return raw_; }
  bool ok() const { return ok_ && !error_; }
  bool error() const { return error_; }

  // Counts angle only (no sign/offset) — used by phase calibration.
  float angle_counts_rad() const
  {
    return math::WrapZeroToTwoPi(
        (static_cast<float>(raw_) / kCpr) * math::k2Pi);
  }

  // Mechanical shaft angle [0, 2π).
  float angle_mech_rad() const
  {
    const float turns = options_.sign * (static_cast<float>(raw_) / kCpr);
    return math::WrapZeroToTwoPi(turns * math::k2Pi + options_.offset_rad);
  }

  // Electrical angle for FOC: mech * pole_pairs.
  float angle_elec_rad(float pole_pairs) const
  {
    return math::WrapZeroToTwoPi(angle_mech_rad() * pole_pairs);
  }

  float commutation_offset_rad() const
  {
    return options_.commutation_valid
               ? math::InterpolateCommutationOffset(
                     options_.commutation_offset_rad, angle_counts_rad())
               : 0.0f;
  }

  void SetOffsetRad(float offset_rad) { options_.offset_rad = offset_rad; }
  void SetSign(float sign) { options_.sign = (sign >= 0.0f) ? 1.0f : -1.0f; }
  void SetCommutationOffsets(const math::CommutationTable& offsets, bool valid)
  {
    options_.commutation_offset_rad = offsets;
    options_.commutation_valid = valid;
  }

  const Options& options() const { return options_; }

 private:
  // Returns true if configuration failed (matches moteus MA732::SetConfig).
  bool WriteConfig()
  {
    if (timer_ == nullptr)
    {
      return true;
    }

    const uint8_t desired_filter = [&]() -> uint8_t {
      const auto filter_us = options_.filter_us;
      if (filter_us == 0)
      {
        return 0;
      }
      if (filter_us <= 40)
      {
        return 5;
      }
      if (filter_us <= 80)
      {
        return 6;
      }
      if (filter_us <= 160)
      {
        return 7;
      }
      if (filter_us <= 320)
      {
        return 8;
      }
      if (filter_us <= 640)
      {
        return 9;
      }
      if (filter_us <= 1280)
      {
        return 10;
      }
      if (filter_us <= 2560)
      {
        return 11;
      }
      if (filter_us <= 5120)
      {
        return 12;
      }
      return 12;
    }();

    // FW = 0x0d[3:0]
    if (SetRegister(0x0d, desired_filter))
    {
      return true;
    }
    if (SetRegister(0x02, options_.bct))
    {
      return true;
    }
    if (SetRegister(0x03, static_cast<uint8_t>(options_.enable_trim & 0x03)))
    {
      return true;
    }
    return false;
  }

  // MA600 write path (no read-back verify). Returns true on hard failure.
  bool SetRegister(uint8_t reg, uint8_t desired)
  {
    spi_.write(0xea54);
    timer_->wait_us(2);
    const uint16_t write_reg_cmd =
        static_cast<uint16_t>((static_cast<uint16_t>(reg) << 8) | desired);
    spi_.write(write_reg_cmd);
    timer_->wait_us(2);
    spi_.write(0x0000);
    timer_->wait_us(2);
    return false;
  }

  hal::MillisecondTimer* timer_ = nullptr;
  Options options_{};
  hal::Stm32Spi spi_;
  uint16_t raw_ = 0;
  bool ok_ = false;
  bool error_ = false;
};

}  // namespace device
