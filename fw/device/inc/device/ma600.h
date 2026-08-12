// MagAlpha MA600 magnetic encoder.
// Platform SPI and delay implementations are injected by the board composition.
#pragma once

#include <cstdint>

#include "ports/angle_sensor.h"
#include "ports/platform_ports.h"

namespace device
{

class Ma600 final : public ports::IAngleSensor
{
 public:
  static constexpr uint32_t kCountsPerTurn = 65536u;

  struct Config
  {
    uint16_t filter_us = 1024;
    uint8_t bct = 0;
    uint8_t enable_trim = 0;
  };

  Ma600(ports::ISpiBus* spi, ports::IDelay* delay, const Config& config)
      : spi_(spi), delay_(delay), config_(config)
  {
  }

  bool Init() override
  {
    error_ = WriteConfig();
    ok_ = !error_;
    if (ok_)
    {
      (void)Sample();
    }
    return ok_;
  }

  uint32_t Sample() override
  {
    if (spi_ == nullptr)
    {
      error_ = true;
      return raw_;
    }
    raw_ = spi_->Transfer16(0x0000u);
    return raw_;
  }

  void StartSample() override
  {
    if (spi_ == nullptr)
    {
      error_ = true;
      return;
    }
    spi_->BeginTransfer16(0x0000u);
  }

  uint32_t FinishSample() override
  {
    if (spi_ == nullptr)
    {
      error_ = true;
      return raw_;
    }
    raw_ = spi_->FinishTransfer16();
    return raw_;
  }

  uint32_t raw() const override { return raw_; }
  uint32_t counts_per_turn() const override { return kCountsPerTurn; }
  bool ok() const override { return ok_ && !error_; }
  bool error() const { return error_; }
  uint16_t filter_us() const { return config_.filter_us; }

  bool SetFilterUs(uint16_t filter_us) override
  {
    config_.filter_us = filter_us;
    error_ = WriteConfig();
    ok_ = !error_;
    return ok_;
  }

 private:
  bool WriteConfig()
  {
    if (spi_ == nullptr || delay_ == nullptr)
    {
      return true;
    }

    const uint8_t desired_filter = [&]() -> uint8_t {
      const auto filter_us = config_.filter_us;
      if (filter_us == 0) return 0;
      if (filter_us <= 40) return 5;
      if (filter_us <= 80) return 6;
      if (filter_us <= 160) return 7;
      if (filter_us <= 320) return 8;
      if (filter_us <= 640) return 9;
      if (filter_us <= 1280) return 10;
      if (filter_us <= 2560) return 11;
      return 12;
    }();

    if (SetRegister(0x0d, desired_filter)) return true;
    if (SetRegister(0x02, config_.bct)) return true;
    if (SetRegister(0x03, static_cast<uint8_t>(config_.enable_trim & 0x03)))
    {
      return true;
    }
    return false;
  }

  // Returns true if configuration failed (matches the existing driver contract).
  bool SetRegister(uint8_t reg, uint8_t desired)
  {
    spi_->Transfer16(0xea54u);
    delay_->WaitUs(2u);
    const uint16_t command = static_cast<uint16_t>(
        (static_cast<uint16_t>(reg) << 8) | desired);
    spi_->Transfer16(command);
    delay_->WaitUs(2u);
    spi_->Transfer16(0x0000u);
    delay_->WaitUs(2u);
    return false;
  }

  ports::ISpiBus* spi_ = nullptr;
  ports::IDelay* delay_ = nullptr;
  Config config_{};
  uint16_t raw_ = 0;
  bool ok_ = false;
  bool error_ = false;
};

}  // namespace device
