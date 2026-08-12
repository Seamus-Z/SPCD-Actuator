// Persistent runtime tunables (motor / foc / servo / encoder).
// Separate flash page from calibration tables so saving PID does not rewrite
// cogging/encoder-comp blobs.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "stm32g4xx_hal.h"

namespace nvs
{

// Keep layouts identical to telemetry::xt_can::*Conf wire structs.
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
  float default_accel_limit_rad_s2 = 0.0f;  // host NaN => stored as quiet NaN
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

class RuntimeConfigStore
{
 public:
  static constexpr uint32_t kMagic = 0x58434647u;  // 'XCFG'
  static constexpr uint32_t kVersion = 1u;
  // Bank2 page 124: 0x0807E000 (2 KiB). Outside APP_FLASH (linker 440K).
  // Cal store owns pages 126-127 at 0x0807F000.
  static constexpr uint32_t kBaseAddr = 0x0807E000u;
  static constexpr uint32_t kSize = 0x800u;
  static constexpr uint32_t kBank = FLASH_BANK_2;
  static constexpr uint32_t kPage = 124u;
  static constexpr uint32_t kNbPages = 1u;

  struct Record
  {
    uint32_t magic = 0;
    uint32_t version = 0;
    MotorConf motor{};
    FocConf foc{};
    ServoConf servo{};
    EncoderConf encoder{};
    uint32_t crc = 0;
  } __attribute__((packed));

  static_assert(sizeof(MotorConf) == 32, "MotorConf size");
  static_assert(sizeof(FocConf) == 24, "FocConf size");
  static_assert(sizeof(ServoConf) == 48, "ServoConf size");
  static_assert(sizeof(EncoderConf) == 16, "EncoderConf size");
  static_assert(sizeof(Record) <= kSize, "runtime config record too large");

  static bool Load(RuntimeConfig* out)
  {
    if (out == nullptr)
    {
      return false;
    }
    Record rec{};
    std::memcpy(&rec, reinterpret_cast<const void*>(kBaseAddr), sizeof(rec));
    if (rec.magic != kMagic || rec.version != kVersion)
    {
      return false;
    }
    if (rec.crc != CrcBytes(&rec, offsetof(Record, crc)))
    {
      return false;
    }
    out->motor = rec.motor;
    out->foc = rec.foc;
    out->servo = rec.servo;
    out->encoder = rec.encoder;
    return true;
  }

  static bool Save(const RuntimeConfig& data)
  {
    Record rec{};
    rec.magic = kMagic;
    rec.version = kVersion;
    rec.motor = data.motor;
    rec.foc = data.foc;
    rec.servo = data.servo;
    rec.encoder = data.encoder;
    rec.crc = CrcBytes(&rec, offsetof(Record, crc));

    HAL_FLASH_Unlock();
    uint32_t page_err = 0xffffffffu;
    FLASH_EraseInitTypeDef erase{};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = kBank;
    erase.Page = kPage;
    erase.NbPages = kNbPages;
    if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK ||
        page_err != 0xffffffffu)
    {
      HAL_FLASH_Lock();
      return false;
    }

    static constexpr size_t kProgramSize = (sizeof(Record) + 7u) & ~size_t{7u};
    alignas(8) uint8_t buf[kProgramSize]{};
    std::memcpy(buf, &rec, sizeof(rec));
    for (uint32_t off = 0; off < kProgramSize; off += 8u)
    {
      uint64_t word = 0;
      std::memcpy(&word, buf + off, sizeof(word));
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, kBaseAddr + off,
                            word) != HAL_OK)
      {
        HAL_FLASH_Lock();
        return false;
      }
    }
    HAL_FLASH_Lock();

    RuntimeConfig verify{};
    return Load(&verify);
  }

 private:
  static uint32_t CrcBytes(const void* data, size_t size)
  {
    const auto* p = static_cast<const uint8_t*>(data);
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < size; ++i)
    {
      c ^= p[i];
      for (int b = 0; b < 8; ++b)
      {
        const uint32_t mask = -(c & 1u);
        c = (c >> 1) ^ (0xEDB88320u & mask);
      }
    }
    return ~c;
  }
};

}  // namespace nvs
