// Persistent runtime tunables (motor / foc / servo / encoder).
// Separate flash page from calibration tables so saving PID does not rewrite
// cogging/encoder-comp blobs.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "middleware/config/runtime_config.h"
#include "stm32g4xx_hal.h"

namespace middleware::persistence
{

using config::EncoderConf;
using config::FocConf;
using config::MotorConf;
using config::RuntimeConfig;
using config::ServoConf;

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

}  // namespace middleware::persistence
