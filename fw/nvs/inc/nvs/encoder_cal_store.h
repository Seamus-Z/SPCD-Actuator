// Persistent encoder phase calibration (moteus-like conf write).
// Lives in the final 4 KiB of STM32G474 flash (Bank2 pages 126-127).
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "stm32g4xx_hal.h"

namespace nvs
{

struct EncoderCalData
{
  float offset_rad = 0.0f;
  float sign = 1.0f;
};

class EncoderCalStore
{
 public:
  static constexpr uint32_t kMagic = 0x58454E43u;  // 'XENC'
  static constexpr uint32_t kVersion = 1u;
  // Match moteus: final 4 KiB of 512 KiB flash.
  static constexpr uint32_t kBaseAddr = 0x0807F000u;
  static constexpr uint32_t kSize = 0x1000u;
  static constexpr uint32_t kBank = FLASH_BANK_2;
  static constexpr uint32_t kPage = 126u;
  static constexpr uint32_t kNbPages = 2u;

  struct Record
  {
    uint32_t magic = 0;
    uint32_t version = 0;
    float offset_rad = 0.0f;
    float sign = 1.0f;
    uint32_t crc = 0;
  } __attribute__((packed));

  static bool Load(EncoderCalData* out)
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
    if (rec.crc != Crc(rec))
    {
      return false;
    }
    if (!(rec.sign == 1.0f || rec.sign == -1.0f))
    {
      return false;
    }
    out->offset_rad = rec.offset_rad;
    out->sign = rec.sign;
    return true;
  }

  // Erase + program. Safe to call from main loop (not ISR).
  static bool Save(const EncoderCalData& data)
  {
    Record rec{};
    rec.magic = kMagic;
    rec.version = kVersion;
    rec.offset_rad = data.offset_rad;
    rec.sign = (data.sign >= 0.0f) ? 1.0f : -1.0f;
    rec.crc = Crc(rec);

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

    // Program as double-words (8 bytes). Record is 20 bytes -> pad to 24.
    alignas(8) uint8_t buf[24]{};
    std::memcpy(buf, &rec, sizeof(rec));
    for (uint32_t off = 0; off < sizeof(buf); off += 8)
    {
      uint64_t word = 0;
      std::memcpy(&word, buf + off, 8);
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, kBaseAddr + off,
                            word) != HAL_OK)
      {
        HAL_FLASH_Lock();
        return false;
      }
    }

    HAL_FLASH_Lock();

    EncoderCalData verify{};
    return Load(&verify) && verify.offset_rad == data.offset_rad &&
           verify.sign == rec.sign;
  }

 private:
  static uint32_t Crc(const Record& rec)
  {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&rec);
    const size_t n = offsetof(Record, crc);
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i)
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
