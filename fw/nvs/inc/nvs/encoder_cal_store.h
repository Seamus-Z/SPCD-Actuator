// Persistent encoder and motor calibration.
// Lives in the final 4 KiB of STM32G474 flash (Bank2 pages 126-127).
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "math/commutation.h"
#include "math/cogging.h"
#include "math/encoder_comp.h"
#include "stm32g4xx_hal.h"

namespace nvs
{

struct MotorConfig
{
  float offset_rad = 0.0f;
  float sign = 1.0f;
  math::CommutationTable commutation_offset_rad{};
  bool commutation_valid = false;
  float commutation_residual_rad = 0.0f;
  float bemf_v_per_hz = 0.0f;  // dq Ke [V*s/rad mech]
  bool bemf_valid = false;
  float resistance_ohm = 0.0f;
  bool resistance_valid = false;
  float inductance_d_H = 0.0f;
  float inductance_q_H = 0.0f;
  bool inductance_valid = false;
  math::CoggingTable cogging_table{};
  float cogging_scale = 0.0f;
  bool cogging_valid = false;
  math::EncoderCompTable encoder_comp_table{};
  float encoder_comp_scale = 0.0f;  // rad per int8 LSB
  bool encoder_comp_valid = false;
};

class MotorConfigStore
{
 public:
  static constexpr uint32_t kMagic = 0x58454E43u;  // 'XENC'
  static constexpr uint32_t kVersion = 7u;
  static constexpr uint32_t kBaseAddr = 0x0807F000u;
  static constexpr uint32_t kSize = 0x1000u;
  static constexpr uint32_t kBank = FLASH_BANK_2;
  static constexpr uint32_t kPage = 126u;
  static constexpr uint32_t kNbPages = 2u;

  // Kept byte-for-byte so firmware can migrate older calibration records.
  struct RecordV3
  {
    uint32_t magic = 0;
    uint32_t version = 0;
    float offset_rad = 0.0f;
    float sign = 1.0f;
    float bemf_v_per_hz = 0.0f;
    uint32_t bemf_valid = 0;
    float resistance_ohm = 0.0f;
    uint32_t resistance_valid = 0;
    float inductance_H = 0.0f;
    uint32_t inductance_valid = 0;
    uint32_t crc = 0;
  } __attribute__((packed));

  struct RecordV4
  {
    uint32_t magic = 0;
    uint32_t version = 0;
    float offset_rad = 0.0f;
    float sign = 1.0f;
    float commutation_offset_rad[math::kCommutationTableSize]{};
    uint32_t commutation_valid = 0;
    float bemf_v_per_hz = 0.0f;
    uint32_t bemf_valid = 0;
    float resistance_ohm = 0.0f;
    uint32_t resistance_valid = 0;
    float inductance_d_H = 0.0f;
    float inductance_q_H = 0.0f;
    uint32_t inductance_valid = 0;
    uint32_t crc = 0;
  } __attribute__((packed));

  struct RecordV5
  {
    uint32_t magic = 0;
    uint32_t version = 0;
    float offset_rad = 0.0f;
    float sign = 1.0f;
    float commutation_offset_rad[math::kCommutationTableSize]{};
    uint32_t commutation_valid = 0;
    float commutation_residual_rad = 0.0f;
    float bemf_v_per_hz = 0.0f;
    uint32_t bemf_valid = 0;
    float resistance_ohm = 0.0f;
    uint32_t resistance_valid = 0;
    float inductance_d_H = 0.0f;
    float inductance_q_H = 0.0f;
    uint32_t inductance_valid = 0;
    uint32_t crc = 0;
  } __attribute__((packed));

  struct RecordV6
  {
    uint32_t magic = 0;
    uint32_t version = 0;
    float offset_rad = 0.0f;
    float sign = 1.0f;
    float commutation_offset_rad[math::kCommutationTableSize]{};
    uint32_t commutation_valid = 0;
    float commutation_residual_rad = 0.0f;
    float bemf_v_per_hz = 0.0f;
    uint32_t bemf_valid = 0;
    float resistance_ohm = 0.0f;
    uint32_t resistance_valid = 0;
    float inductance_d_H = 0.0f;
    float inductance_q_H = 0.0f;
    uint32_t inductance_valid = 0;
    int8_t cogging_table[math::kCoggingTableSize]{};
    float cogging_scale = 0.0f;
    uint32_t cogging_valid = 0;
    uint32_t crc = 0;
  } __attribute__((packed));

  struct Record
  {
    uint32_t magic = 0;
    uint32_t version = 0;
    float offset_rad = 0.0f;
    float sign = 1.0f;
    float commutation_offset_rad[math::kCommutationTableSize]{};
    uint32_t commutation_valid = 0;
    float commutation_residual_rad = 0.0f;
    float bemf_v_per_hz = 0.0f;
    uint32_t bemf_valid = 0;
    float resistance_ohm = 0.0f;
    uint32_t resistance_valid = 0;
    float inductance_d_H = 0.0f;
    float inductance_q_H = 0.0f;
    uint32_t inductance_valid = 0;
    int8_t cogging_table[math::kCoggingTableSize]{};
    float cogging_scale = 0.0f;
    uint32_t cogging_valid = 0;
    int8_t encoder_comp_table[math::kEncoderCompTableSize]{};
    float encoder_comp_scale = 0.0f;
    uint32_t encoder_comp_valid = 0;
    uint32_t crc = 0;
  } __attribute__((packed));

  static bool Load(MotorConfig* out)
  {
    if (out == nullptr)
    {
      return false;
    }

    uint32_t header[2]{};
    std::memcpy(header, reinterpret_cast<const void*>(kBaseAddr),
                sizeof(header));
    if (header[0] != kMagic)
    {
      return false;
    }

    if (header[1] == kVersion)
    {
      Record rec{};
      std::memcpy(&rec, reinterpret_cast<const void*>(kBaseAddr), sizeof(rec));
      if (rec.crc != CrcBytes(&rec, offsetof(Record, crc)) ||
          !(rec.sign == 1.0f || rec.sign == -1.0f))
      {
        return false;
      }
      *out = MotorConfig{};
      out->offset_rad = rec.offset_rad;
      out->sign = rec.sign;
      std::memcpy(out->commutation_offset_rad.data(),
                  rec.commutation_offset_rad,
                  sizeof(rec.commutation_offset_rad));
      out->commutation_residual_rad = rec.commutation_residual_rad;
      out->commutation_valid =
          rec.commutation_valid != 0 &&
          rec.commutation_residual_rad >= 0.0f &&
          rec.commutation_residual_rad <= math::kMaxCommutationResidualRad;
      out->bemf_v_per_hz = rec.bemf_v_per_hz;
      out->bemf_valid = rec.bemf_valid != 0;
      out->resistance_ohm = rec.resistance_ohm;
      out->resistance_valid = rec.resistance_valid != 0;
      out->inductance_d_H = rec.inductance_d_H;
      out->inductance_q_H = rec.inductance_q_H;
      out->inductance_valid = rec.inductance_valid != 0;
      std::memcpy(out->cogging_table.data(), rec.cogging_table,
                  sizeof(rec.cogging_table));
      out->cogging_scale = rec.cogging_scale;
      out->cogging_valid = rec.cogging_valid != 0;
      std::memcpy(out->encoder_comp_table.data(), rec.encoder_comp_table,
                  sizeof(rec.encoder_comp_table));
      out->encoder_comp_scale = rec.encoder_comp_scale;
      out->encoder_comp_valid =
          rec.encoder_comp_valid != 0 && rec.encoder_comp_scale > 0.0f;
      return true;
    }

    if (header[1] == 6u)
    {
      RecordV6 rec{};
      std::memcpy(&rec, reinterpret_cast<const void*>(kBaseAddr), sizeof(rec));
      if (rec.crc != CrcBytes(&rec, offsetof(RecordV6, crc)) ||
          !(rec.sign == 1.0f || rec.sign == -1.0f))
      {
        return false;
      }
      *out = MotorConfig{};
      out->offset_rad = rec.offset_rad;
      out->sign = rec.sign;
      std::memcpy(out->commutation_offset_rad.data(),
                  rec.commutation_offset_rad,
                  sizeof(rec.commutation_offset_rad));
      out->commutation_residual_rad = rec.commutation_residual_rad;
      out->commutation_valid =
          rec.commutation_valid != 0 &&
          rec.commutation_residual_rad >= 0.0f &&
          rec.commutation_residual_rad <= math::kMaxCommutationResidualRad;
      out->bemf_v_per_hz = rec.bemf_v_per_hz;
      out->bemf_valid = rec.bemf_valid != 0;
      out->resistance_ohm = rec.resistance_ohm;
      out->resistance_valid = rec.resistance_valid != 0;
      out->inductance_d_H = rec.inductance_d_H;
      out->inductance_q_H = rec.inductance_q_H;
      out->inductance_valid = rec.inductance_valid != 0;
      std::memcpy(out->cogging_table.data(), rec.cogging_table,
                  sizeof(rec.cogging_table));
      out->cogging_scale = rec.cogging_scale;
      out->cogging_valid = rec.cogging_valid != 0;
      // V6 had no encoder geometric compensation table.
      return true;
    }

    if (header[1] == 5u)
    {
      RecordV5 rec{};
      std::memcpy(&rec, reinterpret_cast<const void*>(kBaseAddr), sizeof(rec));
      if (rec.crc != CrcBytes(&rec, offsetof(RecordV5, crc)) ||
          !(rec.sign == 1.0f || rec.sign == -1.0f))
      {
        return false;
      }
      *out = MotorConfig{};
      out->offset_rad = rec.offset_rad;
      out->sign = rec.sign;
      std::memcpy(out->commutation_offset_rad.data(),
                  rec.commutation_offset_rad,
                  sizeof(rec.commutation_offset_rad));
      out->commutation_residual_rad = rec.commutation_residual_rad;
      out->commutation_valid =
          rec.commutation_valid != 0 &&
          rec.commutation_residual_rad >= 0.0f &&
          rec.commutation_residual_rad <= math::kMaxCommutationResidualRad;
      out->bemf_v_per_hz = rec.bemf_v_per_hz;
      out->bemf_valid = rec.bemf_valid != 0;
      out->resistance_ohm = rec.resistance_ohm;
      out->resistance_valid = rec.resistance_valid != 0;
      out->inductance_d_H = rec.inductance_d_H;
      out->inductance_q_H = rec.inductance_q_H;
      out->inductance_valid = rec.inductance_valid != 0;
      // V5 had no cogging table; leave cogging_valid = false.
      return true;
    }

    if (header[1] == 4u)
    {
      RecordV4 rec{};
      std::memcpy(&rec, reinterpret_cast<const void*>(kBaseAddr), sizeof(rec));
      if (rec.crc != CrcBytes(&rec, offsetof(RecordV4, crc)) ||
          !(rec.sign == 1.0f || rec.sign == -1.0f))
      {
        return false;
      }
      *out = MotorConfig{};
      out->offset_rad = rec.offset_rad;
      out->sign = rec.sign;
      // V4 did not persist fit quality. Preserve motor parameters and global
      // alignment, but require a new qualified spin map before compensation.
      out->commutation_valid = false;
      out->bemf_v_per_hz = rec.bemf_v_per_hz;
      out->bemf_valid = rec.bemf_valid != 0;
      out->resistance_ohm = rec.resistance_ohm;
      out->resistance_valid = rec.resistance_valid != 0;
      out->inductance_d_H = rec.inductance_d_H;
      out->inductance_q_H = rec.inductance_q_H;
      out->inductance_valid = rec.inductance_valid != 0;
      return true;
    }

    if (header[1] == 3u)
    {
      RecordV3 rec{};
      std::memcpy(&rec, reinterpret_cast<const void*>(kBaseAddr), sizeof(rec));
      if (rec.crc != CrcBytes(&rec, offsetof(RecordV3, crc)) ||
          !(rec.sign == 1.0f || rec.sign == -1.0f))
      {
        return false;
      }
      *out = MotorConfig{};
      out->offset_rad = rec.offset_rad;
      out->sign = rec.sign;
      out->bemf_v_per_hz = rec.bemf_v_per_hz;
      out->bemf_valid = rec.bemf_valid != 0;
      out->resistance_ohm = rec.resistance_ohm;
      out->resistance_valid = rec.resistance_valid != 0;
      out->inductance_d_H = rec.inductance_H;
      out->inductance_q_H = rec.inductance_H;
      out->inductance_valid = rec.inductance_valid != 0;
      return true;
    }
    return false;
  }

  static bool Save(const MotorConfig& data)
  {
    Record rec{};
    rec.magic = kMagic;
    rec.version = kVersion;
    rec.offset_rad = data.offset_rad;
    rec.sign = (data.sign >= 0.0f) ? 1.0f : -1.0f;
    std::memcpy(rec.commutation_offset_rad,
                data.commutation_offset_rad.data(),
                sizeof(rec.commutation_offset_rad));
    rec.commutation_residual_rad = data.commutation_residual_rad;
    rec.commutation_valid =
        data.commutation_valid &&
        data.commutation_residual_rad >= 0.0f &&
        data.commutation_residual_rad <= math::kMaxCommutationResidualRad
            ? 1u
            : 0u;
    rec.bemf_v_per_hz = data.bemf_v_per_hz;
    rec.bemf_valid = data.bemf_valid ? 1u : 0u;
    rec.resistance_ohm = data.resistance_ohm;
    rec.resistance_valid = data.resistance_valid ? 1u : 0u;
    rec.inductance_d_H = data.inductance_d_H;
    rec.inductance_q_H = data.inductance_q_H;
    rec.inductance_valid = data.inductance_valid ? 1u : 0u;
    std::memcpy(rec.cogging_table, data.cogging_table.data(),
                sizeof(rec.cogging_table));
    rec.cogging_scale = data.cogging_scale;
    rec.cogging_valid = data.cogging_valid ? 1u : 0u;
    std::memcpy(rec.encoder_comp_table, data.encoder_comp_table.data(),
                sizeof(rec.encoder_comp_table));
    rec.encoder_comp_scale = data.encoder_comp_scale;
    rec.encoder_comp_valid =
        (data.encoder_comp_valid && data.encoder_comp_scale > 0.0f) ? 1u : 0u;
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
    static_assert(kProgramSize <= kSize, "calibration record exceeds flash area");
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

    MotorConfig verify{};
    return Load(&verify) && verify.offset_rad == data.offset_rad &&
           verify.sign == rec.sign &&
           verify.commutation_valid == (rec.commutation_valid != 0) &&
           (!verify.commutation_valid ||
            (verify.commutation_residual_rad ==
                 data.commutation_residual_rad &&
             std::memcmp(verify.commutation_offset_rad.data(),
                         data.commutation_offset_rad.data(),
                         sizeof(rec.commutation_offset_rad)) == 0)) &&
           verify.bemf_valid == data.bemf_valid &&
           (!verify.bemf_valid ||
            verify.bemf_v_per_hz == data.bemf_v_per_hz) &&
           verify.resistance_valid == data.resistance_valid &&
           (!verify.resistance_valid ||
            verify.resistance_ohm == data.resistance_ohm) &&
           verify.inductance_valid == data.inductance_valid &&
           (!verify.inductance_valid ||
            (verify.inductance_d_H == data.inductance_d_H &&
             verify.inductance_q_H == data.inductance_q_H)) &&
           verify.cogging_valid == (rec.cogging_valid != 0) &&
           (!verify.cogging_valid ||
            (verify.cogging_scale == data.cogging_scale &&
             std::memcmp(verify.cogging_table.data(),
                         data.cogging_table.data(),
                         sizeof(rec.cogging_table)) == 0)) &&
           verify.encoder_comp_valid == (rec.encoder_comp_valid != 0) &&
           (!verify.encoder_comp_valid ||
            (verify.encoder_comp_scale == data.encoder_comp_scale &&
             std::memcmp(verify.encoder_comp_table.data(),
                         data.encoder_comp_table.data(),
                         sizeof(rec.encoder_comp_table)) == 0));
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
