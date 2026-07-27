// Bootloader flash programming helper (unlock / erase / program / lock).
#pragma once

#include <cstdint>

class BL_FlashWriter
{
 public:
  bool locked() const { return locked_; }

  void Unlock();
  uint32_t Lock();
  uint32_t ProgramByte(uint32_t address, uint8_t value);

 private:
  uint32_t FlushWord();
  uint32_t MaybeEraseSector(uint32_t address);
  uint32_t EraseSector(int bank, int sector);
  uint32_t Wait();

  bool locked_ = true;
  uint32_t shadow_start_ = 0;
  uint64_t shadow_ = ~0ull;
  uint64_t shadow_bits_ = 0;
  bool sectors_erased_[256] = {};
};
