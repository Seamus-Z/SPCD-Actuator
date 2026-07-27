#include "BL_FlashWriter.h"

#include "stm32g4xx.h"

void BL_FlashWriter::Unlock()
{
  for (auto& v : sectors_erased_)
  {
    v = false;
  }
  shadow_start_ = 0;
  shadow_ = ~0ull;
  shadow_bits_ = 0;

  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
  __HAL_FLASH_DATA_CACHE_DISABLE();

  if (!locked_)
  {
    return;
  }

  FLASH->SR |= (FLASH_SR_OPTVERR | FLASH_SR_RDERR | FLASH_SR_FASTERR |
                FLASH_SR_MISERR | FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_PROGERR |
                FLASH_SR_OPERR | FLASH_SR_EOP);
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
  locked_ = false;
}

uint32_t BL_FlashWriter::Lock()
{
  if (locked_)
  {
    return 0;
  }
  if (shadow_bits_)
  {
    const auto err = FlushWord();
    if (err)
    {
      return err;
    }
  }
  FLASH->CR |= FLASH_CR_LOCK;
  locked_ = true;
  return 0;
}

uint32_t BL_FlashWriter::ProgramByte(uint32_t intaddr, uint8_t value)
{
  const uint32_t this_shadow = intaddr & ~(0x7u);
  const uint32_t offset = intaddr & 0x7u;

  if (this_shadow != shadow_start_ && shadow_start_ != 0)
  {
    const auto err = FlushWord();
    if (err)
    {
      return err;
    }
  }

  shadow_start_ = this_shadow;
  const uint64_t mask = (0xffull << (offset * 8));
  shadow_ = (shadow_ & ~mask) | (static_cast<uint64_t>(value) << (offset * 8));
  shadow_bits_ |= mask;

  if (shadow_bits_ == 0xffffffffffffffffull)
  {
    return FlushWord();
  }
  return 0;
}

uint32_t BL_FlashWriter::FlushWord()
{
  const auto err = MaybeEraseSector(shadow_start_);
  if (err)
  {
    return err;
  }

  if (*reinterpret_cast<uint32_t*>(shadow_start_) !=
          static_cast<uint32_t>(shadow_ & 0xffffffffu) ||
      *reinterpret_cast<uint32_t*>(shadow_start_ + 4u) !=
          static_cast<uint32_t>(shadow_ >> 32u))
  {
    FLASH->CR |= FLASH_CR_PG;
    *reinterpret_cast<uint32_t*>(shadow_start_) =
        static_cast<uint32_t>(shadow_ & 0xffffffffu);
    __ISB();
    *reinterpret_cast<uint32_t*>(shadow_start_ + 4u) =
        static_cast<uint32_t>(shadow_ >> 32u);
    const auto result = Wait();
    FLASH->CR &= ~FLASH_CR_PG;
    if (result)
    {
      return result;
    }
  }

  shadow_start_ = 0;
  shadow_ = ~0ull;
  shadow_bits_ = 0;
  return 0;
}

uint32_t BL_FlashWriter::MaybeEraseSector(uint32_t address)
{
  const int bank = (address < 0x08040000) ? 1 : 2;
  const uint32_t bank_start = (bank == 1) ? 0x08000000 : 0x08040000;
  const int sector = static_cast<int>((address - bank_start) / 2048);
  const int sector_index = (bank - 1) * 128 + sector;

  if (!sectors_erased_[sector_index])
  {
    const auto err = EraseSector(bank, sector);
    if (err)
    {
      return err;
    }
    sectors_erased_[sector_index] = true;
  }
  return 0;
}

uint32_t BL_FlashWriter::EraseSector(int bank, int sector)
{
  if (bank == 1)
  {
    FLASH->CR &= ~FLASH_CR_BKER;
  } else {
    FLASH->CR |= FLASH_CR_BKER;
  }
  FLASH->CR = (FLASH->CR & ~FLASH_CR_PNB_Msk) |
              (static_cast<uint32_t>(sector) << FLASH_CR_PNB_Pos);
  FLASH->CR |= FLASH_CR_PER;
  FLASH->CR |= FLASH_CR_STRT;
  const auto result = Wait();
  FLASH->CR &= ~FLASH_CR_PER;
  return result;
}

uint32_t BL_FlashWriter::Wait()
{
  uint32_t timeout = 8000000;
  while (FLASH->SR & FLASH_FLAG_BSY)
  {
    if (--timeout == 0)
    {
      return 0x80000001u;
    }
  }
  const uint32_t error = (FLASH->SR & FLASH_FLAG_SR_ERRORS);
  if (error != 0u)
  {
    FLASH->SR |= error;
    return error;
  }
  if (FLASH->SR & FLASH_FLAG_EOP)
  {
    FLASH->SR |= FLASH_FLAG_EOP;
  }
  return 0;
}
