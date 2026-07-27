#include <cstdint>

#include "stm32g4xx.h"

#include "bootloader.h"

namespace
{
constexpr uint32_t kBootMagicAddr = 0x20000000;
constexpr uint32_t kBootMagicValue = 0xB00710AD;
}

extern "C" __attribute__((noreturn)) void EnterBootloader()
{
    *reinterpret_cast<volatile uint32_t*>(kBootMagicAddr) = kBootMagicValue;
    __DSB();
    NVIC_SystemReset();
    while (true) {}
}
