// Shared bootloader memory map / protocol constants.
#pragma once

#include <cstdint>

inline constexpr uint32_t kAppStart = 0x08010000;
inline constexpr uint32_t kBootloaderStart = 0x0800C000;
inline constexpr uint32_t kBootloaderEnd = 0x08010000;
inline constexpr uint32_t kFlashEnd = 0x08080000;

inline constexpr uint32_t kBootMagicAddr = 0x20000000;
inline constexpr uint32_t kBootMagicValue = 0xB00710AD;

inline constexpr uint8_t kDefaultCanId = 1;
inline constexpr uint8_t kTunnelChannel = 1;

inline constexpr uint32_t kSystemClockHz = 16000000;
