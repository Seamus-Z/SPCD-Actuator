// Cogging-torque compensation table (moteus motor.cogging_dq_comp subset).
// A per-rotor-position feed-forward q current that cancels cogging torque.
// Indexed by mechanical encoder position over one full revolution [0, 2π).
// Stored as int8 + a float scale exactly like moteus (table[i] * scale = A).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "math/constants.h"

namespace math
{

// moteus uses 1024; keep the same resolution and wire/NVS layout.
inline constexpr size_t kCoggingTableSize = 1024;
using CoggingTable = std::array<int8_t, kCoggingTableSize>;

// position_rad: mechanical rotor angle. Wrapped to [0, 2π) internally.
// Returns the feed-forward q current [A] = lerp(table) * scale, matching
// moteus's ISR sample() in bldc_servo_control.h.
inline float InterpolateCogging(const CoggingTable& table, float scale,
                                float position_rad)
{
  const float wrapped = WrapZeroToTwoPi(position_rad);
  const float fraction = wrapped * (1.0f / k2Pi);  // [0, 1)
  const float scaled =
      fraction * static_cast<float>(kCoggingTableSize);
  const auto index = static_cast<uint32_t>(scaled);
  const size_t i0 = static_cast<size_t>(index) % kCoggingTableSize;
  const size_t i1 = (i0 + 1u) % kCoggingTableSize;
  const float frac = scaled - static_cast<float>(index);
  const float a = static_cast<float>(table[i0]) * scale;
  const float b = static_cast<float>(table[i1]) * scale;
  return a + frac * (b - a);
}

}  // namespace math
