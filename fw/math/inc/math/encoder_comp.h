// Encoder geometric compensation (moteus motor_position compensation_table).
// Corrects 1/rev and 2/rev magnet eccentricity/tilt before the PLL.
// Indexed by raw encoder angle over one revolution; stored as int8 + scale
// exactly like moteus (table[i] * scale = radians of mechanical correction).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "math/constants.h"

namespace math
{

// moteus SourceConfig::kCompensationSize = 256.
inline constexpr size_t kEncoderCompTableSize = 256;
using EncoderCompTable = std::array<int8_t, kEncoderCompTableSize>;

// position_rad: raw encoder angle (counts domain) wrapped to [0, 2π).
// Returns mechanical correction [rad] = lerp(table) * scale.
inline float InterpolateEncoderComp(const EncoderCompTable& table, float scale,
                                    float position_rad)
{
  const float wrapped = WrapZeroToTwoPi(position_rad);
  const float fraction = wrapped * (1.0f / k2Pi);
  const float scaled =
      fraction * static_cast<float>(kEncoderCompTableSize);
  const auto index = static_cast<uint32_t>(scaled);
  const size_t i0 = static_cast<size_t>(index) % kEncoderCompTableSize;
  const size_t i1 = (i0 + 1u) % kEncoderCompTableSize;
  const float frac = scaled - static_cast<float>(index);
  const float a = static_cast<float>(table[i0]) * scale;
  const float b = static_cast<float>(table[i1]) * scale;
  return a + frac * (b - a);
}

}  // namespace math
