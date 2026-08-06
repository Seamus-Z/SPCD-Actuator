// Periodic electrical-angle correction indexed by raw encoder angle.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "math/constants.h"

namespace math
{

inline constexpr size_t kCommutationTableSize = 64;
// A larger direction-normalized scatter means the correction is not
// repeatable and can rotate commanded q current into d current.
inline constexpr float kMaxCommutationResidualRad = 0.25f;
using CommutationTable = std::array<float, kCommutationTableSize>;

inline float InterpolateCommutationOffset(const CommutationTable& table,
                                          float raw_angle_rad)
{
  const float wrapped = WrapZeroToTwoPi(raw_angle_rad);
  const float scaled = wrapped *
                       (static_cast<float>(kCommutationTableSize) / k2Pi);
  const auto index = static_cast<uint32_t>(scaled);
  const size_t i0 = static_cast<size_t>(index) % kCommutationTableSize;
  const size_t i1 = (i0 + 1u) % kCommutationTableSize;
  const float fraction = scaled - static_cast<float>(index);
  return table[i0] + fraction * (table[i1] - table[i0]);
}

}  // namespace math
