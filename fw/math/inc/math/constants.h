// Scalar math helpers used by FOC / control (moteus-compatible constants).
#pragma once

#include <cstdint>

namespace math
{

constexpr float kPi = 3.141592653589793f;
constexpr float k2Pi = 6.283185307179586f;
constexpr float kSqrt3 = 1.7320508075688772f;
constexpr float kSqrt3_4 = 0.86602540378f;  // √3 / 2
constexpr float kSvpwmRatio = 1.15470053838f;  // 2/√3

// Wrap to [0, 2π).
inline float WrapZeroToTwoPi(float x)
{
  const int32_t divisor = static_cast<int32_t>(x / k2Pi);
  const float mod = x - static_cast<float>(divisor) * k2Pi;
  return (mod >= 0.0f) ? mod : (mod + k2Pi);
}

// Wrap to (−π, π].
inline float WrapNegPiToPi(float x)
{
  if (x >= -kPi && x <= kPi)
  {
    return x;
  }
  const float wrapped = WrapZeroToTwoPi(x);
  return (wrapped > kPi) ? (wrapped - k2Pi) : wrapped;
}

// Radians → Q31 angle (for future CORDIC); currently unused by soft sin/cos.
inline int32_t RadiansToQ31(float x)
{
  const float scaled = x / k2Pi;
  const int32_t i = static_cast<int32_t>(scaled);
  float mod = scaled - static_cast<float>(i);
  if (mod < 0.0f)
  {
    mod += 1.0f;
  }
  return static_cast<int32_t>(
      ((mod > 0.5f) ? (mod - 1.0f) : mod) * 4294967296.0f);
}

}  // namespace math
