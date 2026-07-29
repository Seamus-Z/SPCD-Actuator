// Park / Clark transforms for open-loop and closed-loop FOC.
#pragma once

// Avoid <cmath>/<math.h>: GCC 15 freestanding C++ pulls hosted TR1 specials.
extern "C" {
float sinf(float);
float cosf(float);
}

#include "math/constants.h"

namespace math
{

struct SinCos
{
  float s = 0.0f;
  float c = 1.0f;
};

// Soft-float path for bring-up; can later swap to STM32G4 CORDIC.
inline SinCos SinCosFromRadians(float theta_rad)
{
  SinCos out;
  out.s = sinf(theta_rad);
  out.c = cosf(theta_rad);
  return out;
}

// abc → dq (power-invariant 2/3 form, same as moteus).
struct DqTransform
{
  float d = 0.0f;
  float q = 0.0f;

  DqTransform(const SinCos& sc, float a, float b, float c)
      : d((2.0f / 3.0f) *
          (a * sc.c +
           (kSqrt3_4 * sc.s - 0.5f * sc.c) * b +
           (-kSqrt3_4 * sc.s - 0.5f * sc.c) * c)),
        q((2.0f / 3.0f) *
          (-sc.s * a -
           (-kSqrt3_4 * sc.c - 0.5f * sc.s) * b -
           (kSqrt3_4 * sc.c - 0.5f * sc.s) * c))
  {
  }
};

// dq → abc (inverse Park + inverse Clark combined).
struct InverseDqTransform
{
  float a = 0.0f;
  float b = 0.0f;
  float c = 0.0f;

  InverseDqTransform(const SinCos& sc, float d, float q)
      : a(sc.c * d - sc.s * q),
        b((kSqrt3_4 * sc.s - 0.5f * sc.c) * d -
          (-kSqrt3_4 * sc.c - 0.5f * sc.s) * q),
        c((-kSqrt3_4 * sc.s - 0.5f * sc.c) * d -
          (kSqrt3_4 * sc.c - 0.5f * sc.s) * q)
  {
  }
};

struct ClarkTransform
{
  float x = 0.0f;
  float y = 0.0f;

  ClarkTransform(float a, float b, float c)
      : x((2.0f * a - b - c) * (1.0f / 3.0f)),
        y((b - c) * (1.0f / kSqrt3))
  {
  }
};

struct InverseClarkTransform
{
  float a = 0.0f;
  float b = 0.0f;
  float c = 0.0f;

  InverseClarkTransform(float x, float y)
      : a(x),
        b((-x + kSqrt3 * y) * 0.5f),
        c((-x - kSqrt3 * y) * 0.5f)
  {
  }
};

// Phase voltages [V] → PWM duties in [0,1] with min/max centering (SVPWM-ish).
// |bus_V| must be > 0. Returns false if bus is invalid.
struct BalancedPwm
{
  float duty_a = 0.5f;
  float duty_b = 0.5f;
  float duty_c = 0.5f;

  static bool FromPhaseVolts(float va, float vb, float vc, float bus_V,
                             float min_duty, float max_duty, BalancedPwm* out)
  {
    if (out == nullptr || bus_V <= 1.0e-3f)
    {
      return false;
    }
    const float inv_bus = 1.0f / bus_V;
    float pa = va * inv_bus;
    float pb = vb * inv_bus;
    float pc = vc * inv_bus;

    const float pmin = (pa < pb) ? ((pa < pc) ? pa : pc) : ((pb < pc) ? pb : pc);
    const float pmax = (pa > pb) ? ((pa > pc) ? pa : pc) : ((pb > pc) ? pb : pc);
    const float offset = 0.5f * (pmin + pmax) - 0.5f;
    pa -= offset;
    pb -= offset;
    pc -= offset;

    auto clamp = [min_duty, max_duty](float x) {
      if (x < min_duty)
      {
        return min_duty;
      }
      if (x > max_duty)
      {
        return max_duty;
      }
      return x;
    };
    out->duty_a = clamp(pa);
    out->duty_b = clamp(pb);
    out->duty_c = clamp(pc);
    return true;
  }
};

}  // namespace math
