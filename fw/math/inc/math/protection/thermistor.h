// NTC thermistor in a 3.3 V / 10 kΩ divider (moteus-x1 vt_sense).
#pragma once

#include <cstdint>

namespace math {
namespace protection {
namespace {

inline float NaturalLog(float x)
{
  if (!(x > 0.0f))
  {
    return -1.0e6f;
  }
  union
  {
    float f;
    uint32_t u;
  } value;
  value.f = x;
  const int exponent = static_cast<int>((value.u >> 23) & 0xffu) - 127;
  value.u = (value.u & 0x7fffffu) | (127u << 23);
  const float mantissa = value.f;
  const float y = (mantissa - 1.0f) / (mantissa + 1.0f);
  const float y2 = y * y;
  const float ln_m =
      2.0f * y *
      (1.0f + y2 * (1.0f / 3.0f + y2 * (1.0f / 5.0f + y2 * (1.0f / 7.0f))));
  return ln_m + static_cast<float>(exponent) * 0.69314718f;
}

}  // namespace

struct Thermistor
{
  float r25_ohm = 47000.0f;
  float beta = 4050.0f;
  float divider_ohm = 10000.0f;
  float vref_V = 3.3f;

  bool ToCelsius(uint16_t adc_raw, float* out_C) const
  {
    if (out_C == nullptr || adc_raw < 8u || adc_raw > 4088u)
    {
      return false;
    }
    const float v = vref_V * static_cast<float>(adc_raw) / 4096.0f;
    if (v <= 1.0e-3f)
    {
      return false;
    }
    const float r_t = vref_V * divider_ohm / v - divider_ohm;
    if (!(r_t > 1.0f) || r25_ohm <= 1.0f || beta <= 1.0f)
    {
      return false;
    }
    const float inv_t =
        (1.0f / 298.15f) + (1.0f / beta) * NaturalLog(r_t / r25_ohm);
    if (!(inv_t > 1.0e-4f))
    {
      return false;
    }
    *out_C = (1.0f / inv_t) - 273.15f;
    return *out_C > -40.0f && *out_C < 250.0f;
  }
};

}  // namespace protection
}  // namespace math
