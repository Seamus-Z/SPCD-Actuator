// Tiny integer/string formatter for telemetry (avoids newlib snprintf stack).
#pragma once

#include <cstddef>
#include <cstdint>

namespace core::text
{

inline size_t AppendChar(char* out, size_t cap, size_t pos, char c)
{
  if (pos + 1 < cap)
  {
    out[pos++] = c;
  }
  if (pos < cap)
  {
    out[pos] = '\0';
  }
  return pos;
}

inline size_t AppendStr(char* out, size_t cap, size_t pos, const char* s)
{
  if (s == nullptr)
  {
    return pos;
  }
  while (*s != '\0')
  {
    pos = AppendChar(out, cap, pos, *s++);
  }
  return pos;
}

inline size_t AppendUInt(char* out, size_t cap, size_t pos, unsigned value)
{
  char tmp[10];
  size_t n = 0;
  do
  {
    tmp[n++] = static_cast<char>('0' + (value % 10u));
    value /= 10u;
  } while (value != 0u && n < sizeof(tmp));

  while (n > 0)
  {
    pos = AppendChar(out, cap, pos, tmp[--n]);
  }
  return pos;
}

inline size_t AppendHex(char* out, size_t cap, size_t pos, unsigned value,
                        unsigned width)
{
  static constexpr char kDigits[] = "0123456789abcdef";
  for (unsigned i = width; i > 0; --i)
  {
    const unsigned shift = (i - 1u) * 4u;
    pos = AppendChar(out, cap, pos, kDigits[(value >> shift) & 0xfu]);
  }
  return pos;
}

inline size_t AppendKeyUInt(char* out, size_t cap, size_t pos,
                            const char* key, unsigned value, bool leading_space)
{
  if (leading_space)
  {
    pos = AppendChar(out, cap, pos, ' ');
  }
  pos = AppendStr(out, cap, pos, key);
  pos = AppendChar(out, cap, pos, '=');
  return AppendUInt(out, cap, pos, value);
}

inline size_t AppendKeyHex(char* out, size_t cap, size_t pos,
                           const char* key, unsigned value, unsigned width,
                           bool leading_space)
{
  if (leading_space)
  {
    pos = AppendChar(out, cap, pos, ' ');
  }
  pos = AppendStr(out, cap, pos, key);
  pos = AppendChar(out, cap, pos, '=');
  return AppendHex(out, cap, pos, value, width);
}

}  // namespace core::text
