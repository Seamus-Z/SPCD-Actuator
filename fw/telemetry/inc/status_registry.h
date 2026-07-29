// Named status channels for the app telemetry tree (text snapshot exporters).
#pragma once

#include <cstddef>
#include <cstdint>

namespace telemetry
{

// Writes a single-line (or multi-field) snapshot into |out|, NUL-terminated.
// Returns bytes written excluding NUL, or 0 on failure.
using FormatFn = size_t (*)(void* context, char* out, size_t out_capacity);

class StatusRegistry
{
 public:
  static constexpr size_t kMaxChannels = 8;

  bool Register(const char* name, FormatFn format, void* context)
  {
    if (name == nullptr || format == nullptr || count_ >= kMaxChannels)
    {
      return false;
    }
    for (size_t i = 0; i < count_; ++i)
    {
      if (NamesEqual(channels_[i].name, name))
      {
        channels_[i].format = format;
        channels_[i].context = context;
        return true;
      }
    }
    channels_[count_].name = name;
    channels_[count_].format = format;
    channels_[count_].context = context;
    ++count_;
    return true;
  }

  size_t count() const { return count_; }

  const char* name_at(size_t index) const
  {
    return index < count_ ? channels_[index].name : nullptr;
  }

  // Formats channel |name| into |out|. Returns false if unknown.
  bool Format(const char* name, char* out, size_t out_capacity) const
  {
    if (out == nullptr || out_capacity == 0)
    {
      return false;
    }
    out[0] = '\0';
    for (size_t i = 0; i < count_; ++i)
    {
      if (NamesEqual(channels_[i].name, name))
      {
        channels_[i].format(channels_[i].context, out, out_capacity);
        return true;
      }
    }
    return false;
  }

  // Writes space-separated channel names into |out|.
  size_t FormatList(char* out, size_t out_capacity) const
  {
    if (out == nullptr || out_capacity == 0)
    {
      return 0;
    }
    size_t pos = 0;
    for (size_t i = 0; i < count_; ++i)
    {
      const char* name = channels_[i].name;
      if (i > 0 && pos + 1 < out_capacity)
      {
        out[pos++] = ' ';
      }
      for (size_t n = 0; name[n] != '\0' && pos + 1 < out_capacity; ++n)
      {
        out[pos++] = name[n];
      }
    }
    out[pos] = '\0';
    return pos;
  }

 private:
  struct Channel
  {
    const char* name = nullptr;
    FormatFn format = nullptr;
    void* context = nullptr;
  };

  static bool NamesEqual(const char* a, const char* b)
  {
    if (a == nullptr || b == nullptr)
    {
      return false;
    }
    while (*a != '\0' && *a == *b)
    {
      ++a;
      ++b;
    }
    return *a == *b;
  }

  Channel channels_[kMaxChannels] = {};
  size_t count_ = 0;
};

}  // namespace telemetry
