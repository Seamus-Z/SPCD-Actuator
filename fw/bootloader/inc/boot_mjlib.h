// Minimal inline replacements for mjlib types used by the production bootloader.
// This avoids external dependencies — everything the multiplex bootloader needs is here.
//
// Original sources (Apache 2.0, mjbots Robotic Systems):
//   mjlib/base/string_span.h
//   mjlib/base/stream.h
//   mjlib/base/buffer_stream.h
//   mjlib/base/tokenizer.h
//   mjlib/base/assert.h
//   mjlib/multiplex/format.h
//   mjlib/multiplex/stream.h
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string_view>

// Avoid <ios> — it pulls in <system_error> which requires libstdc++ linking.
// We only need std::streamsize which is just a signed integer type.
namespace std
{
using streamsize = std::ptrdiff_t;
}

// ============================================================================
// mjlib::base::string_span
// ============================================================================
namespace mjlib
{
namespace base
{

class string_span
{
 public:
  using iterator = char*;
  using const_iterator = const char*;
  using index_type = std::ptrdiff_t;
  using value_type = char;
  using pointer = char*;
  using const_pointer = const char*;
  using reference = char&;
  using const_reference = const char&;

  constexpr string_span() noexcept = default;
  constexpr string_span(const string_span& other) noexcept = default;
  constexpr string_span& operator=(const string_span& other) noexcept = default;
  constexpr string_span(pointer ptr, index_type size) : ptr_(ptr), size_(size) {}
  constexpr string_span(iterator begin, iterator end) : ptr_(begin), size_(end - begin) {}

  template <std::size_t Size>
  constexpr string_span(char (&array)[Size]) : ptr_(array), size_(Size) {}

  constexpr reference operator[](index_type index) const { return ptr_[index]; }
  constexpr pointer data() const { return ptr_; }
  constexpr index_type length() const noexcept { return size_; }
  constexpr index_type size() const noexcept { return size_; }
  constexpr bool empty() const noexcept { return size_ == 0; }
  constexpr iterator begin() const noexcept { return ptr_; }
  constexpr iterator end() const noexcept { return ptr_ + size_; }

 private:
  char* ptr_ = nullptr;
  index_type size_ = 0;
};

// ============================================================================
// mjlib::base::ReadStream / WriteStream (abstract)
// ============================================================================
class WriteStream
{
 public:
  virtual ~WriteStream() {}
  virtual void write(const std::string_view&) = 0;
};

class ReadStream
{
 public:
  virtual ~ReadStream() {}
  virtual void ignore(std::streamsize) = 0;
  virtual void read(const string_span&) = 0;
  virtual std::streamsize gcount() const = 0;
};

// ============================================================================
// mjlib::base::BufferWriteStream / BufferReadStream
// ============================================================================
class BufferWriteStream : public WriteStream
{
 public:
  BufferWriteStream(const string_span& buffer) : buffer_(buffer) {}

  void write(const std::string_view& data) override
  {
    if (static_cast<std::streamsize>(offset_ + data.size()) <= buffer_.size())
    {
      std::memcpy(&buffer_[offset_], data.data(), data.size());
      offset_ += data.size();
    }
  }

  void skip(std::streamsize amount)
  {
    if ((offset_ + amount) <= static_cast<std::streamsize>(buffer_.size()))
    {
      offset_ += amount;
    }
  }

  std::streamsize offset() const noexcept { return offset_; }
  std::streamsize remaining() const noexcept { return buffer_.size() - offset_; }
  std::streamsize size() const noexcept { return buffer_.size(); }
  char* position() { return &buffer_[offset_]; }

 private:
  const string_span buffer_;
  std::streamsize offset_ = 0;
};

class BufferReadStream : public ReadStream
{
 public:
  BufferReadStream(const std::string_view& buffer) : buffer_(buffer) {}

  void ignore(std::streamsize amount) override
  {
    std::streamsize to_ignore = std::min<std::streamsize>(amount, buffer_.size() - offset_);
    fast_ignore(to_ignore);
  }

  void fast_ignore(std::streamsize to_ignore)
  {
    last_read_ = to_ignore;
    offset_ += last_read_;
  }

  void read(const string_span& buffer) override
  {
    std::streamsize to_read = std::min<std::streamsize>(buffer.size(), buffer_.size() - offset_);
    last_read_ = to_read;
    std::memcpy(buffer.data(), &buffer_[offset_], last_read_);
    offset_ += last_read_;
  }

  std::streamsize gcount() const override { return last_read_; }
  std::streamsize offset() const noexcept { return offset_; }
  const char* position() const noexcept { return &buffer_[offset_]; }
  std::streamsize remaining() const noexcept { return buffer_.size() - offset_; }

 private:
  const std::string_view buffer_;
  std::streamsize offset_ = 0;
  std::streamsize last_read_ = 0;
};

// ============================================================================
// mjlib::base::Tokenizer
// ============================================================================
class Tokenizer
{
 public:
  Tokenizer(const std::string_view& source, const char* delimiters)
      : source_(source), delimiters_(delimiters), position_(source_.cbegin()) {}

  std::string_view next()
  {
    if (position_ == source_.end()) { return {}; }
    const auto start = position_;
    auto my_next = position_;
    bool found = false;
    for (; my_next != source_.end(); ++my_next)
    {
      if (std::strchr(delimiters_, *my_next) != nullptr)
      {
        position_ = my_next;
        ++position_;
        found = true;
        break;
      }
    }
    if (!found) { position_ = my_next; }
    return std::string_view(&*start, my_next - start);
  }

  std::string_view remaining() const
  {
    if (position_ == source_.end()) { return {}; }
    return std::string_view(&*position_, source_.end() - position_);
  }

 private:
  const std::string_view source_;
  const char* const delimiters_;
  std::string_view::const_iterator position_;
};

// ============================================================================
// mjlib::base::assertion_failed (weak — app can override)
// ============================================================================
void assertion_failed(const char* expression, const char* filename, int line);

}  // namespace base
}  // namespace mjlib

#define MJ_ASSERT(expr) \
  !!(expr) ? ((void)0) : ::mjlib::base::assertion_failed(#expr, __FILE__, __LINE__)
