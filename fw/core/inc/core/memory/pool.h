// Bump-allocator pool (moteus/mjlib-style). Never frees individual blocks.
#pragma once

#include <cstddef>
#include <cstdint>
#include <new>
#include <utility>

namespace core::memory
{

class Pool
{
 public:
  Pool(char* data, std::size_t size) : data_(data), size_(size) {}

  void* Allocate(std::size_t size, std::size_t alignment)
  {
    if (alignment == 0)
    {
      alignment = 1;
    }

    const std::uintptr_t base = reinterpret_cast<std::uintptr_t>(data_);
    const std::uintptr_t raw = base + position_;
    const std::uintptr_t aligned =
        (raw + static_cast<std::uintptr_t>(alignment) - 1u) &
        ~(static_cast<std::uintptr_t>(alignment) - 1u);
    const std::size_t offset = static_cast<std::size_t>(aligned - base);

    if (offset > size_ || size > (size_ - offset))
    {
      // Pool exhausted: halt (no heap fallback).
      while (true)
      {
      }
    }

    position_ = offset + size;
    return reinterpret_cast<void*>(aligned);
  }

  std::size_t size() const { return size_; }
  std::size_t available() const { return size_ - position_; }
  std::size_t used() const { return position_; }

 private:
  char* const data_;
  const std::size_t size_;
  std::size_t position_ = 0;
};

template <std::size_t N = 8192>
class SizedPool : public Pool
{
 public:
  SizedPool() : Pool(data_, N) {}

  static constexpr std::size_t kCapacity = N;

 private:
  alignas(8) char data_[N] = {};
};

// Allocates from a Pool and never destroys (same contract as mjlib PoolPtr).
template <typename T>
class PoolPtr
{
 public:
  template <typename... Args>
  PoolPtr(Pool* pool, Args&&... args)
      : ptr_(reinterpret_cast<T*>(
            pool->Allocate(sizeof(T), alignof(T))))
  {
    new (ptr_) T(std::forward<Args>(args)...);
  }

  T& operator*() { return *ptr_; }
  const T& operator*() const { return *ptr_; }
  T* operator->() { return ptr_; }
  const T* operator->() const { return ptr_; }
  T* get() { return ptr_; }
  const T* get() const { return ptr_; }

 private:
  T* const ptr_;
};

}  // namespace core::memory
