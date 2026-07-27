// Bootloader multiplex flash command server (echo/unlock/lock/w/r/reset).
#pragma once

#include <cstdint>
#include <string_view>

#include "stm32g4xx.h"
#include "boot_mjlib.h"
#include "BL_CanDriver.h"
#include "BL_FlashWriter.h"

enum class BL_FlashSessionState
{
  LOCKED,
  UNLOCKED,
};

class BL_CommandServer
{
 public:
  BL_CommandServer(uint8_t can_id, FDCAN_GlobalTypeDef* fdcan);

  void Step();

  BL_FlashSessionState flash_state() const { return flash_state_; }

 private:
  template <typename T>
  struct Buffer
 {
    T data[256] = {};
    size_t pos = 0;

    std::string_view view() const
    {
      return {reinterpret_cast<const char*>(data), pos};
    }
    size_t capacity() const { return sizeof(data) / sizeof(*data); }
    mjlib::base::BufferWriteStream writer()
    {
      return mjlib::base::BufferWriteStream(
          mjlib::base::string_span(&data[pos], capacity() - pos));
    }
  };

  void ReadFrame();
  void WriteResponse(uint8_t host_id, int max_bytes, const BL_CanFrame& source);
  void RunLine();
  void ReadFlash(const std::string_view& addr_str,
                 const std::string_view& size_str,
                 mjlib::base::BufferWriteStream& writer);
  void WriteFlash(const std::string_view& addr_str,
                  const std::string_view& data_str,
                  mjlib::base::BufferWriteStream& writer);
  bool WriteByte(uint32_t address,
                 uint8_t byte,
                 mjlib::base::BufferWriteStream& writer);

  const uint8_t id_;
  BL_CanDriver driver_;
  BL_FlashWriter flash_;
  BL_FlashSessionState flash_state_ = BL_FlashSessionState::LOCKED;

  Buffer<char> command_;
  Buffer<char> response_;
  Buffer<char> out_frame_;
};
