// APP multiplex text tunnel (same framing family as the bootloader).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "HAL/fdcan.h"
#include "boot_mjlib.h"
#include "telemetry/status_registry.h"

namespace telemetry
{

class DiagnosticServer
{
 public:
  // Return true if the verb was handled (response already written).
  using AppCommandHandler =
      bool (*)(void* context, std::string_view verb,
               mjlib::base::Tokenizer& tokenizer,
               mjlib::base::BufferWriteStream& writer);

  DiagnosticServer(hal::FDCan* can, uint8_t can_id, StatusRegistry* registry);

  void SetAppCommandHandler(AppCommandHandler handler, void* context)
  {
    app_handler_ = handler;
    app_context_ = context;
  }

  // Handle one RX frame. Returns true if it was a multiplex frame for us.
  bool HandleFrame(const FDCAN_RxHeaderTypeDef& header,
                   const uint8_t* data,
                   size_t len);

  // Parse and run any complete command line buffered from HandleFrame.
  void RunLine();

 private:
  template <typename T>
  struct Buffer
  {
    // Must fit "cur " + format_body_ + "\r\n" (was 256 → cur timed out).
    T data[512] = {};
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

  void WriteResponse(uint8_t host_id, int max_bytes, bool bitrate_switch);
  void AppendResponse(std::string_view text);
  void HandleCommand(std::string_view verb,
                     mjlib::base::Tokenizer& tokenizer,
                     mjlib::base::BufferWriteStream& writer);

  hal::FDCan* can_ = nullptr;
  uint8_t can_id_ = 1;
  StatusRegistry* registry_ = nullptr;
  AppCommandHandler app_handler_ = nullptr;
  void* app_context_ = nullptr;

  Buffer<char> command_;
  Buffer<char> response_;
  Buffer<char> out_frame_;

  // Kept off the MSP stack: HandleCommand previously used ~240B locals.
  char channel_name_[32] = {};
  char format_body_[448] = {};
  char list_buf_[128] = {};
};

}  // namespace telemetry
