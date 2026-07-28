#include "diagnostic_server.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <optional>

#include "BL_Config.h"

namespace telemetry
{
namespace
{

struct MultiplexFormat
{
  enum class Subframe : uint8_t
  {
    CLIENT_TO_SERVER = 0x40,
    SERVER_TO_CLIENT = 0x41,
    CLIENT_POLL_SERVER = 0x42,
  };
};

void WriteVaruint(mjlib::base::BufferWriteStream& ostr, uint32_t value)
{
  do
  {
    if (ostr.remaining() <= 0)
    {
      return;
    }
    uint8_t this_byte = value & 0x7f;
    value >>= 7;
    this_byte |= value ? 0x80 : 0x00;
    *reinterpret_cast<uint8_t*>(ostr.position()) = this_byte;
    ostr.skip(1);
  } while (value);
}

std::optional<uint32_t> ReadVaruint(mjlib::base::BufferReadStream& istr)
{
  uint32_t result = 0;
  const uint8_t* position = reinterpret_cast<const uint8_t*>(istr.position());
  auto remaining = istr.remaining();

  int pos = 0;
  int i = 0;
  for (; i < 5; i++)
  {
    if (remaining == 0)
    {
      istr.fast_ignore(i);
      return {};
    }
    remaining--;
    const auto this_byte = *position;
    position++;
    result |= (this_byte & 0x7f) << pos;
    pos += 7;
    if ((this_byte & 0x80) == 0)
    {
      istr.fast_ignore(i + 1);
      return result;
    }
  }
  istr.fast_ignore(i);
  return std::numeric_limits<uint32_t>::max();
}

uint32_t U32(MultiplexFormat::Subframe v)
{
  return static_cast<uint32_t>(v);
}

void CopyName(char* dest, size_t dest_size, std::string_view name)
{
  if (dest == nullptr || dest_size == 0)
  {
    return;
  }
  const size_t n = std::min(name.size(), dest_size - 1);
  std::memcpy(dest, name.data(), n);
  dest[n] = '\0';
}

}  // namespace

DiagnosticServer::DiagnosticServer(hal::FDCan* can,
                                   uint8_t can_id,
                                   StatusRegistry* registry)
    : can_(can), can_id_(can_id), registry_(registry)
{
  AppendResponse("xtellar app telemetry 1\r\n");
}

bool DiagnosticServer::HandleFrame(const FDCAN_RxHeaderTypeDef& header,
                                   const uint8_t* data,
                                   size_t len)
{
  if (can_ == nullptr || data == nullptr)
  {
    return false;
  }
  if (header.IdType != FDCAN_EXTENDED_ID)
  {
    return false;
  }

  const uint32_t identifier = header.Identifier;
  const uint8_t source_id = (identifier >> 8) & 0xFF;
  const uint8_t dest_id = identifier & 0xFF;
  if (dest_id != can_id_ && dest_id != 0x7F)
  {
    return false;
  }

  mjlib::base::BufferReadStream buffer_stream{std::string_view(
      reinterpret_cast<const char*>(data), len)};

  auto maybe_subframe_id = ReadVaruint(buffer_stream);
  if (!maybe_subframe_id)
  {
    return false;
  }

  if (*maybe_subframe_id != U32(MultiplexFormat::Subframe::CLIENT_TO_SERVER) &&
      *maybe_subframe_id != U32(MultiplexFormat::Subframe::CLIENT_POLL_SERVER))
  {
    return false;
  }

  const bool poll_only =
      (*maybe_subframe_id == U32(MultiplexFormat::Subframe::CLIENT_POLL_SERVER));
  const bool query = (source_id & 0x80) != 0;

  auto maybe_channel = ReadVaruint(buffer_stream);
  if (!maybe_channel || *maybe_channel != kTunnelChannel)
  {
    return false;
  }

  auto maybe_bytes = ReadVaruint(buffer_stream);
  if (!maybe_bytes)
  {
    return false;
  }

  if (!poll_only && *maybe_bytes > 0)
  {
    const size_t bytes = *maybe_bytes;
    if (static_cast<size_t>(buffer_stream.remaining()) < bytes)
    {
      return false;
    }
    if (command_.pos + bytes > command_.capacity())
    {
      command_.pos = 0;
      return true;
    }
    std::memcpy(&command_.data[command_.pos], buffer_stream.position(), bytes);
    buffer_stream.fast_ignore(static_cast<std::streamsize>(bytes));
    command_.pos += bytes;
  }

  if (query)
  {
    WriteResponse(source_id & 0x7F,
                  poll_only ? static_cast<int>(*maybe_bytes) : -1,
                  header.BitRateSwitch == FDCAN_BRS_ON);
  }
  return true;
}

void DiagnosticServer::WriteResponse(uint8_t host_id,
                                     int max_bytes,
                                     bool bitrate_switch)
{
  out_frame_.pos = 0;
  auto buffer_stream = out_frame_.writer();

  WriteVaruint(buffer_stream, U32(MultiplexFormat::Subframe::SERVER_TO_CLIENT));
  WriteVaruint(buffer_stream, kTunnelChannel);

  constexpr size_t kMaxCanPayload = 64 - 3;
  const size_t bytes_to_write = std::min<size_t>(
      kMaxCanPayload,
      max_bytes >= 0 ? std::min<size_t>(static_cast<size_t>(max_bytes), response_.pos)
                     : response_.pos);

  WriteVaruint(buffer_stream, static_cast<uint32_t>(bytes_to_write));
  buffer_stream.write(response_.view().substr(0, bytes_to_write));
  out_frame_.pos = buffer_stream.offset();

  std::memmove(&response_.data[0], &response_.data[bytes_to_write],
               response_.pos - bytes_to_write);
  response_.pos -= bytes_to_write;

  const uint32_t response_id = (static_cast<uint32_t>(can_id_) << 8) | host_id;
  hal::FDCan::SendOptions send_options;
  send_options.fdcan_frame = hal::FDCan::Override::REQUIRE;
  // Host multiplex tools always use extended IDs (moteus-compatible).
  send_options.extended_id = hal::FDCan::Override::REQUIRE;
  send_options.bitrate_switch = bitrate_switch ? hal::FDCan::Override::REQUIRE
                                               : hal::FDCan::Override::DISABLE;
  (void)can_->Send(response_id, out_frame_.view(), send_options);
}

void DiagnosticServer::AppendResponse(std::string_view text)
{
  auto writer = response_.writer();
  writer.write(text);
  response_.pos += writer.offset();
}

void DiagnosticServer::RunLine()
{
  auto writer = response_.writer();
  const auto command_end = command_.view().find_first_of("\r\n");
  if (command_end == std::string_view::npos)
  {
    return;
  }

  mjlib::base::Tokenizer tokenizer(command_.view(), " \r\n");
  const auto next = tokenizer.next();
  if (!next.empty())
  {
    HandleCommand(next, tokenizer, writer);
  }

  response_.pos += writer.offset();

  const auto to_consume = command_end + 1;
  std::memmove(command_.data, command_.data + to_consume,
               command_.capacity() - to_consume);
  command_.pos -= to_consume;
}

void DiagnosticServer::HandleCommand(std::string_view verb,
                                     mjlib::base::Tokenizer& tokenizer,
                                     mjlib::base::BufferWriteStream& writer)
{
  if (verb == "echo")
  {
    writer.write(tokenizer.remaining());
    return;
  }

  if (verb == "list")
  {
    list_buf_[0] = '\0';
    if (registry_ != nullptr)
    {
      registry_->FormatList(list_buf_, sizeof(list_buf_));
    }
    writer.write(list_buf_);
    writer.write("\r\n");
    return;
  }

  channel_name_[0] = '\0';
  if (verb == "get")
  {
    const auto name = tokenizer.next();
    if (name.empty())
    {
      writer.write("ERR usage: get <channel>\r\n");
      return;
    }
    CopyName(channel_name_, sizeof(channel_name_), name);
  } else if (verb == "drv") {
    CopyName(channel_name_, sizeof(channel_name_), "drv8353s");
  } else if (verb == "status") {
    CopyName(channel_name_, sizeof(channel_name_), "status");
  } else {
    writer.write("ERR unknown command\r\n");
    return;
  }

  format_body_[0] = '\0';
  if (registry_ == nullptr ||
      !registry_->Format(channel_name_, format_body_, sizeof(format_body_)))
  {
    writer.write("ERR unknown channel\r\n");
    return;
  }
  writer.write(channel_name_);
  writer.write(" ");
  writer.write(format_body_);
  writer.write("\r\n");
}

}  // namespace telemetry

namespace mjlib
{
namespace base
{

void __attribute__((weak)) assertion_failed(const char* /*expression*/,
                                             const char* /*filename*/,
                                             int /*line*/)
{
  while (true)
  {
  }
}

}  // namespace base
}  // namespace mjlib
