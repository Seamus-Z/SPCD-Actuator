#include "BL_CommandServer.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <optional>

#include "BL_Config.h"

namespace
{

namespace multiplex
{

struct Format
{
  enum class Subframe : uint8_t
  {
    kClientToServer = 0x40,
    kServerToClient = 0x41,
    kClientPollServer = 0x42,
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

}  // namespace multiplex

template <typename T>
uint32_t u32(T v)
{
  return static_cast<uint32_t>(v);
}

void uint8_hex(uint8_t value, char* buffer)
{
  constexpr char digits[] = "0123456789ABCDEF";
  buffer[0] = digits[value >> 4];
  buffer[1] = digits[value & 0x0f];
  buffer[2] = 0;
}

void uint32_hex(uint32_t value, char* buffer)
{
  for (int i = 0; i < 4; i++)
  {
    uint8_hex((value >> ((3 - i) * 8)) & 0xFF, &buffer[i * 2]);
  }
}

std::optional<uint32_t> hex_to_i(const std::string_view& str)
{
  if (str.empty() || str.size() > 8)
  {
    return {};
  }
  uint32_t result = 0;
  for (char c : str)
  {
    result <<= 4;
    if (c >= '0' && c <= '9')
    {
      result |= static_cast<uint32_t>(c - '0');
    } else if (c >= 'a' && c <= 'f') {
      result |= static_cast<uint32_t>(c - 'a' + 0x0a);
    } else if (c >= 'A' && c <= 'F') {
      result |= static_cast<uint32_t>(c - 'A' + 0x0a);
    } else {
      return {};
    }
  }
  return result;
}

bool addr_in_range(uint32_t addr, uint32_t size, uint32_t base, uint32_t end)
{
  return addr >= base && addr <= end && size <= end - addr;
}

}  // namespace

BL_CommandServer::BL_CommandServer(uint8_t can_id, FDCAN_GlobalTypeDef* fdcan)
    : id_(can_id), driver_(fdcan)
{
  driver_.Init();

  auto writer = response_.writer();
  writer.write("multiplex bootloader protocol 1 ");
  writer.write(__DATE__);
  writer.write(" ");
  writer.write(__TIME__);
  writer.write("\r\n");
  response_.pos = writer.offset();
}

void BL_CommandServer::Step()
{
  ReadFrame();
  RunLine();
}

void BL_CommandServer::ReadFrame()
{
  BL_CanFrame frame;
  while (!driver_.Poll(frame))
  {
  }

  const uint8_t source_id = (frame.identifier >> 8) & 0xFF;
  const uint8_t dest_id = (frame.identifier & 0xFF);
  if (dest_id != id_ && dest_id != 0x7F)
  {
    return;
  }

  mjlib::base::BufferReadStream buffer_stream{std::string_view(
      reinterpret_cast<const char*>(frame.data), frame.size)};

  auto maybe_subframe_id = multiplex::ReadVaruint(buffer_stream);
  if (!maybe_subframe_id)
  {
    return;
  }

  using SF = multiplex::Format::Subframe;
  if (*maybe_subframe_id != u32(SF::kClientToServer) &&
      *maybe_subframe_id != u32(SF::kClientPollServer))
  {
    return;
  }

  const bool poll_only = (*maybe_subframe_id == u32(SF::kClientPollServer));
  const bool query = (source_id & 0x80) != 0;

  auto maybe_channel = multiplex::ReadVaruint(buffer_stream);
  if (!maybe_channel || *maybe_channel != kTunnelChannel)
  {
    return;
  }

  auto maybe_bytes = multiplex::ReadVaruint(buffer_stream);
  if (!maybe_bytes)
  {
    return;
  }

  if (!poll_only && *maybe_bytes > 0)
  {
    const size_t bytes = *maybe_bytes;
    if (static_cast<size_t>(buffer_stream.remaining()) < bytes)
    {
      return;
    }
    if (command_.pos + bytes > command_.capacity())
    {
      command_.pos = 0;
      return;
    }
    std::memcpy(&command_.data[command_.pos], buffer_stream.position(), bytes);
    buffer_stream.fast_ignore(static_cast<std::streamsize>(bytes));
    command_.pos += bytes;
  }

  if (query)
  {
    WriteResponse(source_id & 0x7F, poll_only ? static_cast<int>(*maybe_bytes) : -1,
                  frame);
  }
}

void BL_CommandServer::WriteResponse(uint8_t host_id,
                                     int max_bytes,
                                     const BL_CanFrame& source_frame)
{
  out_frame_.pos = 0;
  auto buffer_stream = out_frame_.writer();

  multiplex::WriteVaruint(buffer_stream,
                          u32(multiplex::Format::Subframe::kServerToClient));
  multiplex::WriteVaruint(buffer_stream, kTunnelChannel);

  constexpr size_t kMaxCanPayload = 64 - 3;
  const size_t bytes_to_write = std::min<size_t>(
      kMaxCanPayload,
      max_bytes >= 0 ? std::min<size_t>(static_cast<size_t>(max_bytes), response_.pos)
                     : response_.pos);

  multiplex::WriteVaruint(buffer_stream, static_cast<uint32_t>(bytes_to_write));
  buffer_stream.write(response_.view().substr(0, bytes_to_write));
  out_frame_.pos = buffer_stream.offset();

  std::memmove(&response_.data[0], &response_.data[bytes_to_write],
               response_.pos - bytes_to_write);
  response_.pos -= bytes_to_write;

  (void)driver_.Send((static_cast<uint32_t>(id_) << 8) | host_id,
                     out_frame_.view(), source_frame.bit_rate_switch != 0);
}

void BL_CommandServer::RunLine()
{
  auto writer = response_.writer();
  const auto command_end = command_.view().find_first_of("\r\n");
  if (command_end == std::string_view::npos)
  {
    return;
  }

  mjlib::base::Tokenizer tokenizer(command_.view(), " \r\n");
  const auto next = tokenizer.next();
  if (next.empty())
  {
    // Ignore blank lines.
  } else if (next == "echo") {
    writer.write(tokenizer.remaining());
  } else if (next == "unlock") {
    flash_.Unlock();
    flash_state_ = BL_FlashSessionState::UNLOCKED;
    writer.write("OK\r\n");
  } else if (next == "lock") {
    if (flash_.Lock())
    {
      writer.write("ERR error locking\r\n");
    } else {
      flash_state_ = BL_FlashSessionState::LOCKED;
      writer.write("OK\r\n");
    }
  } else if (next == "w") {
    const auto addr_str = tokenizer.next();
    const auto data_str = tokenizer.next();
    if (addr_str.empty() || data_str.empty())
    {
      writer.write("ERR malformed write\r\n");
    } else {
      WriteFlash(addr_str, data_str, writer);
    }
  } else if (next == "r") {
    const auto addr_str = tokenizer.next();
    const auto size_str = tokenizer.next();
    if (addr_str.empty() || size_str.empty())
    {
      writer.write("ERR malformed read\r\n");
    } else {
      ReadFlash(addr_str, size_str, writer);
    }
  } else if (next == "reset") {
    if (flash_.Lock())
    {
      writer.write("ERR error locking\r\n");
    } else {
      flash_state_ = BL_FlashSessionState::LOCKED;
      *reinterpret_cast<volatile uint32_t*>(kBootMagicAddr) = 0;
      NVIC_SystemReset();
    }
  } else {
    writer.write("ERR unknown command\r\n");
  }

  response_.pos += writer.offset();

  const auto to_consume = command_end + 1;
  std::memmove(command_.data, command_.data + to_consume,
               command_.capacity() - to_consume);
  command_.pos -= to_consume;
}

void BL_CommandServer::ReadFlash(const std::string_view& addr_str,
                                 const std::string_view& size_str,
                                 mjlib::base::BufferWriteStream& writer)
{
  char buf[10] = {};
  const auto maybe_addr = hex_to_i(addr_str);
  const auto maybe_size = hex_to_i(size_str);
  if (!maybe_addr || !maybe_size)
  {
    writer.write("ERR malformed hex\r\n");
    return;
  }
  const uint32_t addr = *maybe_addr;
  const uint32_t size = *maybe_size;
  if (size > 32)
  {
    writer.write("size too big\r\n");
    return;
  }
  if (!addr_in_range(addr, size, 0x08000000, kFlashEnd) &&
      !addr_in_range(addr, size, 0x20000000, 0x20020000) &&
      !addr_in_range(addr, size, 0x10000000, 0x10008000))
  {
    writer.write("ERR address not readable\r\n");
    return;
  }

  uint32_hex(addr, buf);
  writer.write(buf);
  writer.write(" ");
  for (uint32_t i = 0; i < size; i++)
  {
    const uint8_t* val = reinterpret_cast<const uint8_t*>(addr + i);
    uint8_hex(*val, buf);
    writer.write(buf);
  }
  writer.write("\r\n");
}

void BL_CommandServer::WriteFlash(const std::string_view& addr_str,
                                  const std::string_view& data_str,
                                  mjlib::base::BufferWriteStream& writer)
{
  if (data_str.size() % 2 != 0)
  {
    writer.write("odd data size\r\n");
    return;
  }
  const auto maybe_addr = hex_to_i(addr_str);
  if (!maybe_addr)
  {
    writer.write("ERR malformed hex\r\n");
    return;
  }
  const uint32_t addr = *maybe_addr;
  const uint32_t bytes = static_cast<uint32_t>(data_str.size() / 2);
  for (uint32_t i = 0; i < bytes; i++)
  {
    const auto maybe_byte =
        hex_to_i(std::string_view(data_str.data() + i * 2, 2));
    if (!maybe_byte)
    {
      writer.write("ERR malformed hex\r\n");
      return;
    }
    if (!WriteByte(addr + i, static_cast<uint8_t>(*maybe_byte), writer))
    {
      return;
    }
  }
  writer.write("OK\r\n");
}

bool BL_CommandServer::WriteByte(uint32_t address,
                                 uint8_t byte,
                                 mjlib::base::BufferWriteStream& writer)
{
  if (flash_state_ != BL_FlashSessionState::UNLOCKED || flash_.locked())
  {
    writer.write("ERR flash is locked\r\n");
    return false;
  }
  if (address < 0x08000000 || address >= kFlashEnd)
  {
    writer.write("ERR address not in flash\r\n");
    return false;
  }
  if (address < kAppStart)
  {
    writer.write("ERR address not writable (bootloader region)\r\n");
    return false;
  }
  const auto err = flash_.ProgramByte(address, byte);
  if (err)
  {
    writer.write("ERR program error ");
    char buf[9] = {};
    uint32_hex(err, buf);
    writer.write(buf);
    writer.write("\r\n");
    return false;
  }
  return true;
}
