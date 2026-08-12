// Pure receive-frame validation shared by BinaryLink and host tests.
#pragma once

#include <cstddef>
#include <cstdint>

#include "protocol/xt_can.h"

namespace middleware::communication
{

constexpr bool IsCommandFrameEnvelope(bool standard_id, uint32_t identifier,
                                      bool data_frame, size_t length,
                                      uint16_t command_id)
{
  return standard_id && identifier == command_id && data_frame &&
         length >= sizeof(protocol::xt_can::Header) + 1;
}

constexpr bool IsCommandProtocolHeader(const protocol::xt_can::Header& header)
{
  return header.magic == protocol::xt_can::kMagic &&
         header.ver == protocol::xt_can::kVersion &&
         header.type == protocol::xt_can::kTypeCmd;
}

}  // namespace middleware::communication
