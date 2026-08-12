#include "middleware/communication/binary_link.h"

#include "middleware/communication/binary_link_policy.h"

#include <cstring>
#include <string_view>

namespace middleware::communication
{

BinaryLink::BinaryLink(hal::FDCan* can, uint8_t node_id)
    : can_(can), node_id_(node_id)
{
}

void BinaryLink::SetCommandHandler(CommandHandler handler, void* context)
{
  handler_ = handler;
  context_ = context;
}

bool BinaryLink::SendRaw(uint16_t can_id, const void* data, size_t len)
{
  if (can_ == nullptr || data == nullptr || len == 0 || len > 64)
  {
    return false;
  }
  hal::FDCan::SendOptions opts;
  opts.extended_id = hal::FDCan::Override::DISABLE;
  opts.fdcan_frame = hal::FDCan::Override::REQUIRE;
  opts.bitrate_switch = hal::FDCan::Override::REQUIRE;
  return can_->Send(can_id, std::string_view(static_cast<const char*>(data), len),
                    opts);
}

void BinaryLink::SendAck(uint8_t cmd, uint8_t seq, uint8_t status)
{
  protocol::xt_can::Ack ack{};
  ack.hdr.magic = protocol::xt_can::kMagic;
  ack.hdr.ver = protocol::xt_can::kVersion;
  ack.hdr.type = protocol::xt_can::kTypeAck;
  ack.hdr.seq = seq;
  ack.cmd = cmd;
  ack.status = status;
  ack.reserved = 0;
  (void)SendRaw(tel_id(), &ack, sizeof(ack));
}

void BinaryLink::SendTelemetry(const protocol::xt_can::Telemetry& telem)
{
  protocol::xt_can::Telemetry out = telem;
  out.hdr.magic = protocol::xt_can::kMagic;
  out.hdr.ver = protocol::xt_can::kVersion;
  out.hdr.type = protocol::xt_can::kTypeTel;
  out.hdr.seq = tel_seq_++;
  (void)SendRaw(tel_id(), &out, sizeof(out));
}

void BinaryLink::SendEncTelem(const protocol::xt_can::EncTelem& enc)
{
  protocol::xt_can::EncTelem out = enc;
  out.hdr.magic = protocol::xt_can::kMagic;
  out.hdr.ver = protocol::xt_can::kVersion;
  out.hdr.type = protocol::xt_can::kTypeEnc;
  out.hdr.seq = tel_seq_++;
  (void)SendRaw(tel_id(), &out, sizeof(out));
}

void BinaryLink::SendCalTelem(const protocol::xt_can::CalTelem& cal)
{
  protocol::xt_can::CalTelem out = cal;
  out.hdr.magic = protocol::xt_can::kMagic;
  out.hdr.ver = protocol::xt_can::kVersion;
  out.hdr.type = protocol::xt_can::kTypeCal;
  out.hdr.seq = tel_seq_++;
  (void)SendRaw(tel_id(), &out, sizeof(out));
}

void BinaryLink::SendCtrlReply(const protocol::xt_can::CtrlReply& reply)
{
  protocol::xt_can::CtrlReply out = reply;
  out.hdr.magic = protocol::xt_can::kMagic;
  out.hdr.ver = protocol::xt_can::kVersion;
  out.hdr.type = protocol::xt_can::kTypeCtrlReply;
  // seq filled by caller (echo command seq)
  (void)SendRaw(tel_id(), &out, sizeof(out));
}

void BinaryLink::SendInfo(const protocol::xt_can::Info& info)
{
  protocol::xt_can::Info out = info;
  out.hdr.magic = protocol::xt_can::kMagic;
  out.hdr.ver = protocol::xt_can::kVersion;
  out.hdr.type = protocol::xt_can::kTypeInfo;
  // seq filled by caller (matches command seq)
  (void)SendRaw(tel_id(), &out, sizeof(out));
}

bool BinaryLink::SendConf(const void* data, size_t len)
{
  return SendRaw(tel_id(), data, len);
}

bool BinaryLink::SendSnapMeta(const protocol::xt_can::SnapMeta& meta)
{
  protocol::xt_can::SnapMeta out = meta;
  out.hdr.magic = protocol::xt_can::kMagic;
  out.hdr.ver = protocol::xt_can::kVersion;
  out.hdr.type = protocol::xt_can::kTypeSnapMeta;
  return SendRaw(tel_id(), &out, sizeof(out));
}

bool BinaryLink::SendSnapData(const protocol::xt_can::SnapData& data)
{
  protocol::xt_can::SnapData out = data;
  out.hdr.magic = protocol::xt_can::kMagic;
  out.hdr.ver = protocol::xt_can::kVersion;
  out.hdr.type = protocol::xt_can::kTypeSnapData;
  return SendRaw(tel_id(), &out, sizeof(out));
}

bool BinaryLink::HandleFrame(const FDCAN_RxHeaderTypeDef& header,
                             const uint8_t* data, size_t len)
{
  if (data == nullptr || can_ == nullptr)
  {
    return false;
  }
  if (!IsCommandFrameEnvelope(header.IdType == FDCAN_STANDARD_ID,
                              header.Identifier,
                              header.RxFrameType == FDCAN_DATA_FRAME, len,
                              cmd_id()))
  {
    return false;
  }

  protocol::xt_can::Header hdr;
  std::memcpy(&hdr, data, sizeof(hdr));
  if (!IsCommandProtocolHeader(hdr))
  {
    return false;
  }

  const uint8_t cmd = data[sizeof(protocol::xt_can::Header)];
  const uint8_t* payload = data + sizeof(protocol::xt_can::Header) + 1;
  const size_t payload_len = len - sizeof(protocol::xt_can::Header) - 1;

  uint8_t status = protocol::xt_can::kStatusBadCmd;
  if (handler_ != nullptr)
  {
    status = handler_(context_, cmd, hdr.seq, payload, payload_len);
  }
  // Stream/query commands reply through the configured telemetry publisher.
  if (!protocol::xt_can::UsesCtrlReply(cmd))
  {
    SendAck(cmd, hdr.seq, status);
  }
  return true;
}

}  // namespace middleware::communication
