// GUI / Monitor binary CAN link (commands + telemetry + snapshot).
#pragma once

#include <cstddef>
#include <cstdint>

#include "HAL/fdcan.h"
#include "telemetry/xt_can.h"

namespace telemetry
{

class BinaryLink
{
 public:
  // Return status byte for ACK (xt_can::kStatus*).
  using CommandHandler = uint8_t (*)(void* context, uint8_t cmd, uint8_t seq,
                                     const uint8_t* payload, size_t payload_len);

  BinaryLink(hal::FDCan* can, uint8_t node_id);

  void SetCommandHandler(CommandHandler handler, void* context);

  // Handle one RX frame if it matches CmdId; sends ACK on success/fail.
  bool HandleFrame(const FDCAN_RxHeaderTypeDef& header, const uint8_t* data,
                   size_t len);

  void SendTelemetry(const xt_can::Telemetry& telem);
  void SendEncTelem(const xt_can::EncTelem& enc);
  void SendCalTelem(const xt_can::CalTelem& cal);
  void SendCtrlReply(const xt_can::CtrlReply& reply);
  void SendAck(uint8_t cmd, uint8_t seq, uint8_t status);
  void SendInfo(const xt_can::Info& info);
  void SendSnapMeta(const xt_can::SnapMeta& meta);
  void SendSnapData(const xt_can::SnapData& data);

  uint8_t node_id() const { return node_id_; }
  uint16_t cmd_id() const { return xt_can::CmdId(node_id_); }
  uint16_t tel_id() const { return xt_can::TelId(node_id_); }

 private:
  bool SendRaw(uint16_t can_id, const void* data, size_t len);

  hal::FDCan* can_ = nullptr;
  uint8_t node_id_ = 1;
  uint8_t tel_seq_ = 0;
  CommandHandler handler_ = nullptr;
  void* context_ = nullptr;
};

}  // namespace telemetry
