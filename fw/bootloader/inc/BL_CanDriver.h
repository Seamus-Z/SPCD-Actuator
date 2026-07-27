// Bootloader-only bare-metal FDCAN driver (register-level I/O).
// Not used by the application image — APP uses fw/HAL/fdcan instead.
#pragma once

#include <cstdint>
#include <string_view>

#include "stm32g4xx.h"

struct BL_CanFrame
{
  uint32_t identifier = 0;
  uint32_t dlc = 0;
  uint32_t bit_rate_switch = 0;
  uint32_t fdformat = 0;
  uint32_t size = 0;
  uint8_t data[64] = {};
};

class BL_CanDriver
{
 public:
  explicit BL_CanDriver(FDCAN_GlobalTypeDef* fdcan);

  void Init();
  bool Poll(BL_CanFrame& frame);
  bool Send(uint32_t identifier, const std::string_view& data, bool brs);

 private:
  FDCAN_GlobalTypeDef* const fdcan_;
  uint32_t rx_fifo0_offset_ = 0;
  uint32_t tx_fifoq_offset_ = 0;
  uint32_t rx_fifo0_addr_ = 0;
  uint32_t tx_fifoq_addr_ = 0;
};
