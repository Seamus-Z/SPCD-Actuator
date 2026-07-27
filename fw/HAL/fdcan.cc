#include "fdcan.h"

#include <algorithm>
#include <cstring>

namespace hal {
namespace {

[[noreturn]] void Halt() {
  while (true) {
  }
}

uint32_t RoundUpDlc(size_t size) {
  if (size == 0) { return FDCAN_DLC_BYTES_0; }
  if (size == 1) { return FDCAN_DLC_BYTES_1; }
  if (size == 2) { return FDCAN_DLC_BYTES_2; }
  if (size == 3) { return FDCAN_DLC_BYTES_3; }
  if (size == 4) { return FDCAN_DLC_BYTES_4; }
  if (size == 5) { return FDCAN_DLC_BYTES_5; }
  if (size == 6) { return FDCAN_DLC_BYTES_6; }
  if (size == 7) { return FDCAN_DLC_BYTES_7; }
  if (size == 8) { return FDCAN_DLC_BYTES_8; }
  if (size <= 12) { return FDCAN_DLC_BYTES_12; }
  if (size <= 16) { return FDCAN_DLC_BYTES_16; }
  if (size <= 20) { return FDCAN_DLC_BYTES_20; }
  if (size <= 24) { return FDCAN_DLC_BYTES_24; }
  if (size <= 32) { return FDCAN_DLC_BYTES_32; }
  if (size <= 48) { return FDCAN_DLC_BYTES_48; }
  if (size <= 64) { return FDCAN_DLC_BYTES_64; }
  return FDCAN_DLC_BYTES_64;
}

bool ApplyOverride(bool value, FDCan::Override o) {
  switch (o) {
    case FDCan::Override::kDefault: return value;
    case FDCan::Override::kRequire: return true;
    case FDCan::Override::kDisable: return false;
  }
  Halt();
}

FDCan::Rate MakeTime(int bitrate, int max_time_seg1, int max_time_seg2) {
  FDCan::Rate result;
  result.prescaler = 1;

  // App currently runs from reset HSI; keep SystemCoreClock coherent so
  // HAL_RCC_GetPCLK1Freq() yields a usable bit-timing base.
  if (SystemCoreClock == 0) {
    SystemCoreClock = 16000000;
  }

  const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  if (pclk1 == 0 || bitrate <= 0) {
    Halt();
  }

  while (true) {
    const uint32_t total_divisor = (pclk1 / static_cast<uint32_t>(result.prescaler)) /
                                   static_cast<uint32_t>(bitrate);
    if (total_divisor < 1) {
      Halt();
    }

    const auto actual_divisor = total_divisor - 1;
    result.time_seg2 = static_cast<int>(actual_divisor / 3);
    result.time_seg1 = static_cast<int>(actual_divisor - result.time_seg2);
    result.sync_jump_width = std::min(16, result.time_seg2);

    if (result.time_seg1 > max_time_seg1 || result.time_seg2 > max_time_seg2 ||
        result.time_seg1 < 1 || result.time_seg2 < 1) {
      result.prescaler++;
      continue;
    }
    break;
  }

  return result;
}

FDCan::Rate ApplyRateOverride(FDCan::Rate base, FDCan::Rate overlay) {
  if (overlay.prescaler >= 0) { base.prescaler = overlay.prescaler; }
  if (overlay.sync_jump_width >= 0) {
    base.sync_jump_width = overlay.sync_jump_width;
  }
  if (overlay.time_seg1 >= 0) { base.time_seg1 = overlay.time_seg1; }
  if (overlay.time_seg2 >= 0) { base.time_seg2 = overlay.time_seg2; }
  return base;
}

}  // namespace

FDCan::FDCan(const Options& options) : options_(options) {
  Init();
}

void FDCan::ConfigureFilters(const FilterConfig& filters) {
  options_.filters = filters;
  Init();
}

void FDCan::ConfigurePins() {
  // First-batch hardware map: FDCAN2 on PB5(RX AF9) / PB6(TX AF9).
  if (options_.instance != FDCAN2) {
    Halt();
  }

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF9_FDCAN2;
  HAL_GPIO_Init(GPIOB, &gpio);
}

void FDCan::Init() {
  // FDCAN kernel clock: PCLK1 (reset default is HSE, which is not enabled).
  __HAL_RCC_FDCAN_CLK_ENABLE();
  RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_FDCANSEL_Msk) |
               (2UL << RCC_CCIPR_FDCANSEL_Pos);

  ConfigurePins();

  std::memset(&handle_, 0, sizeof(handle_));
  handle_.Instance = options_.instance;
  handle_.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  handle_.Init.FrameFormat = [&]() {
    if (options_.fdcan_frame && options_.bitrate_switch) {
      return FDCAN_FRAME_FD_BRS;
    }
    if (options_.fdcan_frame) {
      return FDCAN_FRAME_FD_NO_BRS;
    }
    return FDCAN_FRAME_CLASSIC;
  }();
  handle_.Init.Mode = [&]() {
    if (options_.bus_monitor) {
      return FDCAN_MODE_BUS_MONITORING;
    }
    if (options_.restricted_mode) {
      return FDCAN_MODE_RESTRICTED_OPERATION;
    }
    return FDCAN_MODE_NORMAL;
  }();
  handle_.Init.AutoRetransmission =
      options_.automatic_retransmission ? ENABLE : DISABLE;
  handle_.Init.TransmitPause = ENABLE;
  // Match moteus: disable protocol exception so PXHD stays set.
  handle_.Init.ProtocolException = DISABLE;

  auto nominal = ApplyRateOverride(
      MakeTime(options_.slow_bitrate, 255, 127), options_.rate_override);
  auto fast = ApplyRateOverride(
      MakeTime(options_.fast_bitrate, 31, 15), options_.fdrate_override);

  config_.clock = static_cast<int>(HAL_RCC_GetPCLK1Freq());
  config_.nominal = nominal;
  config_.data = fast;

  handle_.Init.NominalPrescaler = nominal.prescaler;
  handle_.Init.NominalSyncJumpWidth = nominal.sync_jump_width;
  handle_.Init.NominalTimeSeg1 = nominal.time_seg1;
  handle_.Init.NominalTimeSeg2 = nominal.time_seg2;
  handle_.Init.DataPrescaler = fast.prescaler;
  handle_.Init.DataSyncJumpWidth = fast.sync_jump_width;
  handle_.Init.DataTimeSeg1 = fast.time_seg1;
  handle_.Init.DataTimeSeg2 = fast.time_seg2;

  handle_.Init.StdFiltersNbr = 0;
  handle_.Init.ExtFiltersNbr = 0;
  if (options_.filters.begin != nullptr && options_.filters.end != nullptr) {
    handle_.Init.StdFiltersNbr = static_cast<uint32_t>(std::count_if(
        options_.filters.begin, options_.filters.end, [](const Filter& f) {
          return f.action != FilterAction::kDisable &&
                 f.type == FilterType::kStandard;
        }));
    handle_.Init.ExtFiltersNbr = static_cast<uint32_t>(std::count_if(
        options_.filters.begin, options_.filters.end, [](const Filter& f) {
          return f.action != FilterAction::kDisable &&
                 f.type == FilterType::kExtended;
        }));
  }
  handle_.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&handle_) != HAL_OK) {
    Halt();
  }

  if (options_.filters.begin != nullptr && options_.filters.end != nullptr) {
    int standard_index = 0;
    int extended_index = 0;
    std::for_each(
        options_.filters.begin, options_.filters.end,
        [&](const Filter& filter) {
          if (filter.action == FilterAction::kDisable) {
            return;
          }

          FDCAN_FilterTypeDef cfg = {};
          cfg.IdType = (filter.type == FilterType::kStandard)
                           ? FDCAN_STANDARD_ID
                           : FDCAN_EXTENDED_ID;
          cfg.FilterIndex = (filter.type == FilterType::kStandard)
                                ? standard_index++
                                : extended_index++;
          switch (filter.mode) {
            case FilterMode::kRange:
              cfg.FilterType = FDCAN_FILTER_RANGE;
              break;
            case FilterMode::kDual:
              cfg.FilterType = FDCAN_FILTER_DUAL;
              break;
            case FilterMode::kMask:
              cfg.FilterType = FDCAN_FILTER_MASK;
              break;
          }
          switch (filter.action) {
            case FilterAction::kDisable:
            case FilterAction::kReject:
              cfg.FilterConfig = FDCAN_FILTER_REJECT;
              break;
            case FilterAction::kAccept:
              cfg.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
              break;
          }
          const uint32_t mask =
              (filter.type == FilterType::kStandard) ? 0x7FFu : 0x1FFFFFFFu;
          cfg.FilterID1 = filter.id1 & mask;
          cfg.FilterID2 = filter.id2 & mask;

          if (HAL_FDCAN_ConfigFilter(&handle_, &cfg) != HAL_OK) {
            Halt();
          }
        });
  }

  auto map_filter_action = [](FilterAction value) {
    switch (value) {
      case FilterAction::kDisable:
      case FilterAction::kAccept:
        return FDCAN_ACCEPT_IN_RX_FIFO0;
      case FilterAction::kReject:
        return FDCAN_REJECT;
    }
    Halt();
  };
  auto map_remote_action = [](FilterAction value) {
    switch (value) {
      case FilterAction::kDisable:
      case FilterAction::kAccept:
        return FDCAN_FILTER_REMOTE;
      case FilterAction::kReject:
        return FDCAN_REJECT_REMOTE;
    }
    Halt();
  };

  if (HAL_FDCAN_ConfigGlobalFilter(
          &handle_,
          map_filter_action(options_.filters.global_std_action),
          map_filter_action(options_.filters.global_ext_action),
          map_remote_action(options_.filters.global_remote_std_action),
          map_remote_action(options_.filters.global_remote_ext_action)) !=
      HAL_OK) {
    Halt();
  }

  if (options_.delay_compensation) {
    if (HAL_FDCAN_ConfigTxDelayCompensation(
            &handle_, options_.tdc_offset, options_.tdc_filter) != HAL_OK) {
      Halt();
    }
    if (HAL_FDCAN_EnableTxDelayCompensation(&handle_) != HAL_OK) {
      Halt();
    }
  } else {
    if (HAL_FDCAN_DisableTxDelayCompensation(&handle_) != HAL_OK) {
      Halt();
    }
  }

  if (HAL_FDCAN_Start(&handle_) != HAL_OK) {
    Halt();
  }
}

bool FDCan::Send(uint32_t dest_id, std::string_view data) {
  return Send(dest_id, data, SendOptions());
}

bool FDCan::Send(uint32_t dest_id,
                 std::string_view data,
                 const SendOptions& send_options) {
  if (last_tx_request_) {
    HAL_FDCAN_AbortTxRequest(&handle_, last_tx_request_);
  }

  FDCAN_TxHeaderTypeDef tx_header = {};
  tx_header.Identifier = dest_id;
  tx_header.IdType =
      ApplyOverride(dest_id >= 2048u, send_options.extended_id)
          ? FDCAN_EXTENDED_ID
          : FDCAN_STANDARD_ID;
  tx_header.TxFrameType =
      ApplyOverride(options_.remote_frame, send_options.remote_frame)
          ? FDCAN_REMOTE_FRAME
          : FDCAN_DATA_FRAME;
  tx_header.DataLength = RoundUpDlc(data.size());
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch =
      ApplyOverride(options_.bitrate_switch, send_options.bitrate_switch)
          ? FDCAN_BRS_ON
          : FDCAN_BRS_OFF;
  tx_header.FDFormat =
      ApplyOverride(options_.fdcan_frame, send_options.fdcan_frame)
          ? FDCAN_FD_CAN
          : FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  uint8_t payload[64] = {};
  const size_t copy_len = std::min(data.size(), sizeof(payload));
  if (copy_len > 0) {
    std::memcpy(payload, data.data(), copy_len);
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&handle_, &tx_header, payload) != HAL_OK) {
    return false;
  }
  last_tx_request_ = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&handle_);
  return true;
}

bool FDCan::Poll(FDCAN_RxHeaderTypeDef* header,
                 uint8_t* data,
                 size_t max_len,
                 size_t* out_len) {
  if (header == nullptr || data == nullptr || out_len == nullptr) {
    return false;
  }

  uint8_t scratch[64] = {};
  if (HAL_FDCAN_GetRxMessage(&handle_, FDCAN_RX_FIFO0, header, scratch) !=
      HAL_OK) {
    return false;
  }

  const int payload_len = ParseDlc(header->DataLength);
  const size_t copy_len =
      std::min(static_cast<size_t>(payload_len), max_len);
  if (copy_len > 0) {
    std::memcpy(data, scratch, copy_len);
  }
  *out_len = copy_len;
  return true;
}

void FDCan::RecoverBusOff() {
  handle_.Instance->CCCR &= ~FDCAN_CCCR_INIT;
}

FDCAN_ProtocolStatusTypeDef FDCan::status() {
  HAL_FDCAN_GetProtocolStatus(&handle_, &status_result_);
  return status_result_;
}

int FDCan::ParseDlc(uint32_t dlc_code) {
  switch (dlc_code) {
    case FDCAN_DLC_BYTES_0: return 0;
    case FDCAN_DLC_BYTES_1: return 1;
    case FDCAN_DLC_BYTES_2: return 2;
    case FDCAN_DLC_BYTES_3: return 3;
    case FDCAN_DLC_BYTES_4: return 4;
    case FDCAN_DLC_BYTES_5: return 5;
    case FDCAN_DLC_BYTES_6: return 6;
    case FDCAN_DLC_BYTES_7: return 7;
    case FDCAN_DLC_BYTES_8: return 8;
    case FDCAN_DLC_BYTES_12: return 12;
    case FDCAN_DLC_BYTES_16: return 16;
    case FDCAN_DLC_BYTES_20: return 20;
    case FDCAN_DLC_BYTES_24: return 24;
    case FDCAN_DLC_BYTES_32: return 32;
    case FDCAN_DLC_BYTES_48: return 48;
    case FDCAN_DLC_BYTES_64: return 64;
    default: return 0;
  }
}

}  // namespace hal

// Clock enable is done in FDCan::Init before HAL_FDCAN_Init; keep the weak
// MSP hook so HAL's call chain remains valid without extra NVIC setup.
extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan) {
  (void)hfdcan;
  __HAL_RCC_FDCAN_CLK_ENABLE();
}
