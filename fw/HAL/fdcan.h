// STM32 HAL-based FDCAN wrapper for xtellar (STM32G474).
// API shape follows moteus/fw/fdcan.h; implementation uses HAL_FDCAN_*.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "stm32g4xx_hal.h"

namespace hal {

class FDCan {
 public:
  enum class FilterAction {
    kDisable,
    kAccept,
    kReject,
  };

  enum class FilterMode {
    kRange,
    kDual,
    kMask,
  };

  enum class FilterType {
    kStandard,
    kExtended,
  };

  struct Filter {
    uint32_t id1 = 0;
    uint32_t id2 = 0;
    FilterMode mode = FilterMode::kRange;
    FilterAction action = FilterAction::kDisable;
    FilterType type = FilterType::kStandard;
  };

  // Negative fields mean "use auto-calculated timing".
  struct Rate {
    int prescaler = -1;
    int sync_jump_width = -1;
    int time_seg1 = -1;
    int time_seg2 = -1;
  };

  struct FilterConfig {
    FilterAction global_std_action = FilterAction::kAccept;
    FilterAction global_ext_action = FilterAction::kAccept;
    FilterAction global_remote_std_action = FilterAction::kAccept;
    FilterAction global_remote_ext_action = FilterAction::kAccept;

    const Filter* begin = nullptr;
    const Filter* end = nullptr;
  };

  struct Options {
    // Default matches moteus / xtellar wiring: FDCAN2 on PB5(RX)/PB6(TX).
    FDCAN_GlobalTypeDef* instance = FDCAN2;

    int slow_bitrate = 1000000;
    int fast_bitrate = 2000000;

    FilterConfig filters;

    bool automatic_retransmission = false;
    bool remote_frame = false;
    bool fdcan_frame = true;
    bool bitrate_switch = true;
    bool restricted_mode = false;
    bool bus_monitor = false;

    bool delay_compensation = false;
    uint32_t tdc_offset = 0;
    uint32_t tdc_filter = 0;

    Rate rate_override;
    Rate fdrate_override;

    Options() {}
  };

  enum class Override {
    kDefault,
    kRequire,
    kDisable,
  };

  struct SendOptions {
    Override bitrate_switch = Override::kDefault;
    Override fdcan_frame = Override::kDefault;
    Override remote_frame = Override::kDefault;
    Override extended_id = Override::kDefault;

    SendOptions() {}
  };

  explicit FDCan(const Options& options);

  void ConfigureFilters(const FilterConfig& filters);

  // Returns false if the TX FIFO is full or HAL rejects the frame.
  bool Send(uint32_t dest_id, std::string_view data);
  bool Send(uint32_t dest_id,
            std::string_view data,
            const SendOptions& send_options);

  // Returns true when a frame was copied into |data|.
  // |out_len| receives the decoded payload length (0..64).
  bool Poll(FDCAN_RxHeaderTypeDef* header,
            uint8_t* data,
            size_t max_len,
            size_t* out_len);

  void RecoverBusOff();

  FDCAN_ProtocolStatusTypeDef status();

  struct Config {
    int clock = 0;
    Rate nominal;
    Rate data;
  };

  Config config() const { return config_; }

  static int ParseDlc(uint32_t dlc_code);

  FDCAN_HandleTypeDef* handle() { return &handle_; }

 private:
  void Init();
  void ConfigurePins();

  Options options_;
  Config config_ = {};
  FDCAN_HandleTypeDef handle_ = {};
  FDCAN_ProtocolStatusTypeDef status_result_ = {};
  uint32_t last_tx_request_ = 0;
};

}  // namespace hal
