// Production-grade CAN-FD bootloader for xtellar (STM32G474)
// Ported from moteus/fw/can_bootloader.cc (mjbots, Apache 2.0)
//
// Protocol: mjlib multiplex text protocol over FDCAN (channel 1)
//   Commands: echo, unlock, lock, w <addr> <hex>, r <addr> <size>, reset
//
// Architecture:
//   - Bare-metal, no OS dependencies
//   - FDCAN2 on PB5/PB6 (AF9), 1 Mbps nominal @ 16 MHz PCLK1
//   - Bus-off recovery
//   - FlashWriter with shadow-word buffering + sector tracking
//   - Dual-bank flash aware
//   - Bootloader area write-protected (0x08000000-0x08010000, incl. vectors)
//   - ISRs disabled, polling-only design
//
// Memory layout:
//   0x08000000 - 0x0800BFFF: ISR vector table (vectors.S)
//   0x0800C000 - 0x0800FFFF: Bootloader code (.boot_entry + .text + .rodata)
//   0x08010000 - 0x0807FFFF: Application
//   0x20000000:              Boot magic value (0xB00710AD), preserved until 'reset' command

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string_view>

#include "stm32g4xx.h"

#include "boot_mjlib.h"
#include "stm32g4xx_fdcan_typedefs.h"

// ============================================================================
// Multiplex format constants (from mjlib/multiplex/format.h)
// ============================================================================
namespace mjlib {
namespace multiplex {

struct Format {
  enum class Subframe : uint8_t {
    kWriteBase = 0x00,
    kWriteInt8 = 0x00,  kWriteInt16 = 0x04,  kWriteInt32 = 0x08,  kWriteFloat = 0x0c,
    kReadBase = 0x10,
    kReadInt8 = 0x10,   kReadInt16 = 0x14,   kReadInt32 = 0x18,   kReadFloat = 0x1c,
    kReplyBase = 0x20,
    kReplyInt8 = 0x20,  kReplyInt16 = 0x24,  kReplyInt32 = 0x28,  kReplyFloat = 0x2c,
    kWriteError = 0x30, kReadError = 0x31,
    kClientToServer = 0x40,    kServerToClient = 0x41,
    kClientPollServer = 0x42,  kServerToClientFlow = 0x43,
    kClientPollServerFlow = 0x44,
    kNop = 0x50,
  };
};

// Varuint encode (bounds-checked: never writes past the buffer)
inline void WriteVaruint(base::BufferWriteStream& ostr, uint32_t value) {
  do {
    if (ostr.remaining() <= 0) { return; }
    uint8_t this_byte = value & 0x7f;
    value >>= 7;
    this_byte |= value ? 0x80 : 0x00;
    *reinterpret_cast<uint8_t*>(ostr.position()) = this_byte;
    ostr.skip(1);
  } while (value);
}

// Varuint decode (buffer-optimized)
inline std::optional<uint32_t> ReadVaruint(base::BufferReadStream& istr) {
  uint32_t result = 0;
  const uint8_t* position = reinterpret_cast<const uint8_t*>(istr.position());
  auto remaining = istr.remaining();

  int pos = 0;
  int i = 0;
  for (; i < 5; i++) {
    if (remaining == 0) { istr.fast_ignore(i); return {}; }
    remaining--;
    const auto this_byte = *position;
    position++;
    result |= (this_byte & 0x7f) << pos;
    pos += 7;
    if ((this_byte & 0x80) == 0) { istr.fast_ignore(i + 1); return result; }
  }
  istr.fast_ignore(i);
  return std::numeric_limits<uint32_t>::max();
}

// Write scalar to buffer
template <typename T>
inline void WriteScalar(base::BufferWriteStream& ostr, T value) {
  std::memcpy(ostr.position(), &value, sizeof(T));
  ostr.skip(sizeof(T));
}

}  // namespace multiplex
}  // namespace mjlib

// ============================================================================
// Bootloader constants
// ============================================================================
static constexpr uint32_t kAppStart       = 0x08010000;
static constexpr uint32_t kFlashPageSize  = 0x800;          // 2KB per page
// The application linker scripts reserve the first 512 bytes of SRAM.  Keep
// the reset-persistent handoff word there so neither image can allocate it as
// data, heap, or stack.
static constexpr uint32_t kBootMagicAddr  = 0x20000000;
static constexpr uint32_t kBootMagicValue = 0xB00710AD;
static constexpr uint32_t kBootloaderStart = 0x0800C000;
static constexpr uint32_t kBootloaderEnd   = 0x08010000;
static constexpr uint32_t kFlashEnd        = 0x08080000;

static constexpr uint8_t kDefaultSourceId = 1;
static constexpr uint8_t kTunnelChannel   = 1;

// ============================================================================
// Utility helpers
// ============================================================================
template <typename T>
inline uint32_t u32(T v) { return static_cast<uint32_t>(v); }

inline void uint8_hex(uint8_t value, char* buffer) {
  constexpr char digits[] = "0123456789ABCDEF";
  buffer[0] = digits[value >> 4];
  buffer[1] = digits[value & 0x0f];
  buffer[2] = 0;
}

inline void uint32_hex(uint32_t value, char* buffer) {
  for (int i = 0; i < 4; i++) {
    uint8_hex((value >> ((3 - i) * 8)) & 0xFF, &buffer[i * 2]);
  }
}

inline std::optional<uint32_t> hex_to_i(const std::string_view& str) {
  if (str.empty() || str.size() > 8) { return {}; }
  uint32_t result = 0;
  for (char c : str) {
    result <<= 4;
    if (c >= '0' && c <= '9')       result |= (c - '0');
    else if (c >= 'a' && c <= 'f')  result |= (c - 'a' + 0x0a);
    else if (c >= 'A' && c <= 'F')  result |= (c - 'A' + 0x0a);
    else { return {}; }
  }
  return result;
}

// Overflow-safe [base, end) range check.
inline bool addr_in_range(uint32_t addr, uint32_t size, uint32_t base, uint32_t end) {
  return addr >= base && addr <= end && size <= end - addr;
}

// ============================================================================
// LED (PB15, active-low on moteus hardware)
// ============================================================================
static void led_on()  { GPIOB->BSRR = 0x80000000; }
static void led_off() { GPIOB->BSRR = 0x00008000; }

// ============================================================================
// Clock configuration: use HSI @ 16MHz directly (no PLL).
// This is the robust bootloader approach — HSI is the reset default,
// requires no configuration, and works at 0 flash wait states.
// FDCAN timing is adjusted for 16MHz PCLK1.
// ============================================================================
static constexpr uint32_t kSystemClockHz = 16000000;
static constexpr uint32_t kPclk1Hz       = 16000000;

static void clock_init() {
    // HSI is already enabled and selected after reset.
    // Just ensure it's stable and set SystemCoreClock for the DWT timer.
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}
    SystemCoreClock = kSystemClockHz;

    // FDCAN kernel clock: reset default is HSE (FDCANSEL=00), but HSE is
    // never enabled here, which would leave the FDCAN core without a clock.
    // Select PCLK1 (16 MHz) instead.
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_FDCANSEL_Msk)
               | (2UL << RCC_CCIPR_FDCANSEL_Pos);

    // APB1 = HCLK = SYSCLK = 16MHz (reset defaults, no change needed)
    // Flash: 0 wait states at 16MHz (reset default)
}


// ============================================================================
// Millisecond timer using DWT cycle counter (no peripheral needed)
// ============================================================================
static void dwt_init() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

// ============================================================================
// FlashWriter — production-grade with shadow-word buffering and sector tracking
// ============================================================================
class FlashWriter {
 public:
  bool locked() const { return locked_; }

  void Unlock() {
    for (auto& v : sectors_erased_) { v = false; }
    // Reset shadow-word state so a stale partial word from a previous
    // session is never flushed into flash.
    shadow_start_ = 0;
    shadow_ = ~0ull;
    shadow_bits_ = 0;

    __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
    __HAL_FLASH_DATA_CACHE_DISABLE();

    if (!locked_) { return; }

    FLASH->SR |= (FLASH_SR_OPTVERR | FLASH_SR_RDERR | FLASH_SR_FASTERR |
                  FLASH_SR_MISERR | FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                  FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_PROGERR |
                  FLASH_SR_OPERR | FLASH_SR_EOP);
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    locked_ = false;
  }

  uint32_t Lock() {
    if (locked_) { return 0; }
    if (shadow_bits_) {
      const auto err = FlushWord();
      if (err) { return err; }
    }
    FLASH->CR |= FLASH_CR_LOCK;
    locked_ = true;
    return 0;
  }

  uint32_t ProgramByte(uint32_t intaddr, uint8_t value) {
    const uint32_t this_shadow = intaddr & ~(0x7);
    const uint32_t offset = intaddr & 0x7;

    if (this_shadow != shadow_start_ && shadow_start_ != 0) {
      const auto err = FlushWord();
      if (err) { return err; }
    }

    shadow_start_ = this_shadow;

    uint64_t mask = (0xffull << (offset * 8));
    shadow_ = (shadow_ & ~mask) | (static_cast<uint64_t>(value) << (offset * 8));
    shadow_bits_ |= mask;

    if (shadow_bits_ == 0xffffffffffffffffull) {
      const auto err = FlushWord();
      if (err) { return err; }
    }
    return 0;
  }

 private:
  uint32_t FlushWord() {
    const auto err = MaybeEraseSector(shadow_start_);
    if (err) { return err; }

    if (*reinterpret_cast<uint32_t*>(shadow_start_) ==
            static_cast<uint32_t>(shadow_ & 0xffffffff) &&
        *reinterpret_cast<uint32_t*>(shadow_start_ + 4u) ==
            static_cast<uint32_t>(shadow_ >> 32u)) {
      // Already matches — nothing to write
    } else {
      FLASH->CR |= FLASH_CR_PG;
      *reinterpret_cast<uint32_t*>(shadow_start_) =
          static_cast<uint32_t>(shadow_ & 0xffffffff);
      __ISB();
      *reinterpret_cast<uint32_t*>(shadow_start_ + 4u) =
          static_cast<uint32_t>(shadow_ >> 32u);
      const auto result = Wait();
      FLASH->CR &= ~FLASH_CR_PG;
      if (result) { return result; }
    }

    shadow_start_ = 0;
    shadow_ = ~0;
    shadow_bits_ = 0;
    return 0;
  }

  uint32_t MaybeEraseSector(uint32_t address) {
    const int bank = (address < 0x08040000) ? 1 : 2;
    const uint32_t bank_start = (bank == 1) ? 0x08000000 : 0x08040000;
    const int sector = (address - bank_start) / 2048;
    const int sector_index = (bank - 1) * 128 + sector;

    if (!sectors_erased_[sector_index]) {
      const auto err = EraseSector(bank, sector);
      if (err) { return err; }
      sectors_erased_[sector_index] = true;
    }
    return 0;
  }

  uint32_t EraseSector(int bank, int sector) {
    if (bank == 1) {
      FLASH->CR &= ~FLASH_CR_BKER;
    } else {
      FLASH->CR |= FLASH_CR_BKER;
    }
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PNB_Msk) | (sector << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_PER;
    FLASH->CR |= FLASH_CR_STRT;
    const auto result = Wait();
    FLASH->CR &= ~FLASH_CR_PER;
    return result;
  }

  uint32_t Wait() {
    // Bounded wait: max page erase is ~22 ms; at 16 MHz this loop count
    // corresponds to well over 100 ms.
    uint32_t timeout = 8000000;
    while (FLASH->SR & FLASH_FLAG_BSY) {
      if (--timeout == 0) { return 0x80000001u; }  // flash controller timeout
    }
    const uint32_t error = (FLASH->SR & FLASH_FLAG_SR_ERRORS);
    if (error != 0u) { FLASH->SR |= error; return error; }
    if (FLASH->SR & FLASH_FLAG_EOP) { FLASH->SR |= FLASH_FLAG_EOP; }
    return 0;
  }

  bool locked_ = true;
  uint32_t shadow_start_ = 0;
  uint64_t shadow_ = ~0ull;  // erased flash is 0xFF — padding must be 1s
  uint64_t shadow_bits_ = 0;
  bool sectors_erased_[256] = {};
};

// ============================================================================
// DLC helper — maps DLC codes to byte sizes
// ============================================================================
static constexpr int kDlcToSize[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

static constexpr int RoundUpDlc(int size) {
  for (int dlc = 0; ; dlc++) {
    if (size <= kDlcToSize[dlc]) return dlc;
  }
  return 15;
}

// ============================================================================
// FDCAN driver — FDCAN2 on PB5/PB6, bus-off recovery
// ============================================================================
struct CanFrame {
  uint32_t identifier = 0;
  uint32_t dlc = 0;
  uint32_t bit_rate_switch = 0;
  uint32_t fdformat = 0;
  uint32_t size = 0;
  uint8_t data[64] = {};
};

class FdcanDriver {
 public:
  explicit FdcanDriver(FDCAN_GlobalTypeDef* fdcan) : fdcan_(fdcan) {
    // Message RAM is shared between the three FDCAN instances.
    // RXF0C/TXBC must be programmed with the *RAM-relative* byte offset,
    // while the CPU accesses elements through absolute addresses.
    uint32_t ram_offset = 0;
    if (fdcan == FDCAN2) ram_offset = SRAMCAN_SIZE;
    if (fdcan == FDCAN3) ram_offset = SRAMCAN_SIZE * 2U;
    rx_fifo0_offset_ = ram_offset + SRAMCAN_RF0SA;
    tx_fifoq_offset_ = ram_offset + SRAMCAN_TFQSA;
    rx_fifo0_addr_ = SRAMCAN_BASE + rx_fifo0_offset_;
    tx_fifoq_addr_ = SRAMCAN_BASE + tx_fifoq_offset_;
  }

  void Init() {
    // Enable FDCAN clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;

    GPIOB->MODER  = (GPIOB->MODER & ~(GPIO_MODER_MODE5_Msk | GPIO_MODER_MODE6_Msk))
                  | (2 << GPIO_MODER_MODE5_Pos) | (2 << GPIO_MODER_MODE6_Pos);
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFF << 20)) | (9 << 20) | (9 << 24);
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR &
                      ~(GPIO_OSPEEDR_OSPEED5_Msk | GPIO_OSPEEDR_OSPEED6_Msk))
                   | (3UL << GPIO_OSPEEDR_OSPEED5_Pos)
                   | (3UL << GPIO_OSPEEDR_OSPEED6_Pos);

    // Reset FDCAN
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_FDCANRST;
    for (volatile int i = 0; i < 100; i++) { __NOP(); }
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_FDCANRST;
    for (volatile int i = 0; i < 100; i++) { __NOP(); }

    // After FDCAN reset, INIT=1. Do NOT clear it.
    fdcan_->CCCR |= FDCAN_CCCR_CCE;

    // Nominal timing: 1 Mbps @ 16MHz PCLK1
    // 16 tq per bit: 1 (sync) + 11 (seg1) + 4 (seg2), sample point 75%.
    fdcan_->NBTP = (3 << FDCAN_NBTP_NSJW_Pos)
                 | (3 << FDCAN_NBTP_NTSEG2_Pos)
                 | (10 << FDCAN_NBTP_NTSEG1_Pos)
                 | (0 << FDCAN_NBTP_NBRP_Pos);

    // Data timing: 2 Mbps @ 16MHz PCLK1
    // 8 tq per bit: 1 (sync) + 5 (seg1) + 2 (seg2), sample point 75%.
    fdcan_->DBTP = (1 << FDCAN_DBTP_DSJW_Pos)
                 | (1 << FDCAN_DBTP_DTSEG2_Pos)
                 | (4 << FDCAN_DBTP_DTSEG1_Pos)
                 | (0 << FDCAN_DBTP_DBRP_Pos);

    // Enable FD operation, disable auto-retransmission, disable protocol exception
    fdcan_->CCCR |= FDCAN_CCCR_FDOE | FDCAN_CCCR_DAR | FDCAN_CCCR_PXHD;

    // FIFO addresses and element counts are fixed by STM32G4's message RAM
    // layout.  The STM32 HAL likewise accesses SRAMCAN_BASE directly rather
    // than programming generic Bosch M_CAN RXF0C/TXBC address fields.

    // Global filter: accept all into RX FIFO 0
    fdcan_->RXGFC = (0 << FDCAN_RXGFC_ANFS_Pos)
                   | (0 << FDCAN_RXGFC_ANFE_Pos);

    // Leave config mode, leave init mode
    fdcan_->CCCR &= ~FDCAN_CCCR_CCE;
    for (volatile int i = 0; i < 100000; i++) { __NOP(); }
    fdcan_->CCCR &= ~FDCAN_CCCR_INIT;
    for (volatile int i = 0; i < 100000; i++) { __NOP(); }
  }

  bool Poll(CanFrame& frame) {
    if (fdcan_->PSR & FDCAN_PSR_BO) {
      // Bus-off recovery
      fdcan_->CCCR &= ~FDCAN_CCCR_INIT;
      return false;
    }

    if ((fdcan_->RXF0S & FDCAN_RXF0S_F0FL) == 0) return false;  // no messages

    const auto get_index = (fdcan_->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
    auto* rx_address = reinterpret_cast<uint32_t*>(
        rx_fifo0_addr_ + get_index * SRAMCAN_RF0_SIZE);

    uint32_t w1 = rx_address[0];
    frame.identifier = (w1 & FDCAN_ELEMENT_MASK_STDID) >> 18;
    if (w1 & FDCAN_ELEMENT_MASK_XTD) {
      frame.identifier = w1 & FDCAN_ELEMENT_MASK_EXTID;
    }

    uint32_t w2 = rx_address[1];
    frame.dlc = (w2 & FDCAN_ELEMENT_MASK_DLC) >> 16;
    frame.bit_rate_switch = w2 & FDCAN_ELEMENT_MASK_BRS;
    frame.fdformat = w2 & FDCAN_ELEMENT_MASK_FDF;
    frame.size = kDlcToSize[frame.dlc];

    auto* pdata = reinterpret_cast<uint8_t*>(&rx_address[2]);
    for (uint32_t i = 0; i < frame.size && i < 64; i++) {
      frame.data[i] = pdata[i];
    }

    fdcan_->RXF0A = get_index;
    return true;
  }

  bool Send(uint32_t identifier, const std::string_view& data, bool brs) {
    // Bounded wait for a free TX element (returns false on a wedged bus
    // instead of hanging the bootloader forever).
    uint32_t timeout = 1000000;
    while (fdcan_->TXFQS & FDCAN_TXFQS_TFQF) {
      if (--timeout == 0) { return false; }
    }

    auto put_index = (fdcan_->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
    uint32_t dlc = RoundUpDlc(data.size());

    uint32_t element_w1 = (identifier >= 2048)
        ? (FDCAN_ESI_ACTIVE | FDCAN_EXTENDED_ID | FDCAN_DATA_FRAME | identifier)
        : (FDCAN_ESI_ACTIVE | FDCAN_STANDARD_ID | FDCAN_DATA_FRAME | (identifier << 18));
    uint32_t element_w2 = (0u << 24)             // no message marker
                        | FDCAN_NO_TX_EVENTS
                        | FDCAN_FD_CAN
                        | (brs ? FDCAN_BRS_ON : 0)
                        | (dlc << 16);

    auto* tx_address = reinterpret_cast<uint32_t*>(
        tx_fifoq_addr_ + put_index * SRAMCAN_TFQ_SIZE);

    *tx_address = element_w1;
    tx_address++;
    *tx_address = element_w2;
    tx_address++;

    size_t rounded_size = kDlcToSize[dlc];
    for (size_t i = 0; i < rounded_size; i += 4) {
      auto get_byte = [&](int offset) -> uint8_t {
        auto pos = i + offset;
        return pos < data.size() ? static_cast<uint8_t>(data[pos]) : 0;
      };
      *tx_address = (get_byte(3) << 24) | (get_byte(2) << 16)
                  | (get_byte(1) << 8)  |  get_byte(0);
      tx_address++;
    }

    fdcan_->TXBAR = (1 << put_index);
    return true;
  }

 private:
  FDCAN_GlobalTypeDef* const fdcan_;
  uint32_t rx_fifo0_offset_ = 0;  // RAM-relative, programmed into RXF0C
  uint32_t tx_fifoq_offset_ = 0;  // RAM-relative, programmed into TXBC
  uint32_t rx_fifo0_addr_ = 0;    // absolute, used by the CPU in Poll()
  uint32_t tx_fifoq_addr_ = 0;    // absolute, used by the CPU in Send()
};

// ============================================================================
// BootloaderServer — multiplex text protocol over FDCAN
// ============================================================================
template <typename T>
struct Buffer {
  T data[256] = {};
  size_t pos = 0;

  std::string_view view() const {
    return {reinterpret_cast<const char*>(data), pos};
  }

  size_t capacity() const { return sizeof(data) / sizeof(*data); }

  mjlib::base::BufferWriteStream writer() {
    return mjlib::base::BufferWriteStream(
        mjlib::base::string_span(&data[pos], capacity() - pos));
  }
};

class BootloaderServer {
 public:
  BootloaderServer(uint8_t id, FDCAN_GlobalTypeDef* fdcan)
      : id_(id), driver_(fdcan) {
    driver_.Init();

    auto writer = response_.writer();
    writer.write("multiplex bootloader protocol 1 ");
    // Include compile-time marker
    writer.write(__DATE__);
    writer.write(" ");
    writer.write(__TIME__);
    writer.write("\r\n");
    response_.pos = writer.offset();
  }

  void Run() {
    while (true) {
      ReadCommand();
      RunCommand();
    }
  }

 private:
  // Processes a single received frame (blocking until one arrives).
  // Returns after each frame so RunCommand() gets a chance to execute
  // even when the host streams data without the query bit set.
  void ReadCommand() {
    CanFrame frame;
    while (!driver_.Poll(frame)) {}  // blocking poll

    // Check if addressed to us
    uint8_t source_id = (frame.identifier >> 8) & 0xFF;
    uint8_t dest_id = (frame.identifier & 0xFF);
    if (dest_id != id_ && dest_id != 0x7F) return;  // 0x7F = broadcast

    // Parse multiplex subframe
    mjlib::base::BufferReadStream buffer_stream{
        std::string_view(reinterpret_cast<const char*>(frame.data), frame.size)};

    auto maybe_subframe_id = mjlib::multiplex::ReadVaruint(buffer_stream);
    if (!maybe_subframe_id) return;

    using SF = mjlib::multiplex::Format::Subframe;
    if (*maybe_subframe_id != u32(SF::kClientToServer) &&
        *maybe_subframe_id != u32(SF::kClientPollServer)) return;

    const bool poll_only = (*maybe_subframe_id == u32(SF::kClientPollServer));
    const bool query = (source_id & 0x80) != 0;

    auto maybe_channel = mjlib::multiplex::ReadVaruint(buffer_stream);
    if (!maybe_channel || *maybe_channel != kTunnelChannel) return;

    auto maybe_bytes = mjlib::multiplex::ReadVaruint(buffer_stream);
    if (!maybe_bytes) return;

    if (!poll_only && *maybe_bytes > 0) {
      size_t bytes = *maybe_bytes;
      if (static_cast<size_t>(buffer_stream.remaining()) < bytes) return;
      if (command_.pos + bytes > command_.capacity()) {
        command_.pos = 0;  // Overflow — discard
        return;
      }
      std::memcpy(&command_.data[command_.pos],
                  buffer_stream.position(), bytes);
      buffer_stream.fast_ignore(bytes);
      command_.pos += bytes;
    }

    if (query) {
      WriteResponse(source_id & 0x7F, poll_only ? *maybe_bytes : -1, frame);
    }
  }

  void WriteResponse(uint8_t id, int max_bytes, const CanFrame& source_frame) {
    out_frame_.pos = 0;
    auto buffer_stream = out_frame_.writer();

    mjlib::multiplex::WriteVaruint(buffer_stream,
        u32(mjlib::multiplex::Format::Subframe::kServerToClient));
    mjlib::multiplex::WriteVaruint(buffer_stream, kTunnelChannel);

    constexpr size_t kMaxCanPayload = 64 - 3;
    size_t bytes_to_write = std::min<size_t>(
        kMaxCanPayload,
        max_bytes >= 0
            ? std::min<size_t>(static_cast<size_t>(max_bytes), response_.pos)
            : response_.pos);

    mjlib::multiplex::WriteVaruint(buffer_stream, bytes_to_write);
    buffer_stream.write(response_.view().substr(0, bytes_to_write));
    out_frame_.pos = buffer_stream.offset();

    // Remove sent bytes from response buffer
    std::memmove(&response_.data[0], &response_.data[bytes_to_write],
                 response_.pos - bytes_to_write);
    response_.pos -= bytes_to_write;

    // Send CAN frame (drop on TX timeout — the host will retry)
    (void)driver_.Send(((id_ << 8) | id), out_frame_.view(),
                       source_frame.bit_rate_switch != 0);
  }

  void RunCommand() {
    auto writer = response_.writer();

    auto command_end = command_.view().find_first_of("\r\n");
    if (command_end == std::string_view::npos) return;  // no complete line yet

    mjlib::base::Tokenizer tokenizer(command_.view(), " \r\n");
    const auto next = tokenizer.next();
    if (next.empty()) {
      // Empty line, ignore
    } else if (next == "echo") {
      writer.write(tokenizer.remaining());
    } else if (next == "unlock") {
      flash_.Unlock();
      writer.write("OK\r\n");
    } else if (next == "lock") {
      if (flash_.Lock()) {
        writer.write("ERR error locking\r\n");
      } else {
        writer.write("OK\r\n");
      }
    } else if (next == "w") {
      const auto addr_str = tokenizer.next();
      const auto data_str = tokenizer.next();
      if (addr_str.empty() || data_str.empty()) {
        writer.write("ERR malformed write\r\n");
      } else {
        WriteFlash(addr_str, data_str, writer);
      }
    } else if (next == "r") {
      const auto addr_str = tokenizer.next();
      const auto size_str = tokenizer.next();
      if (addr_str.empty() || size_str.empty()) {
        writer.write("ERR malformed read\r\n");
      } else {
        ReadFlash(addr_str, size_str, writer);
      }
    } else if (next == "reset") {
      if (flash_.Lock()) {
        writer.write("ERR error locking\r\n");
      } else {
        // A 'reset' command ends the bootloader session: clear the boot
        // magic so the next reset boots the application. The magic is
        // intentionally NOT cleared at bootloader entry, so an interrupted
        // flash session re-enters the bootloader instead of jumping into
        // a half-programmed application.
        *reinterpret_cast<volatile uint32_t*>(kBootMagicAddr) = 0;
        NVIC_SystemReset();
      }
    } else {
      writer.write("ERR unknown command\r\n");
    }

    response_.pos += writer.offset();

    // Consume the processed command line
    const auto to_consume = command_end + 1;
    std::memmove(command_.data, command_.data + to_consume,
                 command_.capacity() - to_consume);
    command_.pos -= to_consume;
  }

  void ReadFlash(const std::string_view& addr_str,
                 const std::string_view& size_str,
                 mjlib::base::BufferWriteStream& writer) {
    char buf[10] = {};
    const auto maybe_addr = hex_to_i(addr_str);
    const auto maybe_size = hex_to_i(size_str);
    if (!maybe_addr || !maybe_size) {
      writer.write("ERR malformed hex\r\n");
      return;
    }
    const uint32_t addr = *maybe_addr;
    const uint32_t size = *maybe_size;
    if (size > 32) {
      writer.write("size too big\r\n");
      return;
    }
    // Only allow reading mapped memory: flash, SRAM, and CCM.
    // Arbitrary addresses can raise a bus fault and wedge the device.
    if (!addr_in_range(addr, size, 0x08000000, kFlashEnd) &&
        !addr_in_range(addr, size, 0x20000000, 0x20020000) &&
        !addr_in_range(addr, size, 0x10000000, 0x10008000)) {
      writer.write("ERR address not readable\r\n");
      return;
    }
    uint32_hex(addr, buf);
    writer.write(buf);
    writer.write(" ");
    for (uint32_t i = 0; i < size; i++) {
      const uint8_t* val = reinterpret_cast<const uint8_t*>(addr + i);
      uint8_hex(*val, buf);
      writer.write(buf);
    }
    writer.write("\r\n");
  }

  void WriteFlash(const std::string_view& addr_str,
                  const std::string_view& data_str,
                  mjlib::base::BufferWriteStream& writer) {
    if (data_str.size() % 2 != 0) {
      writer.write("odd data size\r\n");
      return;
    }
    const auto maybe_addr = hex_to_i(addr_str);
    if (!maybe_addr) {
      writer.write("ERR malformed hex\r\n");
      return;
    }
    const uint32_t addr = *maybe_addr;
    const uint32_t bytes = data_str.size() / 2;
    for (uint32_t i = 0; i < bytes; i++) {
      const auto maybe_byte =
          hex_to_i(std::string_view(data_str.data() + i * 2, 2));
      if (!maybe_byte) {
        writer.write("ERR malformed hex\r\n");
        return;
      }
      if (!WriteByte(addr + i, static_cast<uint8_t>(*maybe_byte), writer)) return;
    }
    writer.write("OK\r\n");
  }

  bool WriteByte(uint32_t address, uint8_t byte,
                 mjlib::base::BufferWriteStream& writer) {
    if (flash_.locked()) {
      writer.write("ERR flash is locked\r\n");
      return false;
    }
    if (address < 0x08000000 || address >= kFlashEnd) {
      writer.write("ERR address not in flash\r\n");
      return false;
    }
    // Protect everything below the application: the ISR vector table
    // (0x08000000-0x0800BFFF) AND the bootloader code (0x0800C000-...).
    // Erasing the vector page would brick the device until SWD recovery.
    if (address < kAppStart) {
      writer.write("ERR address not writable (bootloader region)\r\n");
      return false;
    }
    const auto err = flash_.ProgramByte(address, byte);
    if (err) {
      writer.write("ERR program error ");
      char buf[9] = {};
      uint32_hex(err, buf);
      writer.write(buf);
      writer.write("\r\n");
      return false;
    }
    return true;
  }

  const uint8_t id_;
  FdcanDriver driver_;
  FlashWriter flash_;
  Buffer<char> command_;
  Buffer<char> response_;
  Buffer<char> out_frame_;
};

// ============================================================================
// ISR stubs — polling-only design, all interrupts disabled
// ============================================================================
extern "C" {
// On any fault: blink a distinctive fast pattern (visible evidence of the
// fault) and reset. The previous behavior (while(1) with no watchdog)
// hung the board until the next power cycle.
static void fault_blink_reset() {
    for (int i = 0; i < 20; i++) {
        led_on();
        for (volatile uint32_t d = 0; d < 100000; d++) { __NOP(); }
        led_off();
        for (volatile uint32_t d = 0; d < 100000; d++) { __NOP(); }
    }
    NVIC_SystemReset();
    while (1) {}
}

void NMI_Handler(void)        { fault_blink_reset(); }
void HardFault_Handler(void)  { fault_blink_reset(); }
void MemManage_Handler(void)  { fault_blink_reset(); }
void BusFault_Handler(void)   { fault_blink_reset(); }
void UsageFault_Handler(void) { fault_blink_reset(); }
void SVC_Handler(void)        {}
void PendSV_Handler(void)     {}
void SysTick_Handler(void)    {}
void Default_Handler(void)    { fault_blink_reset(); }
}

namespace mjlib {
namespace base {
// Weak assertion handler — application may override
void __attribute__((weak)) assertion_failed(const char* expression, const char* filename, int line) {
    while (1) {}
}
}  // namespace base
}  // namespace mjlib

// ============================================================================
// Bootloader entry point (placed in .boot_entry at 0x0800C000)
// ============================================================================
// BSS/data symbols from linker script
extern "C" {
    extern uint8_t __bss_start__;
    extern uint8_t __bss_end__;
    extern char _sdata;
    extern char _edata;
    extern char _sidata;

static void jump_to_app(uint32_t addr) {
    uint32_t sp = *reinterpret_cast<uint32_t*>(addr);
    uint32_t pc = *reinterpret_cast<uint32_t*>(addr + 4);

    if (sp < 0x20000200 || sp > 0x20020000 || (sp & 0x7)) return;
    if (pc < kAppStart || pc >= kFlashEnd || !(pc & 1)) return;

    // --- Jump ---
    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    SCB->VTOR = addr;
    __set_CONTROL(0);
    __set_BASEPRI(0);
    __set_FAULTMASK(0);
    __DSB();
    __ISB();

    __set_MSP(sp);
    __enable_irq();
    __asm volatile ("bx %0\n" :: "r"(pc) : "memory");
}

static bool is_bootloader_requested() {
    return *reinterpret_cast<uint32_t*>(kBootMagicAddr) == kBootMagicValue;
}

__attribute__((section(".boot_entry")))
void BootloaderEntry() {
    // --- Manually initialize BSS and data (no C runtime) ---
    std::memset(&__bss_start__, 0, &__bss_end__ - &__bss_start__);
    char* dst = &_sdata;
    char* src = &_sidata;
    while (dst != &_edata) {
        *dst = *src;
        dst++;
        src++;
    }

    // --- Hardware init ---
    clock_init();
    dwt_init();

    // --- GPIO / LED ---
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE15_Msk)
                 | (1 << GPIO_MODER_MODE15_Pos);
    led_off();

    // --- Decide: application or bootloader ---
    if (!is_bootloader_requested()) {
        // Normal boot: 3 rapid blinks, then try the application
        for (int i = 0; i < 3; i++) {
            led_on();
            for (volatile uint32_t d = 0; d < 200000; d++) { __NOP(); }
            led_off();
            for (volatile uint32_t d = 0; d < 200000; d++) { __NOP(); }
        }

        jump_to_app(kAppStart);
        // If jump_to_app returns, the application vector table was invalid
        // (e.g., a failed/incomplete flash). Fall through to bootloader mode
        // so recovery is always possible over CAN.
    } else {
        // We were explicitly asked to enter the bootloader. The boot magic
        // is cleared by the 'reset' command after a successful flash session.
    }

    // --- Bootloader mode: 4 slow blinks (~2s total @16MHz) ---
    for (int i = 0; i < 4; i++) {
        led_on();
        for (volatile uint32_t d = 0; d < 400000; d++) { __NOP(); }
        led_off();
        for (volatile uint32_t d = 0; d < 400000; d++) { __NOP(); }
    }

    // Polling-only design: keep interrupts disabled
    __disable_irq();

    // CAN wiring on this board: PB5/PB6 (AF9) = FDCAN2 on STM32G474.
    // The old PXHD-based auto-detect could never work because CCCR (and
    // PXHD) is cleared by every reset, so it always fell back to FDCAN1
    // whose pins (PA11/PA12 or PB8/PB9) are not connected.
    BootloaderServer server(kDefaultSourceId, FDCAN2);
    server.Run();
    // Never returns
}

}  // extern "C"
