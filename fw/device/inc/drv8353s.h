#pragma once

#include <cstdint>

#include "HAL/millisecond_timer.h"
#include "drivers/DigitalIn.h"
#include "drivers/DigitalOut.h"
#include "PinNames.h"

namespace device
{

namespace drv8353s_registers
{

enum class Address : uint8_t
{
  FAULT_STATUS_1 = 0x00,
  FAULT_STATUS_2 = 0x01,
  DRIVER_CONTROL = 0x02,
  GATE_DRIVE_HS = 0x03,
  GATE_DRIVE_LS = 0x04,
  OCP_CONTROL = 0x05,
  CSA_CONTROL = 0x06,
  DRIVER_CONFIG = 0x07,
};

struct Spi
{
  static constexpr uint16_t READ = 1u << 15;
  static constexpr uint8_t ADDRESS_SHIFT = 11;
  static constexpr uint16_t ADDRESS_MASK = 0x0fu;
  static constexpr uint16_t DATA_MASK = 0x07ffu;
};

struct FaultStatus1
{
  static constexpr uint16_t FAULT = 1u << 10;
  static constexpr uint16_t VDS_OCP = 1u << 9;
  static constexpr uint16_t GDF = 1u << 8;
  static constexpr uint16_t UVLO = 1u << 7;
  static constexpr uint16_t OTSD = 1u << 6;
  static constexpr uint16_t VDS_PHASE_MASK = 0x003fu;
};

struct FaultStatus2
{
  static constexpr uint16_t SA_OC = 1u << 10;
  static constexpr uint16_t SB_OC = 1u << 9;
  static constexpr uint16_t SC_OC = 1u << 8;
  static constexpr uint16_t OTW = 1u << 7;
  static constexpr uint16_t CPUV = 1u << 6;
  static constexpr uint16_t VGS_PHASE_MASK = 0x003fu;
};

struct DriverControl
{
  static constexpr uint8_t OCP_ACT_SHIFT = 10;
  static constexpr uint8_t DIS_CPUV_SHIFT = 9;
  static constexpr uint8_t DIS_GDF_SHIFT = 8;
  static constexpr uint8_t OTW_REP_SHIFT = 7;
  static constexpr uint8_t PWM_MODE_SHIFT = 5;
  static constexpr uint8_t PWM_COM1_SHIFT = 4;
  static constexpr uint8_t PWM_DIR_SHIFT = 3;
};

struct GateDriveHs
{
  static constexpr uint8_t LOCK_SHIFT = 8;
  static constexpr uint16_t UNLOCK_CODE = 3;
  static constexpr uint8_t IDRIVEP_HS_SHIFT = 4;
  static constexpr uint8_t IDRIVEN_HS_SHIFT = 0;
};

struct GateDriveLs
{
  static constexpr uint8_t CBC_SHIFT = 10;
  static constexpr uint8_t TDRIVE_SHIFT = 8;
  static constexpr uint8_t IDRIVEP_LS_SHIFT = 4;
  static constexpr uint8_t IDRIVEN_LS_SHIFT = 0;
};

struct OcpControl
{
  static constexpr uint8_t TRETRY_SHIFT = 10;
  static constexpr uint8_t DEAD_TIME_SHIFT = 8;
  static constexpr uint8_t OCP_MODE_SHIFT = 6;
  static constexpr uint8_t OCP_DEG_SHIFT = 4;
  static constexpr uint8_t VDS_LVL_SHIFT = 0;
};

struct CsaControl
{
  static constexpr uint8_t CSA_FET_SHIFT = 10;
  static constexpr uint8_t VREF_DIV_SHIFT = 9;
  static constexpr uint8_t LS_REF_SHIFT = 8;
  static constexpr uint8_t CSA_GAIN_SHIFT = 6;
  static constexpr uint8_t DIS_SEN_SHIFT = 5;
  static constexpr uint8_t SEN_LVL_SHIFT = 0;
  static constexpr uint16_t CALIBRATION_MASK = 0x001cu;
};

struct DriverConfig
{
  static constexpr uint16_t CAL_MODE = 1u << 0;
};

}  // namespace drv8353s_registers

// Driver for the SPI variant of TI's DRV8353S three-phase gate driver.
// The RTAR suffix only describes the package and tape/reel delivery.
class Drv8353s
{
 public:
  enum class PwmMode : uint8_t
  {
    PWM_6X = 0,
    PWM_3X = 1,
    PWM_1X = 2,
    INDEPENDENT = 3,
  };

  enum class OcpMode : uint8_t
  {
    LATCHED_FAULT = 0,
    AUTOMATIC_RETRY = 1,
    REPORT_ONLY = 2,
    DISABLED = 3,
  };

  struct Options
  {
    PinName mosi = NC;
    PinName miso = NC;
    PinName sck = NC;
    PinName cs = NC;
    PinName enable = NC;
    PinName fault = NC;
    PinName hiz = NC;
    int spi_frequency_hz = 1000000;
  };

  struct Config
  {
    bool disable_charge_pump_uvlo = false;
    bool disable_gate_drive_fault = false;
    bool report_overtemperature_warning = false;
    PwmMode pwm_mode = PwmMode::PWM_3X;
    bool pwm_1x_asynchronous = false;
    bool pwm_1x_direction = false;

    uint16_t source_current_hs_ma = 100;
    uint16_t sink_current_hs_ma = 200;
    uint16_t source_current_ls_ma = 100;
    uint16_t sink_current_ls_ma = 200;
    bool cycle_by_cycle = true;
    uint16_t peak_drive_time_ns = 1000;

    bool retry_time_50us = false;
    uint16_t dead_time_ns = 100;
    OcpMode ocp_mode = OcpMode::LATCHED_FAULT;
    uint8_t ocp_deglitch_us = 4;
    uint16_t vds_threshold_mv = 700;

    bool csa_fet = false;
    bool vref_divide_by_2 = true;
    bool low_side_reference = false;
    uint8_t csa_gain = 20;
    bool disable_sense_ocp = false;
    uint16_t sense_threshold_mv = 500;
  };

  struct Status
  {
    uint16_t fault_status_1 = 0;
    uint16_t fault_status_2 = 0;
    bool fault_line = false;

    bool fault = false;
    bool vds_ocp = false;
    bool gate_drive_fault = false;
    bool undervoltage = false;
    bool overtemperature_shutdown = false;
    bool overtemperature_warning = false;
    bool charge_pump_undervoltage = false;
    bool sense_a_ocp = false;
    bool sense_b_ocp = false;
    bool sense_c_ocp = false;
    uint8_t vds_phase_faults = 0;
    uint8_t vgs_phase_faults = 0;
  };

  Drv8353s(hal::MillisecondTimer* timer, const Options& options);

  // Complete the x1 gate-driver startup sequence: keep the power stage in
  // high impedance, enable the IC, configure and verify registers, then
  // calibrate all three current-shunt amplifiers.
  bool Init(const Config& config);

  // EN_GATE must remain high for at least 1 ms before SPI access.
  void Enable();
  void Disable();
  bool enabled() const { return enabled_; }

  // Board-level HIZ control. Init always leaves the power stage off.
  void PowerOn();
  void PowerOff();
  bool power_on() const { return power_on_; }

  uint16_t ReadRegister(uint8_t address);
  void WriteRegister(uint8_t address, uint16_t value);

  // Programs registers 2..7 and reads them back. Bit N in the return value
  // indicates that register N did not verify.
  uint8_t Configure(const Config& config);
  Status ReadStatus();

  // Latest status snapshot (updated by ReadStatus / Init).
  const Status& last_status() const { return last_status_; }
  bool init_ok() const { return init_ok_; }

  // Telemetry channel name and text exporter for StatusRegistry.
  static constexpr const char* kTelemetryChannel = "drv8353s";
  static size_t TelemetryExport(void* context, char* out, size_t out_capacity);
  size_t FormatTelemetry(char* out, size_t out_capacity);

  // Runs the current-shunt amplifier offset calibration sequence.
  bool CalibrateCurrentSense();

  static float CsaSettlingTimeSeconds(uint8_t gain);

 private:
  uint16_t Transfer(uint16_t value);

  hal::MillisecondTimer* timer_ = nullptr;
  mbed::DigitalOut cs_;
  mbed::DigitalOut mosi_;
  mbed::DigitalIn miso_;
  mbed::DigitalOut sck_;
  mbed::DigitalOut enable_;
  mbed::DigitalOut hiz_;
  mbed::DigitalIn fault_;
  uint32_t spi_half_period_us_ = 1;
  bool enabled_ = false;
  bool power_on_ = false;
  bool init_ok_ = false;
  Status last_status_ = {};
};

}  // namespace device
