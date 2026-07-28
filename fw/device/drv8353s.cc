#include "drv8353s.h"

#include <cstddef>

#include "telemetry/text_format.h"

namespace device
{
namespace
{

namespace reg = drv8353s_registers;

constexpr uint8_t Address(reg::Address address)
{
  return static_cast<uint8_t>(address);
}

template <size_t N>
uint16_t MapChoice(const uint16_t (&table)[N], uint16_t value)
{
  for (uint16_t i = 0; i < N; ++i)
  {
    if (value <= table[i])
    {
      return i;
    }
  }
  return N - 1;
}

uint16_t Bit(bool value, unsigned position)
{
  return value ? static_cast<uint16_t>(1u << position) : 0;
}

// Datasheet discrete choices; index = register code written to the field.
// IDRIVEP_HS/LS: peak source current (mA)
constexpr uint16_t IDRIVEP_MA[] =
{
  50, 50, 100, 150, 300, 350, 400, 450,
  550, 600, 650, 700, 850, 900, 950, 1000,
};
// IDRIVEN_HS/LS: peak sink current (mA)
constexpr uint16_t IDRIVEN_MA[] =
{
  100, 100, 200, 300, 600, 700, 800, 900,
  1100, 1200, 1300, 1400, 1700, 1800, 1900, 2000,
};
// TDRIVE: peak current drive time (ns)
constexpr uint16_t TDRIVE_NS[] =
{
  500, 1000, 2000, 4000,
};
// DEAD_TIME (ns)
constexpr uint16_t DEAD_TIME_NS[] =
{
  50, 100, 200, 400,
};
// OCP_DEG: overcurrent deglitch (us)
constexpr uint16_t OCP_DEG_US[] =
{
  1, 2, 4, 8,
};
// VDS_LVL: VDS overcurrent threshold (mV)
constexpr uint16_t VDS_LVL_MV[] =
{
  60, 70, 80, 90, 100, 200, 300, 400,
  500, 600, 700, 800, 900, 1000, 1500, 2000,
};
// CSA_GAIN: current-shunt amplifier gain
constexpr uint16_t CSA_GAIN[] =
{
  5, 10, 20, 40,
};
// SEN_LVL: sense OCP threshold (mV)
constexpr uint16_t SEN_LVL_MV[] =
{
  250, 500, 750, 1000,
};

}  // namespace

Drv8353s::Drv8353s(hal::MillisecondTimer* timer, const Options& options)
    : timer_(timer),
      cs_(options.cs, 1),
      mosi_(options.mosi, 0),
      miso_(options.miso, PullUp),
      sck_(options.sck, 0),
      enable_(options.enable, 0),
      hiz_(options.hiz, 0),
      fault_(options.fault, PullUp)
{
  // x1 routes MOSI to PC13, which is not the hardware SPI3 MOSI pin.
  // Match the old firmware's GPIO SPI: 16-bit, MSB first, mode 1.
  if (options.spi_frequency_hz > 0)
  {
    spi_half_period_us_ = 500000u /
                          static_cast<uint32_t>(options.spi_frequency_hz);
    if (spi_half_period_us_ == 0)
    {
      spi_half_period_us_ = 1;
    }
  }
}

bool Drv8353s::Init(const Config& config)
{
  PowerOff();
  Disable();
  Enable();

  init_ok_ = false;
  if (Configure(config) != 0)
  {
    Disable();
    return false;
  }
  if (!CalibrateCurrentSense())
  {
    Disable();
    return false;
  }

  const Status status = ReadStatus();
  if (status.fault || status.fault_line)
  {
    Disable();
    return false;
  }
  init_ok_ = true;
  return true;
}

void Drv8353s::Enable()
{
  enable_ = 1;
  timer_->wait_us(1000);
  enabled_ = true;
}

void Drv8353s::Disable()
{
  PowerOff();
  enable_ = 0;
  enabled_ = false;
  init_ok_ = false;
}

void Drv8353s::PowerOn()
{
  hiz_ = 1;
  power_on_ = true;
}

void Drv8353s::PowerOff()
{
  hiz_ = 0;
  power_on_ = false;
}

uint16_t Drv8353s::Transfer(uint16_t value)
{
  cs_ = 0;
  timer_->wait_us(spi_half_period_us_);

  uint16_t result = 0;
  for (int bit = 15; bit >= 0; --bit)
  {
    mosi_ = (value & (1u << bit)) != 0;
    sck_ = 1;
    timer_->wait_us(spi_half_period_us_);
    sck_ = 0;
    result = static_cast<uint16_t>((result << 1) | (miso_.read() ? 1u : 0u));
    timer_->wait_us(spi_half_period_us_);
  }

  mosi_ = 0;
  cs_ = 1;
  timer_->wait_us(spi_half_period_us_);
  return result;
}

uint16_t Drv8353s::ReadRegister(uint8_t address)
{
  return Transfer(static_cast<uint16_t>(
             reg::Spi::READ |
             ((address & reg::Spi::ADDRESS_MASK) << reg::Spi::ADDRESS_SHIFT))) &
         reg::Spi::DATA_MASK;
}

void Drv8353s::WriteRegister(uint8_t address, uint16_t value)
{
  Transfer(static_cast<uint16_t>(
      ((address & reg::Spi::ADDRESS_MASK) << reg::Spi::ADDRESS_SHIFT) |
      (value & reg::Spi::DATA_MASK)));
}

uint8_t Drv8353s::Configure(const Config& c)
{
  const uint16_t registers[8] = {
      0,
      0,
      static_cast<uint16_t>(
          Bit(true, reg::DriverControl::OCP_ACT_SHIFT) |
          Bit(c.disable_charge_pump_uvlo, reg::DriverControl::DIS_CPUV_SHIFT) |
          Bit(c.disable_gate_drive_fault, reg::DriverControl::DIS_GDF_SHIFT) |
          Bit(c.report_overtemperature_warning, reg::DriverControl::OTW_REP_SHIFT) |
          (static_cast<uint16_t>(c.pwm_mode) << reg::DriverControl::PWM_MODE_SHIFT) |
          Bit(c.pwm_1x_asynchronous, reg::DriverControl::PWM_COM1_SHIFT) |
          Bit(c.pwm_1x_direction, reg::DriverControl::PWM_DIR_SHIFT)),
      static_cast<uint16_t>(
          (reg::GateDriveHs::UNLOCK_CODE << reg::GateDriveHs::LOCK_SHIFT) |
          (MapChoice(IDRIVEP_MA, c.source_current_hs_ma) <<
           reg::GateDriveHs::IDRIVEP_HS_SHIFT) |
          (MapChoice(IDRIVEN_MA, c.sink_current_hs_ma) <<
           reg::GateDriveHs::IDRIVEN_HS_SHIFT)),
      static_cast<uint16_t>(
          Bit(c.cycle_by_cycle, reg::GateDriveLs::CBC_SHIFT) |
          (MapChoice(TDRIVE_NS, c.peak_drive_time_ns) <<
           reg::GateDriveLs::TDRIVE_SHIFT) |
          (MapChoice(IDRIVEP_MA, c.source_current_ls_ma) <<
           reg::GateDriveLs::IDRIVEP_LS_SHIFT) |
          (MapChoice(IDRIVEN_MA, c.sink_current_ls_ma) <<
           reg::GateDriveLs::IDRIVEN_LS_SHIFT)),
      static_cast<uint16_t>(
          Bit(c.retry_time_50us, reg::OcpControl::TRETRY_SHIFT) |
          (MapChoice(DEAD_TIME_NS, c.dead_time_ns) << reg::OcpControl::DEAD_TIME_SHIFT) |
          (static_cast<uint16_t>(c.ocp_mode) << reg::OcpControl::OCP_MODE_SHIFT) |
          (MapChoice(OCP_DEG_US, c.ocp_deglitch_us) << reg::OcpControl::OCP_DEG_SHIFT) |
          (MapChoice(VDS_LVL_MV, c.vds_threshold_mv) << reg::OcpControl::VDS_LVL_SHIFT)),
      static_cast<uint16_t>(
          Bit(c.csa_fet, reg::CsaControl::CSA_FET_SHIFT) |
          Bit(c.vref_divide_by_2, reg::CsaControl::VREF_DIV_SHIFT) |
          Bit(c.low_side_reference, reg::CsaControl::LS_REF_SHIFT) |
          (MapChoice(CSA_GAIN, c.csa_gain) << reg::CsaControl::CSA_GAIN_SHIFT) |
          Bit(c.disable_sense_ocp, reg::CsaControl::DIS_SEN_SHIFT) |
          (MapChoice(SEN_LVL_MV, c.sense_threshold_mv) <<
           reg::CsaControl::SEN_LVL_SHIFT)),
      reg::DriverConfig::CAL_MODE,
  };

  constexpr uint8_t first = Address(reg::Address::DRIVER_CONTROL);
  constexpr uint8_t last = Address(reg::Address::DRIVER_CONFIG);
  for (uint8_t i = first; i <= last; ++i) { WriteRegister(i, registers[i]); }

  uint8_t verify_failures = 0;
  for (uint8_t i = first; i <= last; ++i)
  {
    if (ReadRegister(i) != registers[i]) { verify_failures |= (1u << i); }
  }
  return verify_failures;
}

Drv8353s::Status Drv8353s::ReadStatus()
{
  Status result;
  result.fault_status_1 = ReadRegister(Address(reg::Address::FAULT_STATUS_1));
  result.fault_status_2 = ReadRegister(Address(reg::Address::FAULT_STATUS_2));
  result.fault_line = fault_.read() == 0;

  const uint16_t fs1 = result.fault_status_1;
  const uint16_t fs2 = result.fault_status_2;
  result.fault = fs1 & reg::FaultStatus1::FAULT;
  result.vds_ocp = fs1 & reg::FaultStatus1::VDS_OCP;
  result.gate_drive_fault = fs1 & reg::FaultStatus1::GDF;
  result.undervoltage = fs1 & reg::FaultStatus1::UVLO;
  result.overtemperature_shutdown = fs1 & reg::FaultStatus1::OTSD;
  result.vds_phase_faults = static_cast<uint8_t>(fs1 & reg::FaultStatus1::VDS_PHASE_MASK);
  result.sense_a_ocp = fs2 & reg::FaultStatus2::SA_OC;
  result.sense_b_ocp = fs2 & reg::FaultStatus2::SB_OC;
  result.sense_c_ocp = fs2 & reg::FaultStatus2::SC_OC;
  result.overtemperature_warning = fs2 & reg::FaultStatus2::OTW;
  result.charge_pump_undervoltage = fs2 & reg::FaultStatus2::CPUV;
  result.vgs_phase_faults = static_cast<uint8_t>(fs2 & reg::FaultStatus2::VGS_PHASE_MASK);
  last_status_ = result;
  return result;
}

size_t Drv8353s::TelemetryExport(void* context, char* out, size_t out_capacity)
{
  if (context == nullptr)
  {
    return 0;
  }
  return static_cast<Drv8353s*>(context)->FormatTelemetry(out, out_capacity);
}

size_t Drv8353s::FormatTelemetry(char* out, size_t out_capacity)
{
  if (out == nullptr || out_capacity == 0)
  {
    return 0;
  }

  Status st = last_status_;
  if (enabled_)
  {
    st = ReadStatus();
  }

  // Hand-rolled to avoid newlib snprintf/_svfprintf stack usage.
  using telemetry::text::AppendKeyHex;
  using telemetry::text::AppendKeyUInt;
  size_t pos = 0;
  pos = AppendKeyUInt(out, out_capacity, pos, "ok", init_ok_ ? 1u : 0u, false);
  pos = AppendKeyUInt(out, out_capacity, pos, "en", enabled_ ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "pwr", power_on_ ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "fault", st.fault ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "fault_line",
                      st.fault_line ? 1u : 0u, true);
  pos = AppendKeyHex(out, out_capacity, pos, "fs1",
                     st.fault_status_1 & 0x7ff, 3, true);
  pos = AppendKeyHex(out, out_capacity, pos, "fs2",
                     st.fault_status_2 & 0x7ff, 3, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "uvlo",
                      st.undervoltage ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "gdf",
                      st.gate_drive_fault ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "vds_ocp",
                      st.vds_ocp ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "otw",
                      st.overtemperature_warning ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "otsd",
                      st.overtemperature_shutdown ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "cpuv",
                      st.charge_pump_undervoltage ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "sa",
                      st.sense_a_ocp ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "sb",
                      st.sense_b_ocp ? 1u : 0u, true);
  pos = AppendKeyUInt(out, out_capacity, pos, "sc",
                      st.sense_c_ocp ? 1u : 0u, true);
  pos = AppendKeyHex(out, out_capacity, pos, "vds_ph",
                     st.vds_phase_faults, 2, true);
  pos = AppendKeyHex(out, out_capacity, pos, "vgs_ph",
                     st.vgs_phase_faults, 2, true);
  return pos;
}

bool Drv8353s::CalibrateCurrentSense()
{
  constexpr uint8_t address = Address(reg::Address::CSA_CONTROL);
  const uint16_t reg6 = ReadRegister(address);
  WriteRegister(address, reg6 | reg::CsaControl::CALIBRATION_MASK);
  // The old 1 ms state machine left the calibration bits asserted until its
  // next tick. Preserve that settling time in this blocking initialization.
  timer_->wait_us(1000);
  WriteRegister(address, reg6 & static_cast<uint16_t>(~reg::CsaControl::CALIBRATION_MASK));
  return (ReadRegister(address) & reg::CsaControl::CALIBRATION_MASK) == 0;
}

float Drv8353s::CsaSettlingTimeSeconds(uint8_t gain)
{
  switch (gain)
  {
    case 5: return 0.75e-6f;
    case 10: return 1.00e-6f;
    case 20: return 1.50e-6f;
    case 40: return 2.50e-6f;
    default: return 3.00e-6f;
  }
}

}  // namespace device
