// Thin ISR-safe SPI master (software CS, 16-bit transfers).
// Ported/simplified from moteus Stm32Spi — no DMA.
#pragma once

#include "drivers/DigitalOut.h"
#include "hal/spi_api.h"
#include "PinNames.h"
#include "ports/platform_ports.h"

namespace hal
{

class Stm32Spi final : public ports::ISpiBus
{
 public:
  struct Options
  {
    PinName mosi = NC;
    PinName miso = NC;
    PinName sck = NC;
    PinName cs = NC;
    int frequency = 6000000;
    int width = 16;
    int mode = 0;  // CPOL=0 CPHA=0
    uint16_t timeout = 20000;
  };

  explicit Stm32Spi(const Options& options) : options_(options), cs_(options.cs, 1)
  {
    spi_init(&spi_, options.mosi, options.miso, options.sck, NC);
    spi_format(&spi_, options.width, options.mode, 0);
    spi_frequency(&spi_, options.frequency);

    auto* const spi = spi_.spi.handle.Instance;
    spi->CR1 &= ~SPI_CR1_SPE;
  }

  uint16_t Transfer16(uint16_t value) override
  {
    BeginTransfer16(value);
    return FinishTransfer16();
  }

  void BeginTransfer16(uint16_t value) override
  {
    auto* const spi = spi_.spi.handle.Instance;
    cs_.write(0);

    uint16_t timeout = options_.timeout;
    while (((spi->SR & SPI_SR_BSY) != 0) && timeout)
    {
      timeout--;
    }

    while (spi->SR & SPI_SR_RXNE)
    {
      (void)spi->DR;
    }
    spi->DR = value;
    spi->CR1 |= SPI_CR1_SPE;
  }

  uint16_t FinishTransfer16() override
  {
    auto* const spi = spi_.spi.handle.Instance;
    uint16_t timeout = options_.timeout;

    while (((spi->SR & SPI_SR_RXNE) == 0) && timeout)
    {
      timeout--;
    }
    const uint16_t result = static_cast<uint16_t>(spi->DR);
    while (((spi->SR & SPI_SR_TXE) == 0) && timeout)
    {
      timeout--;
    }
    while (((spi->SR & SPI_SR_BSY) != 0) && timeout)
    {
      timeout--;
    }
    spi->CR1 &= ~SPI_CR1_SPE;

    cs_.write(1);
    return result;
  }

 private:
  Options options_{};
  spi_t spi_{};
  mbed::DigitalOut cs_;
};

}  // namespace hal
