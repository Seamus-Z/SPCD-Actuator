#include "BL_CanDriver.h"

#include "stm32g4xx_fdcan_typedefs.h"

namespace
{

constexpr int kDlcToSize[] =
{
    0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

constexpr int RoundUpDlc(int size)
{
  for (int dlc = 0;; dlc++)
  {
    if (size <= kDlcToSize[dlc])
    {
      return dlc;
    }
  }
  return 15;
}

}  // namespace

BL_CanDriver::BL_CanDriver(FDCAN_GlobalTypeDef* fdcan) : fdcan_(fdcan)
{
  uint32_t ram_offset = 0;
  if (fdcan == FDCAN2)
  {
    ram_offset = SRAMCAN_SIZE;
  }
  if (fdcan == FDCAN3)
  {
    ram_offset = SRAMCAN_SIZE * 2U;
  }
  rx_fifo0_offset_ = ram_offset + SRAMCAN_RF0SA;
  tx_fifoq_offset_ = ram_offset + SRAMCAN_TFQSA;
  rx_fifo0_addr_ = SRAMCAN_BASE + rx_fifo0_offset_;
  tx_fifoq_addr_ = SRAMCAN_BASE + tx_fifoq_offset_;
}

void BL_CanDriver::Init()
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

  // FDCAN2 on PB5(RX)/PB6(TX), AF9.
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE5_Msk | GPIO_MODER_MODE6_Msk)) |
                 (2 << GPIO_MODER_MODE5_Pos) | (2 << GPIO_MODER_MODE6_Pos);
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFFu << 20)) | (9u << 20) | (9u << 24);
  GPIOB->OSPEEDR =
      (GPIOB->OSPEEDR & ~(GPIO_OSPEEDR_OSPEED5_Msk | GPIO_OSPEEDR_OSPEED6_Msk)) |
      (3UL << GPIO_OSPEEDR_OSPEED5_Pos) | (3UL << GPIO_OSPEEDR_OSPEED6_Pos);

  RCC->APB1RSTR1 |= RCC_APB1RSTR1_FDCANRST;
  for (volatile int i = 0; i < 100; i++)
  {
    __NOP();
  }
  RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_FDCANRST;
  for (volatile int i = 0; i < 100; i++)
  {
    __NOP();
  }

  fdcan_->CCCR |= FDCAN_CCCR_CCE;

  // 1 Mbps nominal / 2 Mbps data @ 16 MHz PCLK1.
  fdcan_->NBTP = (3 << FDCAN_NBTP_NSJW_Pos) | (3 << FDCAN_NBTP_NTSEG2_Pos) |
                 (10 << FDCAN_NBTP_NTSEG1_Pos) | (0 << FDCAN_NBTP_NBRP_Pos);
  fdcan_->DBTP = (1 << FDCAN_DBTP_DSJW_Pos) | (1 << FDCAN_DBTP_DTSEG2_Pos) |
                 (4 << FDCAN_DBTP_DTSEG1_Pos) | (0 << FDCAN_DBTP_DBRP_Pos);

  fdcan_->CCCR |= FDCAN_CCCR_FDOE | FDCAN_CCCR_DAR | FDCAN_CCCR_PXHD;
  fdcan_->RXGFC =
      (0 << FDCAN_RXGFC_ANFS_Pos) | (0 << FDCAN_RXGFC_ANFE_Pos);

  fdcan_->CCCR &= ~FDCAN_CCCR_CCE;
  for (volatile int i = 0; i < 100000; i++)
  {
    __NOP();
  }
  fdcan_->CCCR &= ~FDCAN_CCCR_INIT;
  for (volatile int i = 0; i < 100000; i++)
  {
    __NOP();
  }
}

bool BL_CanDriver::Poll(BL_CanFrame& frame)
{
  if (fdcan_->PSR & FDCAN_PSR_BO)
  {
    fdcan_->CCCR &= ~FDCAN_CCCR_INIT;
    return false;
  }
  if ((fdcan_->RXF0S & FDCAN_RXF0S_F0FL) == 0)
  {
    return false;
  }

  const auto get_index =
      (fdcan_->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
  auto* rx_address = reinterpret_cast<uint32_t*>(
      rx_fifo0_addr_ + get_index * SRAMCAN_RF0_SIZE);

  const uint32_t w1 = rx_address[0];
  frame.identifier = (w1 & FDCAN_ELEMENT_MASK_STDID) >> 18;
  if (w1 & FDCAN_ELEMENT_MASK_XTD)
  {
    frame.identifier = w1 & FDCAN_ELEMENT_MASK_EXTID;
  }

  const uint32_t w2 = rx_address[1];
  frame.dlc = (w2 & FDCAN_ELEMENT_MASK_DLC) >> 16;
  frame.bit_rate_switch = w2 & FDCAN_ELEMENT_MASK_BRS;
  frame.fdformat = w2 & FDCAN_ELEMENT_MASK_FDF;
  frame.size = kDlcToSize[frame.dlc];

  auto* pdata = reinterpret_cast<uint8_t*>(&rx_address[2]);
  for (uint32_t i = 0; i < frame.size && i < 64; i++)
  {
    frame.data[i] = pdata[i];
  }

  fdcan_->RXF0A = get_index;
  return true;
}

bool BL_CanDriver::Send(uint32_t identifier,
                        const std::string_view& data,
                        bool brs)
{
  uint32_t timeout = 1000000;
  while (fdcan_->TXFQS & FDCAN_TXFQS_TFQF)
  {
    if (--timeout == 0)
    {
      return false;
    }
  }

  const auto put_index =
      (fdcan_->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
  const uint32_t dlc = RoundUpDlc(static_cast<int>(data.size()));

  const uint32_t element_w1 =
      (identifier >= 2048)
          ? (FDCAN_ESI_ACTIVE | FDCAN_EXTENDED_ID | FDCAN_DATA_FRAME | identifier)
          : (FDCAN_ESI_ACTIVE | FDCAN_STANDARD_ID | FDCAN_DATA_FRAME |
             (identifier << 18));
  const uint32_t element_w2 = FDCAN_NO_TX_EVENTS | FDCAN_FD_CAN |
                              (brs ? FDCAN_BRS_ON : 0) | (dlc << 16);

  auto* tx_address = reinterpret_cast<uint32_t*>(
      tx_fifoq_addr_ + put_index * SRAMCAN_TFQ_SIZE);
  *tx_address++ = element_w1;
  *tx_address++ = element_w2;

  const size_t rounded_size = kDlcToSize[dlc];
  for (size_t i = 0; i < rounded_size; i += 4)
  {
    auto get_byte = [&](int offset) -> uint8_t
    {
      const size_t pos = i + static_cast<size_t>(offset);
      return pos < data.size() ? static_cast<uint8_t>(data[pos]) : 0;
    };
    *tx_address++ = (get_byte(3) << 24) | (get_byte(2) << 16) |
                    (get_byte(1) << 8) | get_byte(0);
  }

  fdcan_->TXBAR = (1u << put_index);
  return true;
}
