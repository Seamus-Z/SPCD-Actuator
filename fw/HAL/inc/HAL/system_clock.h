// Bare-metal system clock bring-up aligned with moteus G4:
// HSI 16 MHz → PLL → SYSCLK/HCLK 170 MHz, APB1/APB2 85 MHz.
#pragma once

namespace hal
{

// Safe to call from bootloader and app, including when PLL is already SYSCLK.
void SetupSystemClock();

}  // namespace hal
