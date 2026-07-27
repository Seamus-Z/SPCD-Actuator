// Application for moteus hardware (STM32G474)
// Bare-metal — app lives at 0x08010000
// CAN listener: standard ID 0x7E with payload "BOOT" requests bootloader mode.
#include <cstring>

#include "HAL/fdcan.h"
#include "bootloader/bootloader.h"

static constexpr uint32_t kBootRequestId = 0x7E;
static constexpr char kBootRequestPayload[] = "BOOT";

static bool check_boot_request(hal::FDCan& can) {
    FDCAN_RxHeaderTypeDef header = {};
    uint8_t data[64] = {};
    size_t len = 0;

    // Opportunistic bus-off recovery; cheap and keeps RX alive after faults.
    const auto st = can.status();
    if (st.BusOff) {
        can.RecoverBusOff();
        return false;
    }

    if (!can.Poll(&header, data, sizeof(data), &len)) {
        return false;
    }

    if (header.IdType != FDCAN_STANDARD_ID ||
        header.Identifier != kBootRequestId ||
        header.RxFrameType != FDCAN_DATA_FRAME ||
        len != sizeof(kBootRequestPayload) - 1) {
        return false;
    }

    return std::memcmp(data, kBootRequestPayload,
                       sizeof(kBootRequestPayload) - 1) == 0;
}

extern "C" {

extern uint8_t __bss_start__;
extern uint8_t __bss_end__;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sidata;

void AppDefault(void)   { while (1); }
void AppHardFault(void) { while (1); }

void AppReset(void) {
    // Initialize .bss and .data (no C runtime in this bare-metal image)
    for (uint8_t* p = &__bss_start__; p < &__bss_end__; ++p) *p = 0;
    {
        uint32_t* dst = &_sdata;
        const uint32_t* src = &_sidata;
        while (dst < &_edata) *dst++ = *src++;
    }

    SCB->VTOR = 0x08010000;
    SystemCoreClock = 16000000;
    __enable_irq();

    // Application owns FDCAN via STM32 HAL wrapper.
    hal::FDCan::Options can_options;
    can_options.instance = FDCAN2;
    can_options.slow_bitrate = 1000000;
    can_options.fast_bitrate = 2000000;
    can_options.fdcan_frame = true;
    can_options.bitrate_switch = true;
    can_options.automatic_retransmission = false;
    hal::FDCan can(can_options);

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE15_Msk)
                 | (1 << GPIO_MODER_MODE15_Pos);

    // Long ON to confirm app startup
    GPIOB->BSRR = 0x80000000;
    for (volatile uint32_t d = 0; d < 2000000; d++) { __NOP(); }
    GPIOB->BSRR = 0x00008000;
    for (volatile uint32_t d = 0; d < 500000; d++) { __NOP(); }

    while (1) {
        if (check_boot_request(can)) {
            // Boot request: fast blink 3 times, then reset into bootloader
            for (int i = 0; i < 3; i++) {
                GPIOB->BSRR = 0x80000000;
                for (volatile uint32_t d = 0; d < 200000; d++) { __NOP(); }
                GPIOB->BSRR = 0x00008000;
                for (volatile uint32_t d = 0; d < 200000; d++) { __NOP(); }
            }
            // A production motor application must disable PWM, gate drive,
            // timers, and any other energy-producing peripherals here before
            // entering the bootloader.
            EnterBootloader();
        }

        // Very fast heartbeat: deliberately obvious for verifying that this
        // newly CAN-flashed application is the image currently executing.
        GPIOB->BSRR = 0x80000000;
        for (volatile uint32_t d = 0; d < 100000; d++) { __NOP(); }
        GPIOB->BSRR = 0x00008000;
        for (volatile uint32_t d = 0; d < 100000; d++) { __NOP(); }
    }
}

} // extern "C"
