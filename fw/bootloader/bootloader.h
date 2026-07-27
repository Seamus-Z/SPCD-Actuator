// Production-grade multiplex CAN-FD bootloader for xtellar
// Placed at 0x0800C000, protocol: mjlib multiplex text over FDCAN channel 1
//
// Boot flow:
//   1. Power-on → BootloaderEntry checks magic at 0x20000000
//   2. Magic == 0xB00710AD → run bootloader (multiplex text protocol on FDCAN)
//   3. Magic != 0xB00710AD → jump to app at 0x08010000
//   4. App sets magic + NVIC_SystemReset() to re-enter bootloader
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/// Bootloader entry point — placed at 0x0800C000 (.boot_entry section).
/// Called from the ISR vector table at 0x08000000.
void BootloaderEntry();

/// Application-visible function to enter bootloader mode.
/// The application must put the motor and all power stages in a safe state
/// before calling this function.  It sets the retained boot magic and resets.
void EnterBootloader();

#ifdef __cplusplus
}
#endif
