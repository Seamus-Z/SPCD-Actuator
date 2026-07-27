// Multiplex CAN-FD bootloader for xtellar (STM32G474).
//
// Layout:
//   vectors.S / *.ld   — stay at package root
//   inc/               — all headers
//   src/               — all C++ sources
//
// Public API used by the application image:
//   EnterBootloader() — set magic + soft reset into Serve mode
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

void BootloaderEntry();
void EnterBootloader();

#ifdef __cplusplus
}
#endif
