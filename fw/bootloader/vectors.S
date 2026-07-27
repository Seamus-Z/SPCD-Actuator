// ISR vector table at 0x08000000
// CPU reads SP from 0x00, reset vector from 0x04 → BootloaderEntry at 0x0800C001 (thumb)

.syntax unified
.cpu cortex-m4
.thumb

.section .vectors,"a",%progbits
.global g_pfnVectors

g_pfnVectors:
    .word 0x20020000         //  0: SP (top of 128KB SRAM/CCM alias)
    .word BootloaderEntry    //  1: Reset → bootloader
    .word Default_Handler    //  2: NMI
    .word HardFault_Handler  //  3: HardFault
    .word Default_Handler    //  4: MemManage
    .word Default_Handler    //  5: BusFault
    .word Default_Handler    //  6: UsageFault
    .word 0                  //  7: reserved
    .word 0                  //  8: reserved
    .word 0                  //  9: reserved
    .word 0                  // 10: reserved
    .word SVC_Handler        // 11: SVCall
    .word Default_Handler    // 12: DebugMon
    .word 0                  // 13: reserved
    .word PendSV_Handler     // 14: PendSV
    .word SysTick_Handler    // 15: SysTick
    // IRQs 16-117: fill with Default_Handler (STM32G474 has up to FMAC_IRQn=101)
    .rept 102
    .word Default_Handler
    .endr
