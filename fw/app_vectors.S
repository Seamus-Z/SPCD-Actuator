// Application ISR vector table at 0x08010000
// CPU reads SP from 0x08010000, reset vector from 0x08010004 → AppReset

.syntax unified
.cpu cortex-m4
.thumb

.section .isr_vector,"a",%progbits
.global g_app_vectors

g_app_vectors:
    .word 0x20020000       //  0: SP (top of stack = 128KB SRAM)
    .word AppReset         //  1: Reset → AppReset
    .word AppDefault       //  2: NMI
    .word AppHardFault     //  3: HardFault
    .word AppDefault       //  4: MemManage
    .word AppDefault       //  5: BusFault
    .word AppDefault       //  6: UsageFault
    .word 0                //  7: reserved
    .word 0                //  8: reserved
    .word 0                //  9: reserved
    .word 0                // 10: reserved
    .word AppDefault       // 11: SVCall
    .word AppDefault       // 12: DebugMon
    .word 0                // 13: reserved
    .word AppDefault       // 14: PendSV
    .word AppDefault       // 15: SysTick
    // IRQs 16-117: fill with AppDefault (STM32G474 has up to FMAC_IRQn=101)
    .rept 102
    .word AppDefault
    .endr
