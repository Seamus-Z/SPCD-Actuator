/* Bare-metal application linker script for STM32G474
   App lives at 0x08010000 (after bootloader at 0x0800C000) */
STACK_SIZE = 0x800;

MEMORY
{
  APP_FLASH (rx) : ORIGIN = 0x08010000, LENGTH = 448K
  /* 0x20000000-0x200001ff is shared reset-persistent handoff storage. */
  RAM (rwx)      : ORIGIN = 0x20000200, LENGTH = 128K - 0x200
}

ENTRY(AppReset)

SECTIONS
{
    .isr_vector : {
        KEEP(*(.isr_vector))
    } > APP_FLASH

    .app_entry : {
        KEEP(*main.o(.app_entry))
        KEEP(*(.app_entry))
    } > APP_FLASH

    .text : {
        *(.text*)
        *(.rodata*)
    } > APP_FLASH

    .ARM.extab : { *(.ARM.extab*) } > APP_FLASH
    .ARM.exidx : { *(.ARM.exidx*) } > APP_FLASH

    .data : ALIGN(8) {
        _sdata = .;
        *(.data*)
        . = ALIGN(8);
        _edata = .;
    } > RAM AT> APP_FLASH
    _sidata = LOADADDR(.data);

    .bss : {
        . = ALIGN(8);
        __bss_start__ = .;
        *(.bss*) *(COMMON)
        . = ALIGN(8);
        __bss_end__ = .;
    } > RAM

    .heap (COPY) : {
        __end__ = .;
        end = __end__;
        *(.heap*)
        . = ORIGIN(RAM) + LENGTH(RAM) - STACK_SIZE;
        __HeapLimit = .;
    } > RAM

    __StackTop = ORIGIN(RAM) + LENGTH(RAM);
    __StackLimit = __StackTop - STACK_SIZE;
    _estack = __StackTop;
    PROVIDE(__stack = __StackTop);

    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")
}
