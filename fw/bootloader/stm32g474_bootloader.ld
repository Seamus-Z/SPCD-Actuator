/* Production bootloader linker script for STM32G474
 *
 * Memory layout:
 *   0x08000000 - 0x0800BFFF: ISR vector table (vectors.S → .vectors section, 48KB)
 *   0x0800C000 - 0x0800FFFF: Bootloader code (.boot_entry + .text + .rodata, 16KB)
 *   0x08010000 - 0x0807FFFF: Application (448KB)
 *
 * The bootloader is bare-metal — no C runtime, no standard library.
 * Initial SP is set in vectors.S. BSS is manually zeroed at entry.
 */
MEMORY
{
  VECTORS  (rx) : ORIGIN = 0x08000000, LENGTH = 0xC000   /* 48KB for vector table */
  BL_FLASH (rx) : ORIGIN = 0x0800C000, LENGTH = 16K      /* 16KB for bootloader code */
  /* Keep 0x20000000-0x200001ff reserved for reset-persistent handoff data. */
  RAM     (rwx) : ORIGIN = 0x20000200, LENGTH = 128K - 0x200
}

ENTRY(BootloaderEntry)

SECTIONS
{
    .vectors : {
        KEEP(*(.vectors))
    } > VECTORS

    .boot_entry : {
        KEEP(*bootloader.o(.boot_entry))
        KEEP(*(.boot_entry))
        KEEP(*bootloader.o(.multiplex_bootloader))
        KEEP(*(.multiplex_bootloader))
    } > BL_FLASH

    .text : {
        *(.text*)
        *(.rodata*)
    } > BL_FLASH

    .ARM.extab : { *(.ARM.extab*) } > BL_FLASH

    __exidx_start = .;
    .ARM.exidx : {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > BL_FLASH
    __exidx_end = .;

    __etext = .;
    _sidata = .;

    .data : AT (__etext)
    {
        __data_start__ = .;
        _sdata = .;
        *(vtable)
        *(.data*)
        . = ALIGN(8);
        __data_end__ = .;
        _edata = .;
    } > RAM

    .bss : {
        . = ALIGN(8);
        __bss_start__ = .;
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(8);
        __bss_end__ = .;
        _ebss = .;
    } > RAM

    /* Stack top — vectors.S sets SP to 0x20020000 (top of CCM alias) */
    __StackTop = 0x20020000;
    _estack = __StackTop;
    PROVIDE(__stack = __StackTop);

    /* Symbols required by mbed retarget (not actually used by bootloader) */
    __end__ = .;
    __HeapLimit = __StackTop;
}
