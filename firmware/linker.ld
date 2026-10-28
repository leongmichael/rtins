/*
 * MPS2 AN386 (QEMU mps2-an386) — MEMORY only.
 * Sources: ARM AN386 / QEMU hw/arm/mps2.c
 *
 *   0x00000000  4 MiB   ZBT SSRAM1   (code + vectors)
 *   0x20000000  4 MiB   ZBT SSRAM2/3 (data, heap/stack if you use them)
 *   0x21000000 16 MiB   PSRAM        (unused for now)
 *   0x40004000          CMSDK UART0
 *
 */

ENTRY(Reset_Handler)

MEMORY
{
    CODE (rx)  : ORIGIN = 0x00000000, LENGTH = 4M
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 4M
}

SECTIONS
{
    .isr_vector : ALIGN(4){
        KEEP(*(.isr_vector))
    } > CODE

    .text : ALIGN(4){
        *(.text*)
        *(.rodata*)
    } > CODE


    .data : ALIGN(4){
        _sdata = .;
        *(.data*)
        _edata = .;
    } > SRAM AT > CODE

    _sidata = LOADADDR(.data);

    .bss : ALIGN(4) {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

    _estack = ORIGIN(SRAM) + LENGTH(SRAM);

}