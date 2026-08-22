#include <stdint.h>

extern int main(void);

/* Provided by the linker script once SECTIONS exists. */
extern uint32_t _estack;

void Reset_Handler(void);
void Default_Handler(void);

void Reset_Handler(void)
{
    /* TODO: copy .data from load address to SRAM (if you use .data) */
    /* TODO: zero .bss */
    (void)main();
    for (;;) {
    }
}

void Default_Handler(void)
{
    for (;;) {
    }
}

/*
 * Cortex-M vector table must sit at 0x00000000.
 * [0] initial stack pointer
 * [1] Reset_Handler
 * Remaining IRQs: leave weak aliases to Default_Handler when you add them.
 *
 * You write the table. Review checklist: SP in SRAM, Thumb bit on
 * handlers, table in a section the linker places at ORIGIN(CODE).
 */
