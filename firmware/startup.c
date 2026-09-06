#include <stdint.h>

extern int main(void);

/* Provided by the linker script once SECTIONS exists. */
extern uint32_t _estack;
extern uint32_t _sidata; 
extern uint32_t _sdata;  
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;


void Reset_Handler(void);
void Default_Handler(void);

void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;

    while (dst < &_edata) {
        *dst++ = *src++;

    }

    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }
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
 */

__attribute__((section(".isr_vector"), used))
const uint32_t isr_vector[] = {
    (uint32_t)&_estack,
    (uint32_t)Reset_Handler,
};
