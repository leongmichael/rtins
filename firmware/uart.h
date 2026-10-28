#ifndef FIRMWARE_UART_H
#define FIRMWARE_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * CMSDK APB UART0 on MPS2 AN386.
 * Base from QEMU/AN386: 0x40004000
 *
 * Offsets (ARM CMSDK UART):
 *   0x000 DATA
 *   0x004 STATE
 *   0x008 CTRL
 *   0x00C INTSTATUS / INTCLEAR
 *   0x010 BAUDDIV
 */

#define UART0_BASE (0x40004000u)

typedef struct {
    volatile uint32_t DATA;
    volatile uint32_t STATE;
    volatile uint32_t CTRL;
    volatile uint32_t INTSTATUS;
    volatile uint32_t BAUDDIV;
} cmsdk_uart_t;

#define UART0 ((cmsdk_uart_t *)UART0_BASE)

#define UART_STATE_TXFULL (1u << 0)
#define UART_STATE_RXFULL (1u << 1)

#define UART_CTRL_TX_EN (1u << 0)
#define UART_CTRL_RX_EN (1u << 1)

/* QEMU MPS2 SYSCLK is 25 MHz. Divisor is ignored for the char backend
 * but must be non-zero for a well-formed CMSDK UART. */
#define UART_PCLK_HZ 25000000u
#define UART_BAUD    115200u

void uart_init(void);
void uart_putc(uint8_t c);
void uart_write(const uint8_t *buf, size_t n);

/* Returns true and writes *out if a byte is waiting. */
bool uart_try_getc(uint8_t *out);

#endif /* FIRMWARE_UART_H */
