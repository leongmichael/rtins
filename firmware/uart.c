#include "uart.h"

void uart_init(void)
{
    UART0->BAUDDIV = UART_PCLK_HZ / UART_BAUD;
    UART0->CTRL = UART_CTRL_TX_EN | UART_CTRL_RX_EN;
}

void uart_putc(uint8_t c)
{
    while ((UART0->STATE & UART_STATE_TXFULL) != 0u) {
    }
    UART0->DATA = c;
}

bool uart_try_getc(uint8_t *out)
{
    if ((UART0->STATE & UART_STATE_RXFULL) == 0u) {
        return false;
    }
    *out = (uint8_t)(UART0->DATA & 0xFFu);
    return true;
}
