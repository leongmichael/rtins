#include "ins_packet.h"
#include "uart.h"

int main(void)
{
    uint8_t b;

    uart_init();
    uart_putc((uint8_t)'A'); /* bring-up: one visible byte on stdio */

    ins_packet_reset();

    for (;;) {
        if (!uart_try_getc(&b)) {
            continue;
        }

        switch (ins_packet_feed(b)) {
        case INS_PACKET_FRAME_READY:
            if (ins_packet_type() == INS_PKT_TYPE_IMU) {
                uart_putc((uint8_t)'I');
            } else if (ins_packet_type() == INS_PKT_TYPE_GNSS) {
                uart_putc((uint8_t)'G');
            } else {
                uart_putc((uint8_t)'?');
            }
            break;
        case INS_PACKET_BAD:
            uart_putc((uint8_t)'!');
            break;
        case INS_PACKET_NEED_MORE:
        default:
            break;
        }
    }
}
