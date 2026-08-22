#include <stdio.h>

#include "ins_packet.h"

/*
 * Host-side replay. Later this opens a file or TCP socket and writes
 * the same byte stream the firmware UART ISR will parse.
 *
 * libc is allowed here. common/ stays HAL-free.
 */

int main(void)
{
    ins_packet_reset();
    (void)ins_packet_feed(0);
    puts("host/replay: stub");
    return 0;
}
