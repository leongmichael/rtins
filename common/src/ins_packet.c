#include "ins_packet.h"

void ins_packet_reset(void)
{
    /* TODO: clear parser state once a frame format exists */
}

ins_packet_result_t ins_packet_feed(uint8_t byte)
{
    (void)byte;
    return INS_PACKET_NEED_MORE;
}
