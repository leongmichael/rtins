#ifndef INS_PACKET_H
#define INS_PACKET_H

#include <stdint.h>

#include "ins_types.h"

/*
 * Byte-oriented frame parser. Implementation and wire format are TBD.
 * UART ISR (firmware) and replay (host) both call this.
 */

typedef enum {
    INS_PACKET_NEED_MORE = 0,
    INS_PACKET_FRAME_READY,
    INS_PACKET_BAD
} ins_packet_result_t;

void ins_packet_reset(void);

/* Feed one byte. Frame payload/type comes later. */
ins_packet_result_t ins_packet_feed(uint8_t byte);

#endif /* INS_PACKET_H */
