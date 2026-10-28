#ifndef INS_PACKET_H
#define INS_PACKET_H

#include <stddef.h>
#include <stdint.h>

#include "ins_types.h"

/*
 * UART frame (little-endian payloads):
 *
 *   A5 5A | type | len | payload[len] | crc8
 *
 * crc8 (poly 0x07, init 0) covers type, len, and payload — not the sync.
 *
 * type 0x01 IMU  payload 28 bytes:
 *   u32 t_us, f32 ax ay az (m/s^2), f32 gx gy gz (rad/s)
 *
 * type 0x02 GNSS payload 17 bytes:
 *   u32 t_us, f32 lat lon (deg), f32 alt (m), u8 fix
 */

#define INS_PKT_SYNC0      0xA5u
#define INS_PKT_SYNC1      0x5Au
#define INS_PKT_TYPE_IMU   0x01u
#define INS_PKT_TYPE_GNSS  0x02u
#define INS_PKT_MAX_PAYLOAD 64u

#define INS_PKT_IMU_LEN  28u
#define INS_PKT_GNSS_LEN 17u

typedef enum {
    INS_PACKET_NEED_MORE = 0,
    INS_PACKET_FRAME_READY,
    INS_PACKET_BAD
} ins_packet_result_t;

void ins_packet_reset(void);

/* Feed one byte. On FRAME_READY, type/len/payload are valid until the
 * next feed() or reset(). */
ins_packet_result_t ins_packet_feed(uint8_t byte);

uint8_t ins_packet_type(void);
uint8_t ins_packet_len(void);
const uint8_t *ins_packet_payload(void);

/* Returns encoded size, or 0 if the arguments will not fit. */
size_t ins_packet_encode(uint8_t type, const uint8_t *payload, uint8_t len,
                         uint8_t *out, size_t out_cap);

void ins_imu_pack(const ins_imu_t *in, uint8_t payload[INS_PKT_IMU_LEN]);
void ins_gnss_pack(const ins_gnss_t *in, uint8_t payload[INS_PKT_GNSS_LEN]);

ins_status_t ins_imu_unpack(const uint8_t *payload, uint8_t len, ins_imu_t *out);
ins_status_t ins_gnss_unpack(const uint8_t *payload, uint8_t len, ins_gnss_t *out);

#endif /* INS_PACKET_H */
