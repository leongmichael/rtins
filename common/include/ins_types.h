#ifndef INS_TYPES_H
#define INS_TYPES_H

#include <stdint.h>

/*
 * HAL-free types shared by firmware and host.
 * Do not include CMSIS, UART, or POSIX headers here.
 */

typedef enum {
    INS_OK = 0,
    INS_ERR_INVALID,
    INS_ERR_NOT_IMPLEMENTED
} ins_status_t;

/* Microseconds. uint32_t wraps ~1.2 h — fine for emulator sessions. */
typedef uint32_t ins_time_t;

typedef struct {
    ins_time_t t_us;
    float acc_mps2[3]; /* specific force, body: x fwd, y right, z down */
    float gyr_rads[3]; /* rad/s, body */
} ins_imu_t;

typedef struct {
    ins_time_t t_us;
    float lat_deg;
    float lon_deg;
    float alt_m;
    uint8_t fix; /* 0 = invalid, non-zero = valid */
} ins_gnss_t;

#endif /* INS_TYPES_H */
