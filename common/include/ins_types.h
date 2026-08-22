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

/*
 * Time representation is an architectural choice. Pick one and
 * replace this typedef — do not mix units across the codebase.
 *
 * Candidates:
 *   uint32_t microseconds (wraps ~1.2 h)
 *   uint64_t microseconds
 *   uint32_t IMU ticks + a known sample rate
 */
typedef uint32_t ins_time_t;

#endif /* INS_TYPES_H */
