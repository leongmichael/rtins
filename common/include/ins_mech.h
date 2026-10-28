#ifndef INS_MECH_H
#define INS_MECH_H

#include "ins_types.h"

/*
 * Flat-Earth strapdown in NED. No Earth rate, no transport rate, no EKF.
 *
 * Attitude: unit quaternion q_nb (scalar-first, body → NED).
 * Body: x forward, y right, z down.
 * IMU accel is specific force (m/s^2). Level, still, body=NED: f ≈ [0,0,-g].
 */

#define INS_G 9.80665f

typedef struct {
    float q_nb[4];   /* w, x, y, z */
    float v_ned[3];  /* m/s */
    float p_ned[3];  /* m from origin */
    ins_time_t t_us;
    uint8_t have_sample;
} ins_mech_t;

void ins_mech_init(ins_mech_t *m);
void ins_mech_imu(ins_mech_t *m, const ins_imu_t *imu);

#endif /* INS_MECH_H */
