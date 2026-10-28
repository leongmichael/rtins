#ifndef INS_EKF_H
#define INS_EKF_H

#include "ins_mech.h"

/*
 * Loosely coupled error-state EKF. Nominal trajectory is ins_mech.
 *
 * x = [ ψ(3), δv(3), δp(3), b_g(3), b_a(3) ]^T
 *   ψ   nav-frame attitude error, C_true ≈ (I - [ψ×]) C_est
 *   δv, δp  NED, true minus estimated
 *   b_g, b_a  body gyro (rad/s) and accel (m/s^2) bias estimates
 *
 * GNSS v1: position only. First valid fix can set the LLA origin.
 * Flat Earth, no Earth-rate in F.
 */

#define INS_EKF_N 15

#define INS_EKF_SIGMA_G   1.0e-3f
#define INS_EKF_SIGMA_A   5.0e-2f
#define INS_EKF_SIGMA_BG  1.0e-5f
#define INS_EKF_SIGMA_BA  1.0e-4f
#define INS_EKF_R_POS     4.0f /* m^2, diagonal */

typedef struct {
    ins_mech_t mech;
    float bg[3];
    float ba[3];
    float P[INS_EKF_N][INS_EKF_N];
    uint8_t have_origin;
    float lat0_deg;
    float lon0_deg;
    float alt0_m;
} ins_ekf_t;

void ins_ekf_init(ins_ekf_t *e);
void ins_ekf_set_origin(ins_ekf_t *e, float lat_deg, float lon_deg, float alt_m);
void ins_ekf_imu(ins_ekf_t *e, const ins_imu_t *imu);
void ins_ekf_gnss(ins_ekf_t *e, const ins_gnss_t *gnss);

#endif /* INS_EKF_H */
