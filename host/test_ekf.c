#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ins_ekf.h"

static int fail(const char *msg)
{
    fprintf(stderr, "test_ekf: %s\n", msg);
    return 1;
}

static void still_imu(ins_imu_t *imu, ins_time_t t_us)
{
    memset(imu, 0, sizeof(*imu));
    imu->t_us = t_us;
    imu->acc_mps2[2] = -INS_G;
}

static int test_open_loop_grows_p(void)
{
    ins_ekf_t e;
    ins_imu_t imu;
    unsigned i;
    float ppos_early;
    float ppos_late;
    float pnorm;

    ins_ekf_init(&e);
    for (i = 0; i < 21u; i++) {
        still_imu(&imu, i * 10000u);
        ins_ekf_imu(&e, &imu);
    }
    ppos_early = e.P[6][6];

    for (i = 21u; i < 201u; i++) {
        still_imu(&imu, i * 10000u);
        ins_ekf_imu(&e, &imu);
    }
    ppos_late = e.P[6][6];
    pnorm = sqrtf(e.mech.p_ned[0] * e.mech.p_ned[0]
                  + e.mech.p_ned[1] * e.mech.p_ned[1]
                  + e.mech.p_ned[2] * e.mech.p_ned[2]);

    printf("open   P_nn %.3f -> %.3f |p|=%.3e\n",
           (double)ppos_early, (double)ppos_late, (double)pnorm);

    if (ppos_late <= ppos_early) {
        return fail("open-loop P_nn did not grow");
    }
    if (pnorm > 1.0e-3f) {
        return fail("open-loop still IMU walked in position");
    }
    return 0;
}

static int test_gnss_pulls_north(void)
{
    ins_ekf_t e;
    ins_imu_t imu;
    ins_gnss_t gnss;
    unsigned i;
    float p_n;

    ins_ekf_init(&e);
    ins_ekf_set_origin(&e, 37.4f, -122.1f, 0.0f);
    e.mech.p_ned[0] = 10.0f;

    memset(&gnss, 0, sizeof(gnss));
    gnss.lat_deg = 37.4f;
    gnss.lon_deg = -122.1f;
    gnss.alt_m = 0.0f;
    gnss.fix = 1;

    for (i = 0; i < 301u; i++) {
        still_imu(&imu, i * 10000u);
        ins_ekf_imu(&e, &imu);
        if (i > 0u && (i % 100u) == 0u) {
            gnss.t_us = imu.t_us;
            ins_ekf_gnss(&e, &gnss);
        }
    }

    p_n = e.mech.p_ned[0];
    printf("gnss   p_n %.3f m (started 10) P_nn %.3f\n",
           (double)p_n, (double)e.P[6][6]);

    if (fabsf(p_n) > 2.0f) {
        return fail("GNSS did not pull north error toward origin");
    }
    if (e.P[6][6] > 20.0f) {
        return fail("GNSS did not reduce P_nn");
    }
    return 0;
}

static int test_still_aided_bounded(void)
{
    ins_ekf_t e;
    ins_imu_t imu;
    ins_gnss_t gnss;
    unsigned i;
    float pnorm;

    ins_ekf_init(&e);
    ins_ekf_set_origin(&e, 37.4f, -122.1f, 0.0f);

    memset(&gnss, 0, sizeof(gnss));
    gnss.lat_deg = 37.4f;
    gnss.lon_deg = -122.1f;
    gnss.alt_m = 0.0f;
    gnss.fix = 1;

    for (i = 0; i < 501u; i++) {
        still_imu(&imu, i * 10000u);
        ins_ekf_imu(&e, &imu);
        if (i > 0u && (i % 100u) == 0u) {
            gnss.t_us = imu.t_us;
            ins_ekf_gnss(&e, &gnss);
        }
    }

    pnorm = sqrtf(e.mech.p_ned[0] * e.mech.p_ned[0]
                  + e.mech.p_ned[1] * e.mech.p_ned[1]
                  + e.mech.p_ned[2] * e.mech.p_ned[2]);
    printf("aided  |p|=%.3e P_nn=%.3f (5 s still + 1 Hz GNSS)\n",
           (double)pnorm, (double)e.P[6][6]);

    if (pnorm > 1.0f) {
        return fail("aided still run diverged in position");
    }
    if (e.P[6][6] > 25.0f) {
        return fail("aided P_nn grew past the frozen init");
    }
    return 0;
}

int main(void)
{
    if (test_open_loop_grows_p() != 0) {
        return 1;
    }
    if (test_gnss_pulls_north() != 0) {
        return 1;
    }
    if (test_still_aided_bounded() != 0) {
        return 1;
    }
    puts("test_ekf: ok");
    return 0;
}
