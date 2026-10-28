#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ins_mech.h"

static void euler_zyx(const float q[4], float *roll, float *pitch, float *yaw)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    *roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    *pitch = asinf(2.0f * (w * y - z * x));
    *yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

static int fail(const char *msg)
{
    fprintf(stderr, "test_mech: %s\n", msg);
    return 1;
}

static int test_still(void)
{
    ins_mech_t m;
    ins_imu_t imu;
    float roll, pitch, yaw;
    unsigned i;
    const unsigned n = 101; /* 1 s at 100 Hz including t=0 */

    ins_mech_init(&m);
    memset(&imu, 0, sizeof(imu));
    imu.acc_mps2[2] = -INS_G;

    for (i = 0; i < n; i++) {
        imu.t_us = i * 10000u;
        ins_mech_imu(&m, &imu);
    }

    euler_zyx(m.q_nb, &roll, &pitch, &yaw);
    printf("still  roll=%.4f deg pitch=%.4f yaw=%.4f |v|=%.4e |p|=%.4e\n",
           (double)(roll * 180.0f / (float)M_PI),
           (double)(pitch * 180.0f / (float)M_PI),
           (double)(yaw * 180.0f / (float)M_PI),
           (double)sqrtf(m.v_ned[0] * m.v_ned[0] + m.v_ned[1] * m.v_ned[1]
                         + m.v_ned[2] * m.v_ned[2]),
           (double)sqrtf(m.p_ned[0] * m.p_ned[0] + m.p_ned[1] * m.p_ned[1]
                         + m.p_ned[2] * m.p_ned[2]));

    if (fabsf(roll) > 1.0e-4f || fabsf(pitch) > 1.0e-4f) {
        return fail("still: attitude moved");
    }
    if (fabsf(m.v_ned[0]) > 1.0e-4f || fabsf(m.v_ned[1]) > 1.0e-4f
        || fabsf(m.v_ned[2]) > 1.0e-4f) {
        return fail("still: velocity walked");
    }
    if (fabsf(m.p_ned[0]) > 1.0e-4f || fabsf(m.p_ned[1]) > 1.0e-4f
        || fabsf(m.p_ned[2]) > 1.0e-4f) {
        return fail("still: position walked");
    }
    return 0;
}

static int test_yaw_rate(void)
{
    ins_mech_t m;
    ins_imu_t imu;
    float roll, pitch, yaw;
    unsigned i;
    const unsigned n = 101;
    const float wz = 0.1f; /* rad/s */
    float yaw_deg;
    float expect_deg;

    ins_mech_init(&m);
    memset(&imu, 0, sizeof(imu));
    imu.acc_mps2[2] = -INS_G;
    imu.gyr_rads[2] = wz;

    for (i = 0; i < n; i++) {
        imu.t_us = i * 10000u;
        ins_mech_imu(&m, &imu);
    }

    euler_zyx(m.q_nb, &roll, &pitch, &yaw);
    yaw_deg = yaw * 180.0f / (float)M_PI;
    expect_deg = wz * 1.0f * 180.0f / (float)M_PI; /* 1 s of integration */
    printf("yaw    roll=%.4f pitch=%.4f yaw=%.4f deg (expect %.4f)\n",
           (double)(roll * 180.0f / (float)M_PI),
           (double)(pitch * 180.0f / (float)M_PI),
           (double)yaw_deg, (double)expect_deg);

    if (fabsf(roll) > 0.05f * (float)M_PI / 180.0f
        || fabsf(pitch) > 0.05f * (float)M_PI / 180.0f) {
        return fail("yaw: roll/pitch not level");
    }
    if (fabsf(yaw_deg - expect_deg) > 0.2f) {
        return fail("yaw: heading off");
    }
    return 0;
}

int main(void)
{
    if (test_still() != 0) {
        return 1;
    }
    if (test_yaw_rate() != 0) {
        return 1;
    }
    puts("test_mech: ok");
    return 0;
}
