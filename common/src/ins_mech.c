#include "ins_mech.h"

#include "ins_math.h"

#include <stddef.h>

#define DT_MAX_S 0.2f

static void quat_normalize(float q[4])
{
    float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    float inv;
    int i;

    if (n2 <= 0.0f) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }
    inv = 1.0f / ins_sqrtf(n2);
    for (i = 0; i < 4; i++) {
        q[i] *= inv;
    }
    if (q[0] < 0.0f) {
        for (i = 0; i < 4; i++) {
            q[i] = -q[i];
        }
    }
}

/* r = a ⊗ b (Hamilton). */
static void quat_mul(const float a[4], const float b[4], float r[4])
{
    r[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    r[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    r[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    r[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

static void quat_to_Cnb(const float q[4], float C[3][3])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    C[0][0] = 1.0f - 2.0f * (y * y + z * z);
    C[0][1] = 2.0f * (x * y - z * w);
    C[0][2] = 2.0f * (x * z + y * w);
    C[1][0] = 2.0f * (x * y + z * w);
    C[1][1] = 1.0f - 2.0f * (x * x + z * z);
    C[1][2] = 2.0f * (y * z - x * w);
    C[2][0] = 2.0f * (x * z - y * w);
    C[2][1] = 2.0f * (y * z + x * w);
    C[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

void ins_mech_init(ins_mech_t *m)
{
    int i;

    m->q_nb[0] = 1.0f;
    m->q_nb[1] = 0.0f;
    m->q_nb[2] = 0.0f;
    m->q_nb[3] = 0.0f;
    for (i = 0; i < 3; i++) {
        m->v_ned[i] = 0.0f;
        m->p_ned[i] = 0.0f;
    }
    m->t_us = 0;
    m->have_sample = 0;
}

void ins_mech_imu(ins_mech_t *m, const ins_imu_t *imu)
{
    float dt;
    float theta[3];
    float dq[4], qn[4];
    float C[3][3];
    float f_n[3], a_n[3], v_new[3];
    uint32_t dt_us;
    int i;

    if (m == NULL || imu == NULL) {
        return;
    }

    if (m->have_sample == 0u) {
        m->t_us = imu->t_us;
        m->have_sample = 1;
        return;
    }

    dt_us = imu->t_us - m->t_us;
    m->t_us = imu->t_us;
    dt = (float)dt_us * 1.0e-6f;
    if (dt <= 0.0f || dt > DT_MAX_S) {
        return;
    }

    theta[0] = imu->gyr_rads[0] * dt;
    theta[1] = imu->gyr_rads[1] * dt;
    theta[2] = imu->gyr_rads[2] * dt;
    dq[0] = 1.0f;
    dq[1] = 0.5f * theta[0];
    dq[2] = 0.5f * theta[1];
    dq[3] = 0.5f * theta[2];
    quat_normalize(dq);
    quat_mul(m->q_nb, dq, qn);
    m->q_nb[0] = qn[0];
    m->q_nb[1] = qn[1];
    m->q_nb[2] = qn[2];
    m->q_nb[3] = qn[3];
    quat_normalize(m->q_nb);

    quat_to_Cnb(m->q_nb, C);
    for (i = 0; i < 3; i++) {
        f_n[i] = C[i][0] * imu->acc_mps2[0]
               + C[i][1] * imu->acc_mps2[1]
               + C[i][2] * imu->acc_mps2[2];
    }
    a_n[0] = f_n[0];
    a_n[1] = f_n[1];
    a_n[2] = f_n[2] + INS_G;

    for (i = 0; i < 3; i++) {
        v_new[i] = m->v_ned[i] + a_n[i] * dt;
        m->p_ned[i] += 0.5f * (m->v_ned[i] + v_new[i]) * dt;
        m->v_ned[i] = v_new[i];
    }
}
