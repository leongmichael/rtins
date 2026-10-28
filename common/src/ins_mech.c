#include "ins_mech.h"

#include "ins_math.h"

#include <stddef.h>

#define DT_MAX_S 0.2f

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

float ins_mech_imu(ins_mech_t *m, const ins_imu_t *imu)
{
    float dt;
    float theta[3];
    float dq[4], qn[4];
    float C[3][3];
    float f_n[3], a_n[3], v_new[3];
    uint32_t dt_us;
    int i;

    if (m == NULL || imu == NULL) {
        return 0.0f;
    }

    if (m->have_sample == 0u) {
        m->t_us = imu->t_us;
        m->have_sample = 1;
        return 0.0f;
    }

    dt_us = imu->t_us - m->t_us;
    m->t_us = imu->t_us;
    dt = (float)dt_us * 1.0e-6f;
    if (dt <= 0.0f || dt > DT_MAX_S) {
        return 0.0f;
    }

    theta[0] = imu->gyr_rads[0] * dt;
    theta[1] = imu->gyr_rads[1] * dt;
    theta[2] = imu->gyr_rads[2] * dt;
    dq[0] = 1.0f;
    dq[1] = 0.5f * theta[0];
    dq[2] = 0.5f * theta[1];
    dq[3] = 0.5f * theta[2];
    ins_quat_normalize(dq);
    ins_quat_mul(m->q_nb, dq, qn);
    m->q_nb[0] = qn[0];
    m->q_nb[1] = qn[1];
    m->q_nb[2] = qn[2];
    m->q_nb[3] = qn[3];
    ins_quat_normalize(m->q_nb);

    ins_quat_to_Cnb(m->q_nb, C);
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
    return dt;
}
