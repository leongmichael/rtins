#include "ins_ekf.h"

#include "ins_math.h"

#include <stddef.h>

#define N INS_EKF_N
#define RE_M 6378137.0f

static void mat_zero(float a[N][N])
{
    int i, j;

    for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
            a[i][j] = 0.0f;
        }
    }
}

static void mat_ident(float a[N][N])
{
    int i;

    mat_zero(a);
    for (i = 0; i < N; i++) {
        a[i][i] = 1.0f;
    }
}

static void mat_mul(const float a[N][N], const float b[N][N], float c[N][N])
{
    int i, j, k;

    for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
            float s = 0.0f;
            for (k = 0; k < N; k++) {
                s += a[i][k] * b[k][j];
            }
            c[i][j] = s;
        }
    }
}

static void mat_mul_bt(const float a[N][N], const float b[N][N], float c[N][N])
{
    int i, j, k;

    for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
            float s = 0.0f;
            for (k = 0; k < N; k++) {
                s += a[i][k] * b[j][k];
            }
            c[i][j] = s;
        }
    }
}

static void mat_symmetrize(float a[N][N])
{
    int i, j;

    for (i = 0; i < N; i++) {
        for (j = i + 1; j < N; j++) {
            float s = 0.5f * (a[i][j] + a[j][i]);
            a[i][j] = s;
            a[j][i] = s;
        }
    }
}

static void skew3(const float v[3], float S[3][3])
{
    S[0][0] = 0.0f;
    S[0][1] = -v[2];
    S[0][2] = v[1];
    S[1][0] = v[2];
    S[1][1] = 0.0f;
    S[1][2] = -v[0];
    S[2][0] = -v[1];
    S[2][1] = v[0];
    S[2][2] = 0.0f;
}

static int inv3(const float a[3][3], float b[3][3])
{
    float det;
    float c00, c01, c02, c10, c11, c12, c20, c21, c22;

    c00 = a[1][1] * a[2][2] - a[1][2] * a[2][1];
    c01 = a[1][2] * a[2][0] - a[1][0] * a[2][2];
    c02 = a[1][0] * a[2][1] - a[1][1] * a[2][0];
    c10 = a[0][2] * a[2][1] - a[0][1] * a[2][2];
    c11 = a[0][0] * a[2][2] - a[0][2] * a[2][0];
    c12 = a[0][1] * a[2][0] - a[0][0] * a[2][1];
    c20 = a[0][1] * a[1][2] - a[0][2] * a[1][1];
    c21 = a[0][2] * a[1][0] - a[0][0] * a[1][2];
    c22 = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    det = a[0][0] * c00 + a[0][1] * c01 + a[0][2] * c02;
    if (ins_fabsf(det) < 1.0e-12f) {
        return -1;
    }
    det = 1.0f / det;
    b[0][0] = c00 * det;
    b[0][1] = c10 * det;
    b[0][2] = c20 * det;
    b[1][0] = c01 * det;
    b[1][1] = c11 * det;
    b[1][2] = c21 * det;
    b[2][0] = c02 * det;
    b[2][1] = c12 * det;
    b[2][2] = c22 * det;
    return 0;
}

static void lla_to_ned(const ins_ekf_t *e, float lat_deg, float lon_deg,
                       float alt_m, float ned[3])
{
    float dlat = (lat_deg - e->lat0_deg) * (INS_PI / 180.0f);
    float dlon = (lon_deg - e->lon0_deg) * (INS_PI / 180.0f);
    float lat0 = e->lat0_deg * (INS_PI / 180.0f);

    ned[0] = dlat * RE_M;
    ned[1] = dlon * RE_M * ins_cosf(lat0);
    ned[2] = -(alt_m - e->alt0_m);
}

static void inject(ins_ekf_t *e, const float x[N])
{
    float dq[4], qn[4];
    int i;

    dq[0] = 1.0f;
    dq[1] = -0.5f * x[0];
    dq[2] = -0.5f * x[1];
    dq[3] = -0.5f * x[2];
    ins_quat_normalize(dq);
    ins_quat_mul(dq, e->mech.q_nb, qn);
    e->mech.q_nb[0] = qn[0];
    e->mech.q_nb[1] = qn[1];
    e->mech.q_nb[2] = qn[2];
    e->mech.q_nb[3] = qn[3];
    ins_quat_normalize(e->mech.q_nb);

    for (i = 0; i < 3; i++) {
        e->mech.v_ned[i] += x[3 + i];
        e->mech.p_ned[i] += x[6 + i];
        e->bg[i] += x[9 + i];
        e->ba[i] += x[12 + i];
    }
}

void ins_ekf_init(ins_ekf_t *e)
{
    int i;

    ins_mech_init(&e->mech);
    for (i = 0; i < 3; i++) {
        e->bg[i] = 0.0f;
        e->ba[i] = 0.0f;
    }
    mat_zero(e->P);
    for (i = 0; i < 3; i++) {
        e->P[i][i] = (5.0f * INS_PI / 180.0f) * (5.0f * INS_PI / 180.0f);
        e->P[3 + i][3 + i] = 1.0f;
        e->P[6 + i][6 + i] = 25.0f;
        e->P[9 + i][9 + i] = 1.0e-4f;
        e->P[12 + i][12 + i] = 1.0e-2f;
    }
    e->have_origin = 0;
    e->lat0_deg = 0.0f;
    e->lon0_deg = 0.0f;
    e->alt0_m = 0.0f;
}

void ins_ekf_set_origin(ins_ekf_t *e, float lat_deg, float lon_deg, float alt_m)
{
    e->lat0_deg = lat_deg;
    e->lon0_deg = lon_deg;
    e->alt0_m = alt_m;
    e->have_origin = 1;
}

void ins_ekf_imu(ins_ekf_t *e, const ins_imu_t *imu)
{
    ins_imu_t u;
    float dt;
    float C[3][3], fn[3], Sk[3][3];
    float F[N][N], Phi[N][N], tmp[N][N], Pp[N][N];
    int i, j;

    if (e == NULL || imu == NULL) {
        return;
    }

    u = *imu;
    for (i = 0; i < 3; i++) {
        u.gyr_rads[i] -= e->bg[i];
        u.acc_mps2[i] -= e->ba[i];
    }

    dt = ins_mech_imu(&e->mech, &u);
    if (dt <= 0.0f) {
        return;
    }

    ins_quat_to_Cnb(e->mech.q_nb, C);
    for (i = 0; i < 3; i++) {
        fn[i] = C[i][0] * u.acc_mps2[0]
              + C[i][1] * u.acc_mps2[1]
              + C[i][2] * u.acc_mps2[2];
    }
    skew3(fn, Sk);

    /* F: ψ̇ = -C b_g, δv̇ = [f^n×] ψ - C b_a, δṗ = δv */
    mat_zero(F);
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            F[i][9 + j] = -C[i][j];
            F[3 + i][j] = Sk[i][j];
            F[3 + i][12 + j] = -C[i][j];
        }
        F[6 + i][3 + i] = 1.0f;
    }

    mat_ident(Phi);
    for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
            Phi[i][j] += F[i][j] * dt;
        }
    }

    mat_mul(Phi, e->P, tmp);
    mat_mul_bt(tmp, Phi, Pp);
    for (i = 0; i < 3; i++) {
        Pp[i][i] += INS_EKF_SIGMA_G * INS_EKF_SIGMA_G * dt;
        Pp[3 + i][3 + i] += INS_EKF_SIGMA_A * INS_EKF_SIGMA_A * dt;
        Pp[9 + i][9 + i] += INS_EKF_SIGMA_BG * INS_EKF_SIGMA_BG * dt;
        Pp[12 + i][12 + i] += INS_EKF_SIGMA_BA * INS_EKF_SIGMA_BA * dt;
    }
    mat_symmetrize(Pp);
    for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
            e->P[i][j] = Pp[i][j];
        }
    }
}

void ins_ekf_gnss(ins_ekf_t *e, const ins_gnss_t *gnss)
{
    float z_ned[3], y[3];
    float S[3][3], Sinv[3][3];
    float K[N][3], x[N];
    float IKH[N][N], tmp[N][N], Pp[N][N];
    int i, j, k;

    if (e == NULL || gnss == NULL || gnss->fix == 0u) {
        return;
    }

    if (e->have_origin == 0u) {
        ins_ekf_set_origin(e, gnss->lat_deg, gnss->lon_deg, gnss->alt_m);
        return;
    }

    lla_to_ned(e, gnss->lat_deg, gnss->lon_deg, gnss->alt_m, z_ned);
    for (i = 0; i < 3; i++) {
        y[i] = z_ned[i] - e->mech.p_ned[i];
    }

    /* S = H P H^T + R, H = [0 0 I 0 0] → S = P_pp + R */
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            S[i][j] = e->P[6 + i][6 + j];
        }
        S[i][i] += INS_EKF_R_POS;
    }
    if (inv3(S, Sinv) != 0) {
        return;
    }

    /* K = P H^T S^{-1} ; H^T selects position columns */
    for (i = 0; i < N; i++) {
        for (j = 0; j < 3; j++) {
            float s = 0.0f;
            for (k = 0; k < 3; k++) {
                s += e->P[i][6 + k] * Sinv[k][j];
            }
            K[i][j] = s;
        }
    }

    for (i = 0; i < N; i++) {
        x[i] = K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];
    }

    mat_ident(IKH);
    for (i = 0; i < N; i++) {
        for (j = 0; j < 3; j++) {
            IKH[i][6 + j] -= K[i][j];
        }
    }

    mat_mul(IKH, e->P, tmp);
    mat_mul_bt(tmp, IKH, Pp);
    for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
            float s = 0.0f;
            for (k = 0; k < 3; k++) {
                s += K[i][k] * INS_EKF_R_POS * K[j][k];
            }
            Pp[i][j] += s;
        }
    }
    mat_symmetrize(Pp);
    for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
            e->P[i][j] = Pp[i][j];
        }
    }

    inject(e, x);
}
