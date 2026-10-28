#include "ins_math.h"

#include <stdint.h>

float ins_sqrtf(float x)
{
    union {
        float f;
        uint32_t u;
    } v;
    float y;
    int i;

    if (x <= 0.0f) {
        return 0.0f;
    }

    v.f = x;
    v.u = (v.u >> 1) + 0x1fc00000u;
    y = v.f;
    for (i = 0; i < 3; i++) {
        y = 0.5f * (y + x / y);
    }
    return y;
}

float ins_fabsf(float x)
{
    return (x < 0.0f) ? -x : x;
}

float ins_cosf(float x)
{
    float x2, x4, x6;

    while (x > INS_PI) {
        x -= 2.0f * INS_PI;
    }
    while (x < -INS_PI) {
        x += 2.0f * INS_PI;
    }
    x2 = x * x;
    x4 = x2 * x2;
    x6 = x4 * x2;
    return 1.0f - x2 * (1.0f / 2.0f) + x4 * (1.0f / 24.0f) - x6 * (1.0f / 720.0f);
}

void ins_quat_normalize(float q[4])
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

void ins_quat_mul(const float a[4], const float b[4], float r[4])
{
    r[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    r[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    r[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    r[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void ins_quat_to_Cnb(const float q[4], float C[3][3])
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
