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
