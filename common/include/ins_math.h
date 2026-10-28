#ifndef INS_MATH_H
#define INS_MATH_H

#define INS_PI 3.14159265f

float ins_sqrtf(float x);
float ins_fabsf(float x);
float ins_cosf(float x_rad);

void ins_quat_normalize(float q[4]);
void ins_quat_mul(const float a[4], const float b[4], float r[4]);
void ins_quat_to_Cnb(const float q[4], float C[3][3]);

#endif /* INS_MATH_H */
