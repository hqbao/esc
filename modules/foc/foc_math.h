#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <math.h>

// Clarke transform: 3-phase (a,b,c) -> orthogonal (alpha, beta)
static inline void clarke(float a, float b, float c,
                           float *alpha, float *beta) {
    *alpha = (2.0f * a - b - c) / 3.0f;
    *beta  = (b - c) * 0.57735027f;  // 1/sqrt(3)
}

// Park transform: stationary (alpha, beta) -> rotating (d, q)
static inline void park(float alpha, float beta, float theta,
                         float *d, float *q) {
    float ct = cosf(theta);
    float st = sinf(theta);
    *d =  alpha * ct + beta * st;
    *q = -alpha * st + beta * ct;
}

// Inverse Park: rotating (d, q) -> stationary (alpha, beta)
static inline void inv_park(float d, float q, float theta,
                             float *alpha, float *beta) {
    float ct = cosf(theta);
    float st = sinf(theta);
    *alpha = d * ct - q * st;
    *beta  = d * st + q * ct;
}

#endif // FOC_MATH_H
