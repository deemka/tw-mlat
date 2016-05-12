/* Last modified: <2015-04-16 13:38:49 dmitry> */
#ifndef MATH_FIXED_H
#define MATH_FIXED_H
#include "signal.h"
#include <stdint.h>

#ifndef M_PI
#define M_PI    3.14159265358979323846
#endif
#ifndef M_TWOPI
#define M_TWOPI 6.28318530717958647692
#endif

typedef sample_t   fixed_t;
typedef sample_lt  fixed_lt;
typedef sample_ut  fixed_ut;
typedef sample_ult fixed_ult;

#ifdef __cplusplus
extern "C" {
#endif

    fixed_t  fx_sin(const fixed_t);
    fixed_t  fx_sinc(const fixed_t);
    fixed_ut fx_sqrt(const fixed_ult);
    fixed_t  fx_mean(const fixed_t*, const size_t);
#ifdef __cplusplus
}
#endif

#endif
