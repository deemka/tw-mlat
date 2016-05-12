/* Last modified: <09-Oct-2015 12:46:49 CEST by Dmitry Ebel>  */
#ifndef MATH_FLOAT_H
#define MATH_FLOAT_H
#include "signal.h"
#include "errors.h"

#ifndef M_PI
#define M_PI    3.14159265358979323846
#endif
#ifndef M_TWOPI
#define M_TWOPI 6.28318530717958647692
#endif

#ifdef __cplusplus
extern "C" {
#endif

    /* Statistics on an array */
    double mean(const double* const in, const size_t len);
    double median(const double* const in, const size_t len);

    float sinc(const float x);
    float gauss(const float x, const float s);
    void  interpolate_parabola(const size_t in_x[3], const float in_y[3], float out[3]);
    float get_parabola_extremum_x(const float par_coeffs[3]);
#ifdef __cplusplus
}
#endif

#endif
