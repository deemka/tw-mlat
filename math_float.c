/* Last modified: <09-Oct-2015 12:48:55 CEST by Dmitry Ebel>  */
#include "math_float.h"
//#include "matrix.h"

#include <math.h>
#include <float.h>
#include <string.h>

float sinc(const float x)
{
    return (  (fabs(x) < FLT_EPSILON) ? 1.0 : sin(x)/x );
}

float gauss(const float x, const float s)
{
    return exp(-x*x/(2.0f*s*s))/(M_TWOPI*s*s);
}

/*void interpolate_parabola(const size_t in_x[3], const float in_y[3], float (out)[3])
{
    matrix_t* mtrx = mtr_alloc(3,3);
    matrix_t* inv  = mtr_alloc(3,3);
    matrix_t* mtry = mtr_alloc(3,1);
    matrix_t* res =  mtr_alloc(3,1);

    mtr_set_element(mtrx, 0, 0, in_x[0]*in_x[0]);
    mtr_set_element(mtrx, 0, 1, in_x[0]);
    mtr_set_element(mtrx, 0, 2, 1);
    mtr_set_element(mtrx, 1, 0, in_x[1]*in_x[1]);
    mtr_set_element(mtrx, 1, 1, in_x[1]);
    mtr_set_element(mtrx, 1, 2, 1);
    mtr_set_element(mtrx, 2, 0, in_x[2]*in_x[2]);
    mtr_set_element(mtrx, 2, 1, in_x[2]);
    mtr_set_element(mtrx, 2, 2, 1);

    mtr_inverse3x3(mtrx, inv);

    mtr_set_element(mtry, 0,0, in_y[0]);
    mtr_set_element(mtry, 1,0, in_y[1]);
    mtr_set_element(mtry, 2,0, in_y[2]);
    
    mtr_product(inv, mtry, res);

    out[0] = mtr_get_element(res, 0, 0);
    out[1] = mtr_get_element(res, 1, 0);
    out[2] = mtr_get_element(res, 2, 0);

    mtr_free(mtrx);
    mtr_free(inv);
    mtr_free(mtry);
    mtr_free(res);
    }*/

float get_parabola_extremum_x(const float par_coeffs[3])
{
    return -.5 * par_coeffs[1] / par_coeffs[0];
}

double mean(const double* const in, const size_t len)
{
    double res = 0;
    for (size_t i=0; i<len; i++) {
	res += *(in + i);
    }
    return res/(double)len;
}

static int compare_flt(const void* f1, const void* f2)
{
    return *((float*)f1) - *((float*)f2);
}

double median(const double* const in, const size_t len)
{
    double* copy = (double*)malloc(sizeof(double)*len);
    memcpy((void*)copy, (void*)in, sizeof(double)*len);
    qsort((void*)copy, len, sizeof(double), &compare_flt);
    double res = copy[len/2];
    free(copy);
    return res;
}
