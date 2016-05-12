/* Last modified: <2015-08-27 10:10:10 dmitry> */
#include "solve.h"
#include "matrix.h"
#include <math.h>

#ifdef DEBUG
#include <stdio.h>
#endif

static volatile error_t err = ERR_NOERROR;

error_t qsolve (const double a, const double b, const double c, double res[2])
{
    double d = b*b - 4*a*c;
    if (d<0) return ERR_NOTANUMBER;
    
    res[0] = (-b+sqrt(d))/(2*a);
    res[1] = (-b-sqrt(d))/(2*a);
    return ERR_NOERROR;
}
