/* Last modified: <2015-04-15 14:37:25 dmitry> */
#include "math_fixed.h"
static volatile fixed_lt sum;
static volatile fixed_lt res;

/* Based on the Taylor series of sin(x):
 *
 *                          2n+1         3     5     7
 *           _MAXPOW   n   x            x     x     x
 * sin(x) ~ \      (-1)  ------  = x - --- + --- - --- + ...
 *          /_           (2n+1)!        3!    5!    7!
 *          n=0  
 */
fixed_t fx_sin(const fixed_t x)
{
    res  = 0;
    fixed_t xpwr = x;
    int8_t  sign = 1;
    fixed_t fct  = 1;
    const int8_t MAXPOW = 7;
    int8_t cnt;
    
    for (cnt=1; cnt<MAXPOW; cnt++) {
	res += sign * xpwr/fct;
	xpwr *= x;
	xpwr *= x;
	fct *= (cnt+1);
	fct *= (cnt+2);
	sign *= -1;
    }

    return res;
}

fixed_t fx_sinc(const fixed_t x)
{
    return ( (x==0) ? 1 : fx_sin(x)/x );
}

/* integer-valued sqrt() using binary search (bisection) */
fixed_ut fx_sqrt(const fixed_ult x)
{
    if (x == 0) return 0;
    
    fixed_ut hi = x;
    fixed_ut lo = 0;
    fixed_ut re = (hi+lo)/2;
    fixed_ult res2 = (fixed_ult)res*(fixed_ult)res;
    while( lo < hi-1 && res2 != x ) {
	if( res2 < x ) {
	    lo = re;
	} else {
	    hi = re;
	}
	re = (hi+lo)/2;
	res2 = (fixed_ult)re*(fixed_ult)re;
    }
    return re;
}

fixed_t fx_mean(const fixed_t* buf, const size_t len) {
    sum = 0;
    size_t i;
    for (i=0; i<len; i++) {
	sum += buf[i];
    }
    
    res = (fixed_t)(sum/len);
    
    return res;
}
