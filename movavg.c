/* Last modified: <09-Oct-2015 12:53:09 CEST by Dmitry Ebel> */

#include "movavg.h"

#include <math.h>

void  mav_init(movavg_t* mc)
{
    mc->mean = 0;
    mc->num = 0;
    mc->var = 0;
}

void  mav_update_mean(movavg_t* mc, const double v)
{
    mc->prev_mean = mc->mean;
    mc->num += 1;
    mc->mean += (v-mc->mean)/mc->num;
    mc->var += (v - mc->mean)*(v-mc->prev_mean);
}

double  mav_get_mean(const movavg_t const * mc)
{
    return mc->mean;
}

double  mav_get_stddev(const movavg_t const * mc)
{
    return sqrt(mc->var/mc->num);
}

double mav_get_z_score(const movavg_t const * mc, const double v)
{
    return (v - mc->mean)/mav_get_stddev(mc);
}

