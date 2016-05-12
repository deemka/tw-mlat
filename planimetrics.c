/* Last modified: <04-Dec-2015 12:27:02 CET by Dmitry Ebel> */
#include "planimetrics.h"
#include <math.h>
#include <stdlib.h>

double pln_get_angle(const cm_t const* u, const cm_t const* v, const size_t dim)
{
    double uu = 0;
    double vv = 0;
    double uv = 0;
    
    for (size_t i=0; i<dim; i++) {
	uu += u[i]*u[i];
	vv += v[i]*v[i];
	uv += u[i]*v[i];
    }

    return (acos(uv/sqrt(uu*vv)));
}


error_t pln_get_centroid(const sensorlist_t* const list, position2d_t* const pos)
{
    cm_t sumx = 0;
    cm_t sumy = 0;
    uint8_t cnt = 0;
    sensornode_t *s = *list;
    while (s != NULL) {
	if ( (s->sensor->flags & SNS_MLAT_OK) == SNS_MLAT_OK ) {
	    sumx += (s->sensor->position_wcs.x);
	    sumy += (s->sensor->position_wcs.y);
	    cnt++;
	}
	s = s->next;
    }
    
    if (cnt > 0) { 
	pos->x = sumx/cnt;
	pos->y = sumy/cnt;
	return ERR_NOERROR;
    } else {
	return ERR_NOTENOUGHDATA;
    }
}

cm_t pln_dist_L2(const position2d_t* const p1, const position2d_t* const p2)
{
    return sqrt( (p1->x-p2->x)*(p1->x-p2->x) + (p1->y-p2->y)*(p1->y-p2->y));
}

cm_t pln_dist_L2_2(const position2d_t* const p1, const position2d_t* const p2)
{
    return (p1->x-p2->x)*(p1->x-p2->x) + (p1->y-p2->y)*(p1->y-p2->y);
}

cm_t pln_dist_L1(const position2d_t* const p1, const position2d_t* const p2)
{
    return (cm_t)fabs((float)p1->x-(float)p2->x) + fabs((float)p1->y-(float)p2->y);
}


int pln_is_in_fov(position2d_t const * const viewer,
		  position2d_t const * const obj,
		  const double beam_angle)
{
    cm_t dx = obj->x - viewer->x;
    cm_t dy = obj->y - viewer->y;

    /* ! atan returns [-pi/2, pi/2] */
    double avo = atan(dy/dx);

    if (dx < 0) {
	avo += M_PI;
    } else {
	if (dy < 0) {
	    avo += M_PI;
	}
    }

    double lim1 = viewer->orientation + beam_angle/2;
    if (lim1 > M_TWOPI) {
	lim1 -= M_TWOPI;
    } else if (lim1 < 0) {
	lim1 += M_TWOPI;
    }
    double lim2 = viewer->orientation - beam_angle/2;
    if (lim2 > M_TWOPI) {
	lim2 -= M_TWOPI;
    } else if (lim2 < 0) {
	lim2 += M_TWOPI;
    }
    
    return (avo < lim1 && avo > lim2);
}

