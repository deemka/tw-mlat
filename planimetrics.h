/* Last modified: <03-Dec-2015 15:39:34 CET by Dmitry Ebel> */
#ifndef PLANIMETRICS_H
#define PLANIMETRICS_H

#include "position.h"
#include "sensor.h"
#include "errors.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif
    /** Calculates the centroid position of all given measurement points
     *  i.e. x_c = \sum_{i=1}^N_sensors (x_i)/N_sensors. It can be assumed that
     *  the centroid of the measured positions = real centroid because position
     *  noise is zero-mean and random and therefore is reduced by the factor
     *  ~sqrt(N_sensors) like in signal averaging. */
    error_t pln_get_centroid(const sensorlist_t* const, position2d_t* const);

    /** Euklidian distanse of two points */
    cm_t    pln_dist_L2(const position2d_t* const, const position2d_t* const);

    /** Squared Euklidian distanse of two points */
    cm_t    pln_dist_L2_2(const position2d_t* const, const position2d_t* const);

    /** Manhattan distanse of two points */
    cm_t    pln_dist_L1(const position2d_t* const, const position2d_t* const);

    /** Angle between two vectors of any dimensionality */
    double  pln_get_angle(const cm_t const* u, const cm_t const* v, const size_t dim);

    int     pln_is_in_fov(position2d_t const * const viewer,
			  position2d_t const * const obj,
			  const double beam_angle);

    
#ifdef __cplusplus
}
#endif

#endif
