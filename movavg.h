/* Last modified: <09-Oct-2015 12:55:09 CEST by Dmitry Ebel>
 */

#ifndef MOVAVG_H
#define MOVAVG_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct {
	double mean;
	double var;
	double prev_mean;
	size_t num;
    } movavg_t;

    void    mav_init(movavg_t*);
    void    mav_update_mean(movavg_t*, const double);
    double  mav_get_mean(const movavg_t const *);
    double  mav_get_stddev(const movavg_t const *);
    double  mav_get_z_score(const movavg_t const *, const double);

#ifdef __cplusplus
}
#endif

#endif
