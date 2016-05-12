/* Last modified: <26-Oct-2015 11:03:58 CET by Dmitry Ebel> */
#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include "error.h"
#include "types.h"
#include "beacon.h"
#include "sensor.h"

#include <libconfig.h>

typedef struct {
    double** clbr_fac;
    double** clbr_offset;
    uint8_t  num_measurements;
    double*** tof;
    double*** distances;
} calibrator_t;


error_t clb_calc_for_sbpair(const double const* tof_dist, const double const* real_dist,
			    const size_t len, double pres[2]);

error_t clb_read_config(const char*, config_t*, matrix_t**, matrix_t**);

//error_t clbr_calc(calibrator_t const*);
#endif
