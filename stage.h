/* Last modified: <09-Dec-2015 10:37:51 CET by Dmitry Ebel> */
#ifndef STAGE_H
#define STAGE_H

#include "errors.h"
#include "types.h"
#include "beacon.h"
#include "vehicle.h"
#include "planimetrics.h"

typedef struct {
    cm_t width;
    cm_t depth;
    beaconlist_t beaconlist;
    vehicle_t *vehicle;
} stage_t;

#ifdef __cplusplus
extern "C" {
#endif

    error_t stg_init_from_config(stage_t*, const char*);
    int     stg_in_mutual_fov(sensor_t const * const s, beacon_t const * const b);
    
#ifdef __cpulsplus
}
#endif

#endif
