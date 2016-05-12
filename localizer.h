#ifndef LOCALIZER_H
#define LOCALIZER_H

#include "errors.h"
#include "vehicle.h"
#include "beacon.h"
#include "sensor.h"
#include "position.h"

typedef struct s_localizer_t {
    vehicle_t *v;
    beaconlist_t **bl;
} localizer_t;

#ifdef __cplusplus
extern "C" {
#endif
    
    
    
#ifdef __cplusplus
}
#endif
    
#endif
