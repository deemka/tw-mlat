/* Last modified: <09-Dec-2015 10:53:59 CET by Dmitry Ebel> */
#ifndef VEHICLE_H
#define VEHICLE_H

#include "errors.h"
#include "types.h"
#include "sensor.h"
#include "beacon.h"
#include "position.h"
#include "planimetrics.h"
#include "kalman.h"

#define VEH_ANY    (uint8_t)(~0)
#define VEH_MOVING (uint8_t)(1<<0)

typedef struct {
    uint8_t      id;
    uint8_t      flags;    
    uint64_t     timestamp_us;
    uint64_t     prev_timestamp_us;
    sensorlist_t sensorlist;
    position2d_t position;
    position2d_t prev_position;  
    cm_t         vx;
    cm_t         vy;
    double       rot_speed;  /* [rad/s] */
    kalman_filter_t kf_pos;
    kalman_filter_t kf_rot;
} vehicle_t;

#ifdef __cplusplus
extern "C" {
#endif

    uint8_t      veh_is_moving(const vehicle_t* const);
    void         veh_set_moving(vehicle_t * const, const uint8_t);
    error_t      veh_init(vehicle_t * const, const uint8_t, sensorlist_t);
    void         veh_set_position(vehicle_t * const, const position2d_t * const);
    position2d_t veh_get_position(const vehicle_t * const);
    double       veh_get_rotation(const vehicle_t * const);
    error_t      veh_calc_rotation(vehicle_t * const);
    error_t      veh_calc_position(vehicle_t * const);
    error_t      veh_update_kfs(vehicle_t * const);
    error_t      veh_check_new_position(vehicle_t*);
    void         veh_update_timestamp(vehicle_t*, const uint64_t);
    
#ifdef __cplusplus
}
#endif

#endif
