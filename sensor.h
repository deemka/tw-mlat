/* Last modified: <08-Dec-2015 16:40:12 CET by Dmitry Ebel> */
#ifndef SENSOR_H
#define SENSOR_H

#include "beacon.h"
#include "position.h"
#include "errors.h"
#include "types.h"
#include "kalman.h"
#include "signal.h"
#include "math_float.h"
#include <time.h>

#define SNS_ANY     (uint8_t)(~0)
#define SNS_USED    (uint8_t)(1<<0)
#define SNS_MLAT_OK (uint8_t)(1<<1)

#ifdef __cplusplus
extern "C" {
#endif

    /** A struct describing position of a sensor obtained from a set of beacons */
    typedef struct sensor_t {
	uint8_t      id;
	uint8_t      flags;
	uint64_t     prev_timestamp_us;
	uint64_t     timestamp_us;
	
	/** Coordinates on the vehicle according to the scheme below (example for 6 sensors).
	 * 
	 * s1-----s2-----s3 ^
	 * |              | |           y_vehicle 
	 * |      .O      | |100 cm    ^
	 * |              | |          |
	 * s6-----s5-----s4 V          +-----> x_vehicle
	 *  <------------>             O=(0,0) 
	 *      200 cm
	 * 
	 * Then s1.position_mnt.x=-100, s1.position_mnt.y=50 etc.
	 *
	 */
	position2d_t    position_mnt;

	/** Measured position translated to the vehicle CS, i.e. with the 
	 *  centroid as the CS origin and axis parallel to the world CS axes.
	 *  Keeps angle calculations simple.
	 */
	position2d_t    position_vcs;
	position2d_t    position_vcs_est;
	
	/** Position on the stage (world CS) obtained by multilateration */
	position3d_t    position_wcs;
	position3d_t    prev_position_wcs;
	//	double          orientation_wcs;
	
	/** Set of available beacons (not shared among other sensors) */
	beaconlist_t    beaconlist;

	kalman_filter_t kf_position_wcs;
	
	/** Distances to beacons */
	cm_t*           distances;
	
	/** Kalman filtered distances to beacons */
	kalman_filter_t*  kf_distances;

	/** Beacon status (maintained locally for each sensor) */
 	uint8_t*  b_flags;
	double*   b_score;
	double    mlat_score;

	/** Current raw signal to be processed */
	sample_t rawsig[BUFFER_LEN];
	} sensor_t;

    /** Sensor struct extended for the use in linked lists */
    typedef struct sensornode_t {
	sensor_t* sensor;
	struct sensornode_t* next;
    } sensornode_t;

    typedef sensornode_t* sensorlist_t;    

    /** Sensor constructor.
     * \arg id numeric ID
     * \arg x_mnt x coordinate of the mount point on the vehicle
     * \arg y_mnt y coordinate of the mount point on the vehicle
     * \arg z_mnt height above ground
     * \arg beaconlist set of available beacons
     * \return sensor
     */
    sensor_t      sns_new(const uint8_t id, const cm_t x_mnt, const cm_t y_mnt, const cm_t z_mnt);

    void          sns_free(sensor_t* const s);
    /** Adds sensor to the sensorlist */
    error_t       sns_add_to_list(sensornode_t** sensorlist, sensor_t* sensor);

    /** Sensor lookup by ID */
    sensornode_t* sns_get_byID(sensornode_t** sensorlist, const  uint8_t id);

    /** Calculates position of the sensor based on its distances to the beacons in 'beaconlist' 
     *  The position is then written to 'pos_world'.
     */
    error_t      sns_mlat2d(sensor_t* const s);

    /** Calculates and returns range measurement quality based on previous sensor position,
	beacon position and recently measured S-B distance */
    double       sns_calc_dist_score(sensor_t* s, const uint8_t bid);

    double       sns_calc_mlat_score(sensor_t* const s);
    
    void         sns_add_beaconlist(sensor_t* s, const beaconlist_t);
    uint8_t      sns_count(sensornode_t** const sensorlist, const uint8_t);
    uint8_t      sns_count_visible_beacons(const sensor_t* const s);
    void         sns_set_flag(sensor_t*, const uint8_t); 
    void         sns_unset_flag(sensor_t* s, const uint8_t f);
    cm_t         sns_get_distance(const sensor_t const* s, const uint8_t bid);
    cm_t         sns_get_filtered_distance(const sensor_t const* s, const uint8_t bid);
    position2d_t sns_get_filtered_position(sensor_t* s);
    double       sns_get_filtered_tof(const sensor_t const* s, const uint8_t bid);
    double       sns_get_tof(const sensor_t const* s, const uint8_t bid);
    error_t      sns_check_new_position(sensor_t* s);
    error_t      sns_update_kf(sensor_t* s);
    void         sns_update_timestamp(sensor_t* s, const uint64_t ts);
    error_t      sns_read_config(const char* fn, sensorlist_t* sl);
    cm_t         sns_pos2dist2d(const position2d_t* const p1, const position2d_t* const p2);
    cm_t         sns_pos2dist3d(const position3d_t* const p1, const position3d_t* const p2);

    double       sns_check_dist(sensor_t * const, const uint8_t, const cm_t, const uint64_t);
    void         sns_print(const sensor_t* const sensor);
    void         sns_list_sensors(sensornode_t** sensorlist);
    cm_t         sns_moved_cm(const sensor_t* const s);
#ifdef __cplusplus
}
#endif

#endif
