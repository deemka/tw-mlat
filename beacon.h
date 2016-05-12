/* Last modified: <09-Dec-2015 11:03:29 CET by Dmitry Ebel> */
/** Beacon (ultrasonar transmitter) data struct  */

#ifndef BEACON_H
#define BEACON_H

#include "errors.h"
#include "position.h"
#include "types.h"
#include <stdint.h>

/* beacon flags (1 byte) */
#define BCN_ANY       (uint8_t)(~0)
#define BCN_IN_FOV    (uint8_t)(1<<0)
#define BCN_SIGNAL_OK (uint8_t)(1<<1)
#define BCN_IGNORED   (uint8_t)(1<<2)
#define BCN_POSKNOWN  (uint8_t)(1<<3)
#define BCN_ISREF     (uint8_t)(1<<4)

    /** \brief Beacon (ultrasonar transmitter) data struct. */
    typedef struct beacon_t {
	uint8_t id; 	  /** Beacon ID [0..255] */
	uint8_t flags;     /** Beacon status (visible, position known, ignored, ...) */
	position3d_t position; /** Beacon position */
    } beacon_t;

    /** \brief Beacon as element of a beacon list. */
    typedef struct beaconnode_t {
	beacon_t            *beacon;
	struct beaconnode_t *next;
    } beaconnode_t;

    /** \brief Beacon list. */
    typedef beaconnode_t* beaconlist_t;

#ifdef __cplusplus
extern "C" {
#endif
    /** Beacon initialization if position is known 
     */
    beacon_t      bcn_init(const uint8_t id, const cm_t x, const cm_t y, const cm_t z);
    beacon_t      bcn_new(const uint8_t id);
    error_t       bcn_add_to_list(beaconnode_t** beaconlist, beacon_t* beacon);
    error_t       bcn_append(beaconnode_t** beaconlist, beacon_t* beacon);
    error_t       bcn_remove_byID(beaconnode_t** beaconlist, const uint8_t id);
    beaconnode_t* bcn_get_byID(beaconnode_t** beaconlist, const uint8_t id);
    uint8_t       bcn_count(const beaconlist_t* const, const uint8_t);
    void          bcn_list_beacons(beaconnode_t** beaconlist);
    void          bcn_print(const beacon_t* const beacon);
    error_t       bcn_set_distance_byID(beaconnode_t** beaconlist, const uint8_t id, const cm_t distance);
    cm_t          bcn_get_distance_byID(beaconnode_t** beaconlist, const uint8_t id);
    int8_t        bcn_is_visible(const beacon_t* const beacon);
    int8_t        bcn_is_ignored(const beacon_t const *beacon);
    error_t       bcn_ignore_byID(beaconnode_t** const beaconlist, const int8_t id);
    void          bcn_ignore(beacon_t* const beacon);
    error_t       bcn_read_config(const char* fn, beaconlist_t*);
#ifdef __cplusplus
}
#endif

#endif
