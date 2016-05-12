/* Last modified: <04-Dec-2015 13:23:56 CET by Dmitry Ebel> */
#ifndef POSITION_H
#define POSITION_H

#include "errors.h"
#include "types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    /** 2d position */
    typedef struct {
	cm_t x;
	cm_t y;
	double orientation;
    } position2d_t;

    /** 3d position */
    typedef struct {
	cm_t x;
	cm_t y;
	cm_t z;
	double orientation_h;
	double orientation_v;
    } position3d_t;

    /** 2D multilateration, low-level data types */
    error_t pos_mlat2d(const position2d_t* const positions, /** array of beacon positions */
		       const cm_t* const distances,         /** array of distances to beacons */
		       const uint8_t count,                 /** array length */
		       position2d_t* const res);            /** resulting receiver position */
    
    /** 3D multilateration */
    error_t pos_mlat3d(const position3d_t* const positions, /** array of beacon positions */
		       const cm_t* const distances,         /** array of distances to beacons */
		       const uint8_t count,                 /** array length */
		       position3d_t* const res);            /** resulting receiver position */

#ifdef __cplusplus
}
#endif

#endif
