/* Last modified: <09-Dec-2015 10:52:54 CET by Dmitry Ebel> */
#include "messaging.h"
#include "serial.h"
#include "errors.h"
#include "signal.h"
#include "math_float.h"
#include "math_fixed.h"
#include "position.h"
#include "beacon.h"
#include "sensor.h"
#include "kalman.h"
#include "planimetrics.h"
#include "vehicle.h"
#include "calibrator.h"
#include "linreg.h"
#include "movavg.h"
#include "pack.h"
#include "stage.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include <libconfig.h>
#include <sys/time.h>

#define  TEST_SNS_ID     1
#define  LAST_BID        0
#define  DO_MLAT        (1 || (sid == TEST_SNS_ID))
#define  CALC_POSITION  (0 || (bid == LAST_BID))
#define  MAKE_DISTMATRIX 0
#define  LOG_DATA        1
#define  DLL_ENABLED     1 /* message delay lock for TOF correction */

uint8_t tof_flags = (TOF_CORRECT_PHASE | TOF_APPLY_RECT_WINDOW) & ~TOF_REMOVE_DC; //TODO: read from config

void *beep(void* arg) {
    int* f = arg;
    char cmd[256];
    sprintf(&cmd[0], "beep -f %d", *f);
    system(&cmd[0]);
    return NULL;
}

double inv(const double x) {return -x;}

int main(int argc, char** argv)
{
    matrix_t       *c0;             /* calibration, constant terms */
    matrix_t       *c1;             /* calibration, linear terms */
    config_t       cfg;             /* theater config */
    error_t        err;             /* return value of many functions */
    message_t      msg;             /* current message from the sampler */
    uint8_t        bid = 255;       /* current beacon ID */
    uint8_t        sid = 255;       /* current sensor ID */
    FILE           *of;             /* file to write to in verbose mode */
    char           fn[32];          /* file name */
    cm_t           dist_cm;         /* current range measurement result */
    sample_t       *rawsig = NULL;  /* raw data */
    sig_filter_t   sfilter;         /* simple fft-based bandpass */
    beaconlist_t   beaconlist = NULL;
    sensorlist_t   sensorlist = NULL;
    beaconnode_t*  bn = NULL;
    sensornode_t*  sn = NULL;
    linreg_t       lr[8][8];         /* linear regression calculator */

    stage_t        stage;
    vehicle_t      vehicle;          /* Fledermaus */
    float          timeofflight;     /* time of flight estimation */
    kf_state_t     curr_kf_state;    /* struct to pass data to the Kalman filter */
    int		   fd;               /* file descriptor of the serial port */
    struct timeval mytime;
    struct timeval myprevtime;
    uint64_t	   ts_us;            /* timestamp of the current message */
    double	   t_correction = 0; /* delay-locked loop correction */
    //double	   dt_s = 0;
    double         mydt_s = 0;       /* local time passed since last known position */
    uint8_t        mydt_divisor = 1; /* */
    double         mydt_est_s = 0;   /* local time passed since last known position */
    movavg_t       mav_dt;           /* moving average calculator for time intervals */
    int		   firstrun = 1;    
    uint32_t       ts = 0, pts = 0;

    err = clb_read_config("calibration.conf", &cfg, &c1, &c0);
    if (err != ERR_NOERROR) {
	fprintf(stderr, "Error reading calibration data\n");
	exit (err);
    }
    
    rawsig = (sample_t*)malloc(BUFFER_LEN*sizeof(sample_t)); //TODO: INFO msg/config?

    sig_filter_init(&sfilter);
    
    bcn_read_config("theater.conf", &beaconlist);

    bn = beaconlist;
    bn->beacon->flags |= BCN_ISREF;
    FILE *bf = fopen("/tmp/beacons.txt", "w");
    bn = beaconlist;
    while (bn != NULL) {
	fprintf(bf, "%d %d %d %d\n",
		(int)bn->beacon->position.x,
		(int)bn->beacon->position.y,
		(int)bn->beacon->position.z,
		bn->beacon->id);
	bn = bn->next;
    }
    fclose(bf);
	   
    sns_read_config("theater.conf", &sensorlist);

    sn = sensorlist;
    while (sn != NULL) {
	sns_add_beaconlist(sn->sensor, beaconlist);
	sn = sn->next;
    }
    
    uint8_t nbeacons = bcn_count(&beaconlist, BCN_ANY); 
    
    for (int i=0; i<8; i++) {
	for (int j=0; j<8;j++) {
	    lr_init(&(lr[i][j]));
	}
    }
    
    veh_init(&vehicle, 0, sensorlist);

    err = stg_init_from_config(&stage, "theater.conf");
    stage.vehicle = &vehicle; //TODO: read from config
    printf("%f\n", stage.width);
    
    printf ("%d\n", stg_in_mutual_fov(vehicle.sensorlist->sensor, beaconlist->beacon));
    
#if LOG_DATA
    sn = sensorlist;
    while (sn != NULL) {

	sprintf(&fn[0], "/tmp/distances_s%d.txt", sn->sensor->id);
	of = fopen(&fn[0], "w");
	fprintf(of, "\n");
	fclose(of);

	sprintf(&fn[0], "/tmp/distances_kf_s%d.txt", sn->sensor->id);
	of = fopen(&fn[0], "w");
	fprintf(of, "\n");
	fclose(of);

	sprintf(&fn[0], "/tmp/position_s%d.txt", sn->sensor->id);
	of = fopen(&fn[0], "w");
	fprintf(of, "\n");
	fclose(of);

	sprintf(&fn[0], "/tmp/mlat_s%d.txt", sn->sensor->id);
	of = fopen(&fn[0], "w");
	fprintf(of, "\n");
	fclose(of);

	sprintf(&fn[0], "/tmp/position_kf_s%d.txt", sn->sensor->id);
	of = fopen(&fn[0], "w");
	fprintf(of, "\n");
	fclose(of);

	of = fopen("/tmp/position_veh.txt", "w");
	fprintf(of, "\n");
	fclose(of);

	of = fopen("/tmp/position_veh_kf.txt", "w");
	fprintf(of, "\n");
	fclose(of);
	
	of = fopen("/tmp/rotation.txt", "w");
	fprintf(of, "\n");
	fclose(of);

	of = fopen("/tmp/rotation_kf.txt", "w");
	fprintf(of, "\n");
	fclose(of);
	sn = sn->next;
    }
#endif

    mav_init(&mav_dt);

    packed_data_t pd;
    uint8_t* data = (uint8_t*)malloc(sizeof(sample_t)*BUFFER_LEN);
    
    pd_init(&pd, BUFFER_LEN);
    pd.sample_counter = pd.buffer_len_samples;
    
    if (argc <2) {
	printf("No serial port name given\n");
	exit(1);
    }

    fd = sio_open_serial(argv[1]);

    if (fd == -1) {
	printf ("Error %d opening %s \n", fd, argv[1]);
	exit(1);
    }
    
    while (1) {
		    
	/**************************************************
	 * Signal acquisition 
	 **************************************************/
	err = msg_read_next_message(&msg, fd, data);
	if (err != ERR_NOERROR) {
	    printf ("Error while reading message: %s\n", err_lookup(err));
	    continue;
	}

	switch(msg.type) {
	case MSG_TYPE_CMP:
	    pd.data = data;   /* FIXME: pd.data allocated but not used */
	    pd_unpack_buffer(&pd, rawsig);
	    break;
	case MSG_TYPE_RAW:
	    memcpy(rawsig, data, BUFFER_LEN*sizeof(sample_t));
	}

	/************************************
	 * Timestamp processing 
	 ************************************/
	
	/* Since the ADC samples all inputs simultaneously, msg timestamp depends on
	   the beacon ID only. So, update the timestamp only if beacon id changed. */
	if (bid != msg.beacon_id ) {
	    bid = msg.beacon_id;
	    sid = msg.sensor_id;
	    myprevtime = mytime;
	    gettimeofday(&mytime, NULL);

	    /* adjust timestamp with a simple delay locked loop */
	    mydt_s = mytime.tv_sec - myprevtime.tv_sec + (mytime.tv_usec - myprevtime.tv_usec)*1e-6;
	    pts = ts;
	    ts = msg.timestamp;

	    mydt_est_s = mydt_s;
	    mydt_divisor = 1;
	    if (!firstrun && mydt_s > 1e-4 ) {
		if (mydt_s > .09 ) {
		    mydt_divisor = (int8_t)(.5 + mydt_s/mav_get_mean(&mav_dt));
		}
#if 1
		mydt_est_s = mydt_s/mydt_divisor;
#else
		mydt_est_s = mydt_s - (mydt_divisor-1)*mav_get_mean(&mav_dt);
#endif
		mav_update_mean(&mav_dt, mydt_est_s);
		t_correction = mav_get_mean(&mav_dt) - mydt_est_s;	
		printf("msg dts %d; local dts: %.6f -> %.6f (mean: %.6f, div: %d), corr: %d \n",
		       ts-pts, mydt_s, mydt_est_s, mav_get_mean(&mav_dt), mydt_divisor, (int)(t_correction*1e6));
	    }
	    
	}
	
	bid = msg.beacon_id;
	sid = msg.sensor_id;

	/*******************************************
	 * Signal processing (range measurement) *
	 ******************************************/
#if 1
	sig_filter_apply(&sfilter, &rawsig[0]);
#endif 

#if LOG_DATA
	sprintf(&fn[0], "/tmp/rawsig_b%ds%d.txt", bid, sid);
	of = fopen(&fn[0], "w");
	for (size_t i=0; i<BUFFER_LEN; i++) {
	    fprintf(of, "%d\n",rawsig[i]);
	}
	fclose(of);
#endif
	sn = sns_get_byID(&sensorlist, sid);
	if (sn == NULL) {
	    continue;
	}

	memcpy((void*)&(sn->sensor->rawsig)[0], (void*)rawsig, sizeof(sample_t)*BUFFER_LEN);
	
	bn = bcn_get_byID(&(sn->sensor->beaconlist), bid);
	if (bn == NULL) {
	    continue;
	}
    
	timeofflight  = sig_get_timeofflight(rawsig, BUFFER_LEN, tof_flags);
	if (timeofflight < 0) {
	    continue;
	}

	/* Delay lock */
	if (DLL_ENABLED) {
	    timeofflight += t_correction;
	}
	
	dist_cm = (cm_t)((mtr_get(c1, bid, sid)*((timeofflight)*SPEED_OF_SOUND) + mtr_get(c0,bid,sid)) * 100 );


	/******************************************
	 * Evaluating range measurement *
	 *****************************************/
	if (dist_cm < 0 || dist_cm > DISTMAX*100) {
	    continue;
	}

	ts_us = mytime.tv_sec*1000000U + mytime.tv_usec;

	double q = sns_check_dist(sn->sensor, bid, dist_cm, ts_us);

	if (q < 0.) {
	    continue;
	}

#if 1
	if ( lr[sid][bid].stddev > .1
	     && fabs(dist_cm - (lr[sid][bid]).data[lr[sid][bid].cnt]) > 3.*lr[sid][bid].stddev) {
	    printf("Outlier \n");
	    printf("b%d s%d using previous value \n", bid, sid);
	    dist_cm = sn->sensor->distances[bid];
	}
#endif
	
	sn->sensor->distances[bid] = dist_cm;

	lr_update_data(&(lr[sid][bid]), sn->sensor->distances[bid]);
	lr_calc(&(lr[sid][bid]));

	sn->sensor->b_score[bid] = 1./lr[sid][bid].stddev;
		
	uint8_t n_good = 0;
	if (sid == TEST_SNS_ID) {
	    for (uint8_t i=0; i<nbeacons; i++) {
		if (lr[sid][i].stddev < 5.0) {
		    n_good++;
		}
	    }
	}

	/************************************************
	 * Range Values OK, Kalman filter distances
	 ***********************************************/

	curr_kf_state.x = sn->sensor->distances[bid];
	curr_kf_state.y = timeofflight;
	curr_kf_state.vx = 0;
	curr_kf_state.vy = 0;
	err = kf_update(sn->sensor->kf_distances + bid, &curr_kf_state);
	sn->sensor->b_flags[bn->beacon->id] |= BCN_IN_FOV;  //TODO: check if in FOV
	sn->sensor->b_flags[bn->beacon->id] |= BCN_SIGNAL_OK;
	
#if LOG_DATA
	if (bn->next == NULL) {
	    sprintf(&fn[0], "/tmp/distances_s%d.txt", sn->sensor->id);
	    of = fopen(&fn[0], "a");
	    for (uint8_t i=0; i<nbeacons; i++) {
		if (bcn_get_byID(&beaconlist, i) != NULL)
		    fprintf(of, "%.1f ", (float)(sn->sensor->distances)[i]);
	    }
	    fprintf(of,"\n");
	    fclose(of);
	    
	    sprintf(&fn[0], "/tmp/distances_kf_s%d.txt", sn->sensor->id);
	    of = fopen(&fn[0], "a");
	    for (uint8_t i=0; i<nbeacons; i++) {
		if (bcn_get_byID(&beaconlist, i) != NULL)
		    fprintf(of, "%.1f ", mtr_get(sn->sensor->kf_distances[i].x,0,0));
	    }
	    fprintf(of,"\n");
	    fclose(of);
	}
#endif	

	/***************************************
	 * Multilateration, sensor positions 
	 ***************************************/
	if (DO_MLAT) {
	    err = sns_mlat2d(sn->sensor);
	    sns_calc_mlat_score(sn->sensor);

	    if (err != ERR_NOERROR) {
		sn->sensor->flags &= ~SNS_MLAT_OK;
	    } else {
		sn->sensor->flags |= SNS_MLAT_OK;
		//err = sns_check_new_position(sn->sensor);

#if LOG_DATA
		sprintf(&fn[0], "/tmp/position_s%d.txt", sn->sensor->id);
		of = fopen(&fn[0], "a");
		fprintf(of, "%.1f %.1f %d\n",
			(double)sn->sensor->position_wcs.x,
			(double)sn->sensor->position_wcs.y,
			sns_count_visible_beacons(sn->sensor));
		fclose(of);

		err = sns_update_kf(sn->sensor);

		sprintf(&fn[0], "/tmp/position_kf_s%d.txt", sn->sensor->id);
		of = fopen(&fn[0], "a");
		fprintf(of, "%.1f %.1f\n",
			mtr_get(sn->sensor->kf_position_wcs.x,0,0),
			mtr_get(sn->sensor->kf_position_wcs.x,1,0));
		fclose(of);
#endif
	    }
	}

	/************************************
	 * Sensor positions to vehicle position 
	 ***********************************/
	if (CALC_POSITION) {
	    veh_update_timestamp(&vehicle, ts_us);
	    err = veh_calc_position(&vehicle);
	    //err = veh_check_new_position(&vehicle);
	    if (firstrun) {
		firstrun = 0;
	    }
	    veh_update_kfs(&vehicle);
	    
#if LOG_DATA
	    //printf("Rotation: %.0f, %f\n", vehicle.rotation*180/M_PI, (float)(1e-6*(vehicle.timestamp_us - vehicle.prev_timestamp_us)));
	    sprintf(&fn[0], "/tmp/position_veh.txt");
	    of = fopen(&fn[0], "a");
	    fprintf(of, "%.1f %.1f\n", (veh_get_position(&vehicle)).x,  (veh_get_position(&vehicle)).y);
	    fclose(of);
	    sprintf(&fn[0], "/tmp/position_veh_kf.txt");
	    of = fopen(&fn[0], "a");
	    fprintf(of, "%.1f %.1f\n", mtr_get(vehicle.kf_pos.x, 0,0),  mtr_get(vehicle.kf_pos.x, 1,0));
	    fclose(of);
	    sprintf(&fn[0], "/tmp/rotation.txt");
	    of = fopen(&fn[0], "a");
	    fprintf(of, "%.1f\n", vehicle.position.orientation*180/M_PI);
	    fclose(of);
	    sprintf(&fn[0], "/tmp/rotation_kf.txt");
	    of = fopen(&fn[0], "a");
	    fprintf(of, "%.1f\n", mtr_get(vehicle.kf_rot.x, 0,0)*180/M_PI);
	    fclose(of);
	    /*	    sprintf(&fn[0], "/tmp/mlat.txt");
	    of = fopen(&fn[0], "w");
	    sensornode_t *stmp = sensorlist;
	    while (stmp != NULL) {
		fprintf(of, "%.1f %.1f 0.1\n", stmp->sensor->position_wcs.x, stmp->sensor->position_wcs.y);
		fprintf(of, "%.1f %.1f 1.0\n", stmp->sensor->position_wcs.x, stmp->sensor->position_wcs.y);
		fprintf(of, "%.1f %.1f 0.1\n", vehicle.position.x, vehicle.position.y);
		fprintf(of, "%.1f %.1f 1.0\n", vehicle.position.x, vehicle.position.y);
		fprintf(of, "%.1f %.1f 2.0\n", vehicle.position.x, vehicle.position.y);
		stmp = stmp->next;
	    }
	    fclose(of);
	    */
#endif
	}
    }

    bcn_remove_byID(&beaconlist, 0);
    bcn_remove_byID(&beaconlist, 1);
    bcn_remove_byID(&beaconlist, 2);
    bcn_remove_byID(&beaconlist, 3);
    bcn_remove_byID(&beaconlist, 4);
    bcn_remove_byID(&beaconlist, 5);
    
    free(rawsig);
    return 0;
}
