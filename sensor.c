/* Last modified: <08-Dec-2015 16:40:08 CET by Dmitry Ebel> */
/* Fledermaus range sensor data structure and functions
 */ 

#include "sensor.h"
#include "matrix.h"
#include "planimetrics.h"

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <libconfig.h>

#define MAX_BEACON_ID 32

static int compar_dbl(const void* restrict v1, const void* restrict v2)
{
    /* reverse order! */
    if ( *((double*)v1) < *((double*)v2) ) {
	return 1;
    } else {
	return -1;
    }
    return 0;
}

sensor_t sns_new(const uint8_t id, const cm_t x_mnt, const cm_t y_mnt, const cm_t z_mnt)
{
    sensor_t res;
    res.id = id;
    res.flags = 0;
    res.position_mnt.x = x_mnt;
    res.position_mnt.y = y_mnt;
    kf_init(&(res.kf_position_wcs));
    res.distances = (cm_t*)calloc(sizeof(cm_t), MAX_BEACON_ID);
    res.b_flags   = (uint8_t*)calloc(sizeof(uint8_t), MAX_BEACON_ID);
    res.b_score = (double*)calloc(sizeof(double), MAX_BEACON_ID);
    res.position_wcs.x = 0;
    res.position_wcs.y = 0;
    res.position_wcs.z = z_mnt;

    return res;
}

void sns_add_beaconlist(sensor_t* s, const beaconlist_t bl)
{
    beaconnode_t* bn = bl;
    s->beaconlist = bl;

    uint8_t maxid = 0;
    while (bn != NULL) {
	if (maxid < bn->beacon->id) {
	    maxid = bn->beacon->id;
	}
	bn = bn->next;
    }
    s->kf_distances = (kalman_filter_t*)malloc(sizeof(kalman_filter_t)*(maxid+1));

    bn = bl;
    while (bn != NULL) {
	s->b_flags[bn->beacon->id] = bn->beacon->flags;	
	kf_init(s->kf_distances + bn->beacon->id);
	bn = bn->next;
    }
}

void sns_set_flag(sensor_t* s, const uint8_t f)
{
    s->flags |= f;
}

void sns_unset_flag(sensor_t* s, const uint8_t f)
{
    s->flags &= (uint8_t)(~f);
}

error_t sns_add_to_list(sensornode_t** sensorlist, sensor_t* s)
{
    error_t err = ERR_NOERROR;
    sensornode_t* newnode = (sensornode_t*)malloc(sizeof(sensornode_t));
    if (newnode == NULL) return ERR_MALLOC;
        
    newnode->sensor = (sensor_t*)malloc(sizeof(sensor_t));
    if (newnode->sensor == NULL) return ERR_MALLOC;
    
    /* copy data */
    newnode->sensor->id = s->id;
    newnode->sensor->timestamp_us = 0;
    newnode->sensor->prev_timestamp_us = 0;
    newnode->sensor->flags = s->flags;
    newnode->sensor->position_mnt.x = s->position_mnt.x;
    newnode->sensor->position_mnt.y = s->position_mnt.y;
    err |= kf_init(&(newnode->sensor->kf_position_wcs));
    newnode->sensor->distances = (cm_t*)calloc(sizeof(cm_t), MAX_BEACON_ID);
    newnode->sensor->b_flags = (uint8_t*)calloc(sizeof(uint8_t), MAX_BEACON_ID);
    newnode->sensor->b_score = (double*)calloc(sizeof(double), MAX_BEACON_ID);
    newnode->sensor->kf_distances = (kalman_filter_t*)malloc(sizeof(kalman_filter_t)*MAX_BEACON_ID);
    for (int i=0; i<MAX_BEACON_ID; i++) {
	err|= kf_init(newnode->sensor->kf_distances + i);
    }
    newnode->sensor->position_wcs.x = s->position_wcs.x;
    newnode->sensor->position_wcs.y = s->position_wcs.y;
    newnode->sensor->position_wcs.z = s->position_wcs.z;
    newnode->next = NULL;

    sensornode_t *curr;
    if (*sensorlist == NULL) {
	*sensorlist = (sensornode_t*)malloc(sizeof(sensornode_t));
	if (*sensorlist == NULL) return ERR_MALLOC;
	*sensorlist = newnode;
    } else {
	curr = *sensorlist;
	while (curr->next != NULL) {
	    curr = curr->next;
	}
	curr->next = newnode;
    }

    return err;
}

void sns_free(sensor_t* const s)
{
    free(s->distances);
}

uint8_t sns_count_visible_beacons(const sensor_t* const s)
{
    uint8_t res = 0;
    beaconnode_t* bn = s->beaconlist;
    
    while (bn != NULL) {
	if ( (s->b_flags[bn->beacon->id] & BCN_SIGNAL_OK)
	     && !(s->b_flags[bn->beacon->id] & BCN_IGNORED) ) {
	    res++;
	}
	bn = bn->next;
    }
    return res;
}

sensornode_t* sns_get_byID(sensornode_t** sensorlist, uint8_t const id)
{
    sensornode_t* currnode = *sensorlist;
    
    while (currnode != NULL) {
	if (currnode->sensor->id == id) {
	    return currnode;
	}
	currnode = currnode->next;
    }
    return NULL;
}

uint8_t sns_count(sensornode_t** const sensorlist, const uint8_t mask)
{
    uint8_t res = 0;
    sensornode_t* sn = *sensorlist;
    
    while (sn != NULL) {
	if (mask == SNS_ANY || ( (sn->sensor->flags & mask) == mask) ) {
	    res++;
	}
	sn = sn->next;
    }
    return res;
}

error_t sns_mlat2d(sensor_t* const s)
{
    uint8_t nbeacons = sns_count_visible_beacons(s);

    s->mlat_score = 0;

    if (nbeacons < 3) {
	s->flags &= ~SNS_MLAT_OK;
	return ERR_NOTENOUGHDATA;
    }

    double *q = (double*)calloc(sizeof(double), nbeacons);
    
    beaconnode_t* bnode = s->beaconlist;
    uint8_t i=0;
    while (bnode != NULL) {
	if (s->b_flags[bnode->beacon->id] & BCN_SIGNAL_OK ) {
	    q[i] = s->b_score[bnode->beacon->id];
	    i++;
	} 
	bnode = bnode->next;
    }

    qsort((void*)q, i, sizeof(double), &compar_dbl);

#if 1
    double q_crit = q[3];
#else
    double q_crit = .25;
#endif
    
    matrix_t* A  = mtr_alloc(nbeacons-1,2);
    matrix_t* b  = mtr_alloc(nbeacons-1,1);
    matrix_t* At = mtr_alloc(2, nbeacons-1);
    matrix_t* Pinv = mtr_alloc(2, 2);
    matrix_t* Ainv = mtr_alloc(2, nbeacons-1);
    matrix_t* P    = mtr_alloc(2, 2);
    matrix_t* res  = mtr_alloc(2, 1);
    
    double dummy, d, r;
    double xN=0, yN=0, rN=0;

    FILE* cf;
    char cfn[256];
    sprintf(cfn, "/tmp/mlat_s%d.txt", s->id);
    cf = fopen(cfn, "w");
    /* find the reference beacon */
    bnode = s->beaconlist;
    while (bnode != NULL) {
	if ( /*s->b_flags[bnode->beacon->id] & BCN_ISREF && */
	    !(s->b_flags[bnode->beacon->id] & BCN_IGNORED) 
	     && s->b_flags[bnode->beacon->id] & BCN_SIGNAL_OK
	    &&  (q[0] - s->b_score[bnode->beacon->id]) < 1e-6) {

	    s->b_flags[bnode->beacon->id] |= BCN_ISREF;
	    xN = (double)(bnode->beacon->position.x);
	    yN = (double)(bnode->beacon->position.y);

	    if (kf_is_steady(s->kf_distances + bnode->beacon->id)) {
		d = sns_get_filtered_distance(s, bnode->beacon->id);
		//d = sns_get_distance(s, bnode->beacon->id);
	    } else {
		d = sns_get_distance(s, bnode->beacon->id);
	    }
	    
	    double h = (double)bnode->beacon->position.z - s->position_wcs.z;
	    rN = sqrt(d*d - h*h);
	    fprintf(cf, "%.1f %.1f %.1f\n", xN, yN, rN);
	    break;
	}
	bnode = bnode->next;	
    }
    
    if (bnode == NULL) {
	return ERR_NODENOTFOUND;
    }
    
    i = 0;
    bnode = s->beaconlist;
    while (bnode != NULL) {
	if ( (s->b_flags[bnode->beacon->id] & BCN_SIGNAL_OK)
	     && !(s->b_flags[bnode->beacon->id] & BCN_ISREF)
	     && !(s->b_flags[bnode->beacon->id] & BCN_IGNORED)) {


	    if (s->b_score[bnode->beacon->id] >= q_crit) {

		mtr_set(A, i, 0, xN - (double)(bnode->beacon->position.x));
		mtr_set(A, i, 1, yN - (double)(bnode->beacon->position.y));
		
		if (kf_is_steady(s->kf_distances + bnode->beacon->id)) {
		    d = sns_get_filtered_distance(s, bnode->beacon->id);
		} else {
		    d = sns_get_distance(s, bnode->beacon->id);
		}
		
		double h = (double)bnode->beacon->position.z - s->position_wcs.z;
		r = sqrt(d*d - h*h);
		fprintf(cf, "%.1f %.1f %.1f\n", (float)bnode->beacon->position.x, (float)bnode->beacon->position.y, r);
		dummy =
		    xN*xN - ((double)bnode->beacon->position.x)*((double)bnode->beacon->position.x)
		    + yN*yN - ((double)bnode->beacon->position.y)*((double)bnode->beacon->position.y)
		    - rN*rN + r*r;
		mtr_set(b, i, 0, .5*dummy);
		i++;
	    } else {
		//printf ("%.4f < %.4f discarded \n", s->b_quality[bnode->beacon->id], q_crit);
	    }
	}
	
	bnode = bnode->next;
    }
    
    error_t err = ERR_NOERROR;
    err |= mtr_transpose(A, At);
    err |= mtr_product(At, A, P);
    
    err |= mtr_inverse2x2(P, Pinv);
    err |= mtr_product(Pinv, At, Ainv);
    err |= mtr_product(Ainv, b, res);

    if (err == ERR_NOERROR) {
	s->prev_position_wcs.x = s->position_wcs.x;
	s->prev_position_wcs.y = s->position_wcs.y;
	
	s->position_wcs.x = (cm_t)(mtr_get(res, 0, 0));
	s->position_wcs.y = (cm_t)(mtr_get(res, 1, 0));
	s->flags |= SNS_MLAT_OK;
	fprintf(cf, "# Sensor label:\n");
	fprintf(cf, "%f %f 1\n", (float)(s->position_wcs.x), (float)(s->position_wcs.y));
	fprintf(cf, "%f %f 3\n", (float)(s->position_wcs.x), (float)(s->position_wcs.y));
 	fprintf(cf, "%f %f 5\n", (float)(s->position_wcs.x), (float)(s->position_wcs.y));
	fprintf(cf, "%f %f 7\n", (float)(s->position_wcs.x), (float)(s->position_wcs.y));

	
	bnode = s->beaconlist;
	while (bnode != NULL) {
	    if ( (s->b_flags[bnode->beacon->id] & BCN_SIGNAL_OK)
		 && !(s->b_flags[bnode->beacon->id] & BCN_IGNORED)
		 && s->b_score[bnode->beacon->id] >= q_crit) {
		cm_t dexpect = sqrt (
				     (bnode->beacon->position.x - s->position_wcs.x)
				     *(bnode->beacon->position.x - s->position_wcs.x) +
				     (bnode->beacon->position.y - s->position_wcs.y)
				     *(bnode->beacon->position.y - s->position_wcs.y) +
				     (bnode->beacon->position.z - s->position_wcs.z)
				     *(bnode->beacon->position.z - s->position_wcs.z));
		
		if (kf_is_steady(s->kf_distances + bnode->beacon->id)) {
		    d = sns_get_filtered_distance(s, bnode->beacon->id);
		} else {
		    d = sns_get_distance(s, bnode->beacon->id);
		}
		s->mlat_score += fabs(dexpect - d);
	    }
	    bnode = bnode->next;
	}

	s->mlat_score = 1./s->mlat_score;

    } else {
	s->flags &= ~SNS_MLAT_OK;
    }

    fclose(cf);
    bnode = s->beaconlist;
    while (bnode != NULL) {
	s->b_flags[bnode->beacon->id] &= ~BCN_ISREF;
	bnode = bnode->next;
    }

    
    
    mtr_free(A);
    mtr_free(b);
    mtr_free(At);
    mtr_free(Pinv);
    mtr_free(P);
    mtr_free(Ainv);
    mtr_free(res);
    free(q);
    return err;
}

cm_t sns_get_distance(const sensor_t const* s, const uint8_t bid) {
    return s->distances[bid];
}

cm_t sns_get_filtered_distance(const sensor_t const* s, const uint8_t bid) {
    return mtr_get(s->kf_distances[bid].x, 0, 0);
}

double sns_get_tof(const sensor_t const* s, const uint8_t bid) {
    return s->distances[bid]/SPEED_OF_SOUND;
}

double sns_get_filtered_tof(const sensor_t const* s, const uint8_t bid) {
    return mtr_get(s->kf_distances[bid].x, 1, 0);
}

position2d_t sns_get_filtered_position(sensor_t* s) {
    position2d_t res;
    res.x = mtr_get(s->kf_position_wcs.x, 0, 0);
    res.y = mtr_get(s->kf_position_wcs.x, 1, 0);
    return res;
}

error_t sns_check_new_position(sensor_t* s)
{
    cm_t pos_change_cm = sqrt ( (s->position_wcs.x - s->prev_position_wcs.x)*(s->position_wcs.x - s->prev_position_wcs.x) + 
				(s->position_wcs.y - s->prev_position_wcs.y)*(s->position_wcs.y - s->prev_position_wcs.y) );
    double dt_s = 1e-6*(s->timestamp_us - s->prev_timestamp_us);

    /* check if timestamp difference is reasonable */
    dt_s = (dt_s > 0 && dt_s < 10.0) ? dt_s : 0;
    cm_t max_change_cm = 2*VEH_MAX_SPEED_CMPS*dt_s;
    
    if (pos_change_cm > max_change_cm &&
	s->prev_position_wcs.x > 0 &&
	s->prev_position_wcs.y > 0) {
	/* outlier -> dismiss calculated position, keep previous position */
	s->position_wcs.x = s->prev_position_wcs.x;
	s->position_wcs.y = s->prev_position_wcs.y;
	return ERR_NOERROR +1;
    } else {
	/* position is reasonable (no outlier) */
	s->prev_position_wcs.x = s->position_wcs.x;
	s->prev_position_wcs.y = s->position_wcs.y;
	return ERR_NOERROR;
    }
}

error_t sns_update_kf(sensor_t* s) {
    kf_state_t current_kf_state;
    error_t err = ERR_NOERROR;
    current_kf_state.x = s->position_wcs.x;
    current_kf_state.y = s->position_wcs.y;
    current_kf_state.vx = 0;
    current_kf_state.vy = 0;
    err = kf_update(&(s->kf_position_wcs), &current_kf_state);
    return err;
}

void sns_update_timestamp(sensor_t* s, const uint64_t ts)
{
    s->prev_timestamp_us = s->timestamp_us;
    s->timestamp_us = ts;
}

error_t sns_read_config(const char* fn, sensorlist_t* sl)
{
    config_t cfg;
    config_setting_t *cfg_sensor, *cfg_sensors, *cfg_mntpos;
    int sid, num_sensors, sns_cnt, used;
    double sx, sy, sz;
    sensor_t s;
    
    config_init(&cfg);
    if(! config_read_file(&cfg, fn)) {
	fprintf(stderr, "%s:%d - %s\n", config_error_file(&cfg),
		config_error_line(&cfg), config_error_text(&cfg));
	config_destroy(&cfg);
	return(1);
    }
    
    cfg_sensors = config_lookup(&cfg, "vehicle.sensors");

    num_sensors  = config_setting_length(cfg_sensors);
    for(sns_cnt = 0; sns_cnt < num_sensors; sns_cnt++) {
	cfg_sensor = config_setting_get_elem(cfg_sensors, sns_cnt);
	config_setting_lookup_int(cfg_sensor, "id", &sid);
	config_setting_lookup_int(cfg_sensor, "used", &used);
	if (used) {
#if LIBCONFIG_VER_MAJOR >=1 && LIBCONFIG_VER_MINOR>=5
	    cfg_mntpos = config_setting_lookup(cfg_sensor, "mount");
#else
	    cfg_mntpos = config_lookup_from(cfg_sensor, "mount"); 
#endif
	    config_setting_lookup_float(cfg_mntpos, "x", &sx);
	    config_setting_lookup_float(cfg_mntpos, "y", &sy);
	    config_setting_lookup_float(cfg_mntpos, "z", &sz);
	    s = sns_new(sid, sx, sy, sz);
	    sns_add_to_list(sl, &s);
	}
    }
    return ERR_NOERROR;
}

cm_t sns_pos2dist2d(const position2d_t* const p1, const position2d_t* const p2)
{
    return sqrt(   (p1->x - p2->x)*(p1->x - p2->x)
		 + (p1->y - p2->y)*(p1->y - p2->y));
}

cm_t sns_pos2dist3d(const position3d_t* const p1, const position3d_t* const p2)
{
    return sqrt(   (p1->x - p2->x)*(p1->x - p2->x)
		 + (p1->y - p2->y)*(p1->y - p2->y)
		 + (p1->z - p2->z)*(p1->z - p2->z));
}

void sns_print(const sensor_t* const sensor)
{
    printf("SID=%2d x=%-6.1f y=%-6.1f z=%-6.1f\n", sensor->id,
	   (float)sensor->position_mnt.x, (float)sensor->position_mnt.y, (float)sensor->position_wcs.z);
}

void sns_list_sensors(sensornode_t** sensorlist)
{
    sensornode_t* sn = *sensorlist;
    
    printf("Sensors:\n");
    while (sn != NULL) {
	sns_print(sn->sensor);
	sn = sn->next;
    }
    printf("\n");
}

double sns_calc_dist_score(sensor_t * s, const uint8_t bid)
{
    beaconnode_t *bn = bcn_get_byID(&(s->beaconlist), bid);
    cm_t dexpect = sns_pos2dist3d(&(bn->beacon->position), &(s->position_wcs));
    s->b_score[bid] = 1. / ( 1.0*fabs(dexpect - (s->distances)[bid])
	     + 0.0 * fabs(sns_get_filtered_distance(s, bid) - (s->distances)[bid]));

    return s->b_score[bid];
}
	
double sns_check_dist(sensor_t* const s, const uint8_t bid,
		      const cm_t dist_cm, const uint64_t ts_us)
{
    cm_t dist_change_cm = fabs( (double)dist_cm - s->distances[bid]);
    cm_t max_change_cm = VEH_MAX_SPEED_CMPS * ( ts_us - s->timestamp_us ) * 1e-6;
    
    if ( dist_change_cm > max_change_cm ) {
	if ( s->distances[bid] > 10 ) {
	    /* Displacement is too large and this is not the first measurement -> discard */
	    s->b_score[bid] = -999.;
	    s->b_flags[bid] &= ~BCN_SIGNAL_OK;
	} else {
	    /* Previous dist was unknown (was 0), so displacement is just numerical */
	    s->b_score[bid] = sns_calc_dist_score(s, bid);
	}
    } else {
	s->b_score[bid] = sns_calc_dist_score(s, bid);
    }
    
    return s->b_score[bid];
}

cm_t sns_moved_cm(const sensor_t* const s)
{
    return sqrt(   (s->position_wcs.x - s->prev_position_wcs.x)
		 * (s->position_wcs.x - s->prev_position_wcs.x)
		 + (s->position_wcs.y - s->prev_position_wcs.y)
		 * (s->position_wcs.y - s->prev_position_wcs.y));
}

double sns_calc_mlat_score(sensor_t* const s)
{
    return s->mlat_score;
}
