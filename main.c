/* Last modified: <26-Oct-2015 11:04:46 CET by Dmitry Ebel> */
#include "beacon.h"
#include "position.h"
#include "matrix.h"
#include "serial.h"
#include "errors.h"
#include "messaging.h"
#include "position.h"
#include "signal.h"
#include "math_float.h"
#include "math_fixed.h"
#include "sensor.h"
#include "kalman.h"
#include "planimetrics.h"
#include "vehicle.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define VEH_WTH 100.0
#define VEH_LEN 200.0
#define NOISE 2
#define DISPLACE (1*((rand()%(2*NOISE+1)-NOISE)))

int main3()
{
    beaconlist_t beaconlist = NULL;

    srand(time(NULL));

    /* diagonag grid/ chess pattern
       B        B         B         300 
            B        B         B    200
       B        B         B         100
            B        B         B    0
       0  100   200  300  400  500
    
       => mean error = 4.41 cm
    */
    /*
    beacon_t b0 = beacon_init(0,   100,  0, 300);
    beacon_t b1 = beacon_init(1,   300,  0, 300);
    beacon_t b2 = beacon_init(2,   500,  0, 300);
    beacon_t b3 = beacon_init(3,    0,  100, 300);
    beacon_t b4 = beacon_init(4,   200, 100, 300);
    beacon_t b5 = beacon_init(5,   400,  100, 300);
    beacon_t b6 = beacon_init(6,   100,  200, 300);
    beacon_t b7 = beacon_init(7,   300,  200, 300);
    beacon_t b8 = beacon_init(8,   500,  200, 300);
    beacon_t b9 = beacon_init(9,   0,  300, 300);
    beacon_t b10 = beacon_init(10, 200,  300, 300);
    beacon_t b11 = beacon_init(11, 400,  300, 300);
    */


    /* rectangular grid
       B     B     B     B   400
       B     B     B     B   200
       B     B     B     B   0
       0     200   400   600
    
       => mean error: b1 is ref: 3.2 cm 
                      b5 is ref: 2.6 cm
    */
    
    beacon_t b0 = bcn_init(0,   0,    0, 300);
    beacon_t b1 = bcn_init(1,   200,  0, 300);
    beacon_t b2 = bcn_init(2,   400,  0, 300);
    beacon_t b3 = bcn_init(3,   600,  0, 300);
    beacon_t b4 = bcn_init(4,   0,    200, 300);
    beacon_t b5 = bcn_init(5,   200,  200, 300);
    beacon_t b6 = bcn_init(6,   400,  200, 300);
    beacon_t b7 = bcn_init(7,   600,  200, 300);
    beacon_t b8 = bcn_init(8,   0,    400, 300);
    beacon_t b9 = bcn_init(9,   200,  400, 300);
    beacon_t b10 = bcn_init(10, 400,  400, 300);
    beacon_t b11 = bcn_init(11, 600,  400, 300);
    

    /* rectangle, OoI within the rectancle 
     
       B     B     B     B   300
       B                 B   200
       B                 B   100
       B     B     B     B   0
       0     200   400   600

       => mean error = 3.8 cm
    */
    /*beacon_t b0 = beacon_init(0, 0,    0, 300);
      beacon_t b1 = beacon_init(1, 200,  0, 300);
      beacon_t b2 = beacon_init(2, 400,  0, 300);
      beacon_t b3 = beacon_init(3, 600,  0, 300);
      beacon_t b4 = beacon_init(4, 600,  100, 300);
      beacon_t b5 = beacon_init(5, 600,  200, 300);
      beacon_t b6 = beacon_init(6, 600,  300, 300);
      beacon_t b7 = beacon_init(7, 400,  300, 300);
      beacon_t b8 = beacon_init(8, 200,  300, 300);
      beacon_t b9 = beacon_init(9, 100,  300, 300);
      beacon_t b10 = beacon_init(10, 0,  300, 300);
      beacon_t b11 = beacon_init(11, 0,  200, 300);
    */

    /* mostly straight line 
       => mean error = 11.5 cm
     
     */
    /*    beacon_t b0 = beacon_init(0, 0,    0, 300);
      beacon_t b1 = beacon_init(1, 150,  0, 300);
      beacon_t b2 = beacon_init(2, 200,  0, 300);
      beacon_t b3 = beacon_init(3, 250,  0, 300);
      beacon_t b4 = beacon_init(4, 300,  0, 300);
      beacon_t b5 = beacon_init(5, 350,  0, 300);
      beacon_t b6 = beacon_init(6, 400,  0, 300);
      beacon_t b7 = beacon_init(7, 450,  0, 300);
      beacon_t b8 = beacon_init(8, 500,  0, 300);
      beacon_t b9 = beacon_init(9, 475,   50, 300);
      beacon_t b10 = beacon_init(10, 275, 90, 300);
      beacon_t b11 = beacon_init(11, 300, 50, 300);
    */
    
    /*beacon_t b0 = beacon_init(0, 0,    243, 311);
      beacon_t b1 = beacon_init(1, 2.5,  341, 300);
      beacon_t b2 = beacon_init(2, 1.7,  411, 307);
      beacon_t b3 = beacon_init(3, 10,   213, 364);
      beacon_t b4 = beacon_init(4, 220,  222, 433);
      beacon_t b5 = beacon_init(5, 420,  201, 522);
      beacon_t b6 = beacon_init(6, 03,    314, 353);
      beacon_t b7 = beacon_init(7, 220,  323, 464);
      beacon_t b8 = beacon_init(8, 420,  302, 521);
      beacon_t b9 = beacon_init(9, 475,   50, 300);
      beacon_t b10 = beacon_init(10, 275, 90, 300);
      beacon_t b11 = beacon_init(11, 300, 50, 300);
    */
    //b3.stat |= BCN_IGNORED;
    b1.flags |= BCN_ISREF;

    bcn_add_to_list(&beaconlist, &b0);
    bcn_add_to_list(&beaconlist, &b1);
    bcn_add_to_list(&beaconlist, &b2);
    bcn_add_to_list(&beaconlist, &b3);
    bcn_add_to_list(&beaconlist, &b4);
    bcn_add_to_list(&beaconlist, &b5);
    //bcn_add(&beaconlist, &b6);
    //bcn_add(&beaconlist, &b7);
    //bcn_add(&beaconlist, &b8);
    //bcn_add(&beaconlist, &b9);
    //bcn_add(&beaconlist, &b10);
    //bcn_add(&beaconlist, &b11);
    
    sensorlist_t sensorlist = NULL;
    sensor_t s0 = sns_new(0, 0, 0,  30.0);
    sns_add_beaconlist(&s0, beaconlist);
    sns_add_to_list(&sensorlist, &s0);

    sensornode_t* s = sensorlist;
    s->sensor->b_flags[0] |= BCN_SIGNAL_OK;
    s->sensor->b_flags[1] |= BCN_SIGNAL_OK;
    s->sensor->b_flags[2] |= BCN_SIGNAL_OK;
    s->sensor->b_flags[3] |= BCN_SIGNAL_OK;
    s->sensor->b_flags[4] |= BCN_SIGNAL_OK;
    s->sensor->b_flags[5] |= BCN_SIGNAL_OK;
    //s->sensor->b_flags[6] |= BCN_SIGNAL_OK;
    //s->sensor->b_flags[7] |= BCN_SIGNAL_OK;
    //s->sensor->b_flags[8] |= BCN_SIGNAL_OK;
    //s->sensor->b_flags[9] |= BCN_SIGNAL_OK;
    //s->sensor->b_flags[10] |= BCN_SIGNAL_OK;
    // s->sensor->b_flags[11] |= BCN_SIGNAL_OK;
    
    position3d_t pos_real;

    float d = 0;
    unsigned cnt = 0;
    for (int i=0; i<100; i++) {
	pos_real.x = (cm_t)(rand()%600);
	pos_real.y = (cm_t)(rand()%400);
	pos_real.z = (cm_t)30;
	FILE* cf = fopen("/tmp/mlat_s00.txt", "w");;
	fprintf(cf, "%f %f 3\n", (float)pos_real.x, (float)pos_real.y);
	fclose(cf);

	s->sensor->distances[0] = sqrt(     (pos_real.x - b0.position.x)*(pos_real.x - b0.position.x)
					    + (pos_real.y - b0.position.y)*(pos_real.y - b0.position.y)
					    + (pos_real.z - b0.position.z)*(pos_real.z - b0.position.z)) + DISPLACE;
	printf("3d dist to %f %f: %f\n", b0.position.x, b0.position.y,  sqrt(     (pos_real.x - b0.position.x)*(pos_real.x - b0.position.x)
									+ (pos_real.y - b0.position.y)*(pos_real.y - b0.position.y)
									+ (pos_real.z - b0.position.z)*(pos_real.z - b0.position.z)) + DISPLACE);
	s->sensor->distances[1] = sqrt(     (pos_real.x - b1.position.x)*(pos_real.x - b1.position.x)
					    + (pos_real.y - b1.position.y)*(pos_real.y - b1.position.y)
					    + (pos_real.z - b1.position.z)*(pos_real.z - b1.position.z)) + DISPLACE;

	s->sensor->distances[2] = sqrt(     (pos_real.x - b2.position.x)*(pos_real.x - b2.position.x)
					    + (pos_real.y - b2.position.y)*(pos_real.y - b2.position.y)
					    + (pos_real.z - b2.position.z)*(pos_real.z - b2.position.z)) + DISPLACE;

	s->sensor->distances[3] = sqrt(     (pos_real.x - b3.position.x)*(pos_real.x - b3.position.x)
					    + (pos_real.y - b3.position.y)*(pos_real.y - b3.position.y)
					    + (pos_real.z - b3.position.z)*(pos_real.z - b3.position.z)) + DISPLACE;

	s->sensor->distances[4] = sqrt(     (pos_real.x - b4.position.x)*(pos_real.x - b4.position.x)
					    + (pos_real.y - b4.position.y)*(pos_real.y - b4.position.y)
					    + (pos_real.z - b4.position.z)*(pos_real.z - b4.position.z)) + DISPLACE;

	s->sensor->distances[5] = sqrt(     (pos_real.x - b5.position.x)*(pos_real.x - b5.position.x)
					    + (pos_real.y - b5.position.y)*(pos_real.y - b5.position.y)
					    + (pos_real.z - b5.position.z)*(pos_real.z - b5.position.z)) + DISPLACE;
    
	s->sensor->distances[6] = sqrt(     (pos_real.x - b6.position.x)*(pos_real.x - b6.position.x)
					    + (pos_real.y - b6.position.y)*(pos_real.y - b6.position.y)
					    + (pos_real.z - b6.position.z)*(pos_real.z - b6.position.z)) + DISPLACE;

	s->sensor->distances[7] = sqrt(     (pos_real.x - b7.position.x)*(pos_real.x - b7.position.x)
					    + (pos_real.y - b7.position.y)*(pos_real.y - b7.position.y)
					    + (pos_real.z - b7.position.z)*(pos_real.z - b7.position.z)) + DISPLACE;

	s->sensor->distances[8] = sqrt(   (pos_real.x - b8.position.x)*(pos_real.x - b8.position.x)
					  + (pos_real.y - b8.position.y)*(pos_real.y - b8.position.y)
					  + (pos_real.z - b8.position.z)*(pos_real.z - b8.position.z)) + DISPLACE;

	s->sensor->distances[9] = sqrt(     (pos_real.x - b9.position.x)*(pos_real.x - b9.position.x)
					    + (pos_real.y - b9.position.y)*(pos_real.y - b9.position.y)
					    + (pos_real.z - b9.position.z)*(pos_real.z - b9.position.z)) + DISPLACE;

	s->sensor->distances[10] = sqrt(     (pos_real.x - b10.position.x)*(pos_real.x - b10.position.x)
					     + (pos_real.y - b10.position.y)*(pos_real.y - b10.position.y)
					     + (pos_real.z - b10.position.z)*(pos_real.z - b10.position.z)) + DISPLACE;

	s->sensor->distances[11] = sqrt(   (pos_real.x - b11.position.x)*(pos_real.x - b11.position.x)
					   + (pos_real.y - b11.position.y)*(pos_real.y - b11.position.y)
					   + (pos_real.z - b11.position.z)*(pos_real.z - b11.position.z)) + DISPLACE;

	//error_t err = sensor_mlat3d(s->sensor);
	//printf("%s\n", errLookup(err));
	//printf("%f %f %f (err = %f)\n", s->sensor->pos_world.x, s->sensor->pos_world.y, s->sensor->pos_world.z,
	//     sqrt( (s->sensor->pos_world.x-pos_real.x)*(s->sensor->pos_world.x-pos_real.x)
	//	     + (s->sensor->pos_world.y-pos_real.y)*(s->sensor->pos_world.y-pos_real.y)
	//	     + (s->sensor->pos_world.z-pos_real.z)*(s->sensor->pos_world.z-pos_real.z) ));
    
	s->sensor->position_wcs.z = pos_real.z;
	error_t err2 = sns_mlat2d(s->sensor);
	//printf("%s\n", errLookup(err2));
	if (err2 != ERR_NOERROR) {
	    cnt++;
	    d += sqrt( (s->sensor->position_wcs.x-pos_real.x)*(s->sensor->position_wcs.x-pos_real.x)
		       + (s->sensor->position_wcs.y-pos_real.y)*(s->sensor->position_wcs.y-pos_real.y) );
	    printf("%f %f (err = %f)\n", s->sensor->position_wcs.x, s->sensor->position_wcs.y,
		   sqrt( (s->sensor->position_wcs.x-pos_real.x)*(s->sensor->position_wcs.x-pos_real.x)
			 + (s->sensor->position_wcs.y-pos_real.y)*(s->sensor->position_wcs.y-pos_real.y) ));
	    
	}
    }
    printf ("mean error: %f\n",d/cnt);
    return 0;
}

int main2(sensorlist_t sl)
{
    srand(time(NULL));

    vehicle_t vstate_real, vehicle;
    veh_init(&vehicle, 0, sl);
    veh_init(&vstate_real, 1, sl);
    matrix_t* rot_matrix = mtr_alloc(2,2);
    
    position2d_t pos;
    position2d_t res;

    error_t err = ERR_NOERROR;

    double veh_calc_err_angle, veh_calc_err_pos;
    double veh_calc_err_angle_tot = 0;
    double veh_calc_err_pos_tot = 0;
    int cnt = 0;

    for (int i=0; i<10000; i++) {
	vstate_real.position.x = (cm_t)(rand()%600);
	vstate_real.position.y = (cm_t)(rand()%400);
	vstate_real.position.orientation = (rand()%180)*M_PI/180.0;


    sensornode_t* s = sl;
    /* simulate sensor positions displaced by DISPLACE due to meas. errors */
    while (s != NULL) {

	/* grab sensor position in the vehicle CS */
	pos.x = s->sensor->position_mnt.x;
	pos.y = s->sensor->position_mnt.y;
	
	/* apply rotation */
	err |= mtr_make_rot2d(rot_matrix, vstate_real.position.orientation);
	err |= mtr_apply_to_pos2d(rot_matrix, &pos, &res);

	/* translate sensor position from vehicle CS to the world CS */ 
	s->sensor->position_wcs.x = res.x + vstate_real.position.x + DISPLACE;
	s->sensor->position_wcs.y = res.y + vstate_real.position.y + DISPLACE;
	s->sensor->flags |= SNS_MLAT_OK;
	s = s->next;
    }

    /* calculate vehicle position based on sensor data */
    veh_calc_position(&vehicle);

    veh_calc_err_pos = sqrt((vstate_real.position.x- vehicle.position.x)*(vstate_real.position.x- vehicle.position.x)+(vstate_real.position.y- vehicle.position.y)*(vstate_real.position.y- vehicle.position.y));
    veh_calc_err_pos_tot += veh_calc_err_pos;
    printf ("Center:    real: (%.1f, %.1f), calc: (%.1f, %.1f), err: %.1f\n",
	    vstate_real.position.x, vstate_real.position.y,
	    vehicle.position.x, vehicle.position.y,
	    veh_calc_err_pos);
    
    /* start value for the orientation angle */
    double dx = (sns_get_byID(&sl, 1))->sensor->position_wcs.x - (sns_get_byID(&sl, 0))->sensor->position_wcs.x;
    double dy = (sns_get_byID(&sl, 1))->sensor->position_wcs.y - (sns_get_byID(&sl, 0))->sensor->position_wcs.y;
    double phis = atan((float)dy/(float)dx);
    
    /* find optimal (in terms of LSQ) orientation */
    s=sl;
    while (s != NULL) {
	/* move to vehicle CS */
	s->sensor->position_vcs.x = s->sensor->position_wcs.x -  vehicle.position.x;
	s->sensor->position_vcs.y = s->sensor->position_wcs.y -  vehicle.position.y;
	s = s->next;
    }
    double dphi = 0;
    double cost;
    double costmin = 1e9;
    double dphi_opt = 0;;

    for (dphi=-5.0*M_PI/180; dphi<5.0*M_PI/180; dphi+=.1*M_PI/180) {
	err |= mtr_make_rot2d(rot_matrix, phis + dphi);
	s=sl;
	cost = 0;
	while (s != NULL) {
	    pos.x = s->sensor->position_mnt.x;
	    pos.y = s->sensor->position_mnt.y;
	    
	    err |= mtr_apply_to_pos2d(rot_matrix, &pos, &(s->sensor->position_vcs_est));
	    cost += pln_dist_L2_2(&(s->sensor->position_vcs), &(s->sensor->position_vcs_est));
	    s = s->next;
	}
	if (cost < costmin) {
	    costmin = cost;
	    dphi_opt = dphi;
	}
    }

    veh_calc_err_angle = fabs(180*vstate_real.position.orientation/M_PI - 180*vehicle.position.orientation/M_PI);
    veh_calc_err_angle_tot += veh_calc_err_angle;
    cnt++;
    printf("Rotation: real: %.1f, simple: %.1f (%.1f), fit: %.1f (%.1f), vh_calc_rotation(): %.1f (%.1f)\n",
	   180*vstate_real.position.orientation/M_PI,
	   180*phis/M_PI, 180*vstate_real.position.orientation/M_PI - 180*phis/M_PI,
	   180*(phis+dphi_opt)/M_PI, 180*vstate_real.position.orientation/M_PI - 180*(phis+dphi_opt)/M_PI,
	   180*vehicle.position.orientation/M_PI, veh_calc_err_angle);
    }
    
    printf ("mean pos err: %.1f, mean rotation err: %.1f\n", veh_calc_err_pos_tot/cnt, veh_calc_err_angle_tot/cnt);
    return 0;
}

int main()
{
    beaconlist_t beaconlist = NULL;

    beacon_t b0 = bcn_init(0, 351.5, 401.0, 274.0);
    beacon_t b1 = bcn_init(1, 366-11.0, 45.0,  274.0);
    beacon_t b2 = bcn_init(2, 244-16.0,   401, 274.0);
    //beacon_t b3 = beacon_init(3,  122-8.5,  399,  274);
    beacon_t b4 = bcn_init(4, 122+39.0,   2.0, 246.0);
    beacon_t b5 = bcn_init(5,  16, 1,  245.);
    beacon_t b6 = bcn_init(6,  -10.0,  449-48.5,  274.0);

    beacon_t b7 = bcn_init(7, 351.5+50, 401.0, 274.0);
    beacon_t b8 = bcn_init(8, 366-11.0+50, 45.0,  274.0);
    beacon_t b9 = bcn_init(9, 244-16.0+50,   401, 274.0);
    beacon_t b10 = bcn_init(10, 122+39.0+50,   2.0, 246.0);
    beacon_t b11 = bcn_init(11,  16+50, 1,  245.);
    beacon_t b12 = bcn_init(12,  -10.0+50,  449-48.5,  274.0);

    b1.flags |= BCN_ISREF;

    bcn_add_to_list(&beaconlist, &b0);
    bcn_add_to_list(&beaconlist, &b1);
    bcn_add_to_list(&beaconlist, &b2);
    //beacon_add(&beaconlist, &b3);
    bcn_add_to_list(&beaconlist, &b4);
    bcn_add_to_list(&beaconlist, &b5);
    bcn_add_to_list(&beaconlist, &b6);
    bcn_add_to_list(&beaconlist, &b7);
    bcn_add_to_list(&beaconlist, &b8);
    bcn_add_to_list(&beaconlist, &b9);
    bcn_add_to_list(&beaconlist, &b10);
    bcn_add_to_list(&beaconlist, &b11);
    bcn_add_to_list(&beaconlist, &b12);

    sensorlist_t sensorlist = NULL;

    sensor_t s0 = sns_new(0, -50.0, 0.0, 1.0);
    sns_add_beaconlist(&s0, beaconlist);
    sensor_t s1 = sns_new(1,  0.0,  0.0, 1.0);
    sns_add_beaconlist(&s1, beaconlist);
    //sensor_t s2 = sns_new(2, 43.0,  0.0, 1.0);
    //sns_add_beaconlist(&s2, beaconlist);
    sensor_t s3 = sns_new(3, 43.0,  0.0, 1.0);
    sns_add_beaconlist(&s3, beaconlist);
    sensor_t s4 = sns_new(4, 70.0,  0.0, 1.0);
    sns_add_beaconlist(&s4, beaconlist);

    sensor_t s5 = sns_new(5, -50.0, 50.0, 1.0);
    sns_add_beaconlist(&s5, beaconlist);
    sensor_t s6 = sns_new(6,  0.0,  50.0, 1.0);
    sns_add_beaconlist(&s6, beaconlist);
    sensor_t s7 = sns_new(7, 43.0,  50.0, 1.0);
    sns_add_beaconlist(&s7, beaconlist);
    sensor_t s8 = sns_new(8, 70.0,  50.0, 1.0);
    sns_add_beaconlist(&s8, beaconlist);

    sns_add_to_list(&sensorlist, &s0);
    sns_add_to_list(&sensorlist, &s1);
    //sensor_add(&sensorlist, &s2);
    sns_add_to_list(&sensorlist, &s3);
    sns_add_to_list(&sensorlist, &s4);
    sns_add_to_list(&sensorlist, &s5);
    sns_add_to_list(&sensorlist, &s6);
    sns_add_to_list(&sensorlist, &s7);
    sns_add_to_list(&sensorlist, &s8);

    return main2(sensorlist);

    uint8_t num_of_beacons = bcn_count(&beaconlist, BCN_ANY);
    
    unsigned bid, sid;
    beaconnode_t* b;
    sensornode_t* s;
    srand(time(NULL));
    int fledm_x = 100;
    int fledm_y = 100;
    cm_t dist = 0;
    position3d_t real_pos;
    position3d_t meas_pos;
    error_t err;

    kalman_filter_t kf;
    kf_state_t z;
    err = kf_init(&kf);
    if (err) {
	printf("KF init error\n");
    }
    int motion_changed = 0;
    mtr_set(kf.x,0,0,100);
    mtr_set(kf.x,1,0,100);
    real_pos.x = 100;
    real_pos.y = 100;
    
#if 0
    int fd = open_serial("/dev/ttyACM0");
    if (fd < 0) {
	return 1;
    }

    message_t msg = {0, 0, NULL};
    
    while(1) {
	error_t err = read_next_message(&msg, fd);
	if (err != ERR_NOERROR) {
	    printf("Error %d\n", err);
	} else {
	    if (msg.msg_type == MSG_TYPE_DIST) {
		memcpy(&id,   msg.data,   sizeof(char));
		memcpy(&dist, msg.data+1, sizeof(unsigned16_t));
		printf("%d %d\n", id, (int)dist);
		set_distance_byID(&beaconlist, id, dist);
		trilateration(&beaconlist, &meas_pos);
		printf("Position: (%d,%d) \n", (int)meas_pos.x, (int)meas_pos.y);
	    }
	}
	list_nodes(&beaconlist);
	usleep(400000);
    }
#endif

    unsigned   len_burst_us = PULSE_DUR_MSEC*1000;
    unsigned   srate_tx     = 4*CARRIER_FREQ; // 0 -1 0 1  
    size_t     len_burst    = len_burst_us*srate_tx/1000000;

    sample_t* burst = generate_burst(CARRIER_FREQ, srate_tx, len_burst_us, M_PI/3);
    sig_apply_gauss(burst, len_burst);
    size_t i;

    int step;
    int steps = 100;
    for (step=0; step<steps; step++) {
	if(step<steps/2) {
	    fledm_x += 0;
	    fledm_y += 0;
	    z.vx = 0;
	    mtr_set(kf.x, 2,0,0);
	}
	if(step>=steps/2) {
	    if(!motion_changed) {
		mtr_set(kf.x, 2, 0, 5*(step/(double)steps)*(step/(double)steps));
		motion_changed=1;
	    }
	    fledm_x += 5;//*(step/(double)steps)*(step/(double)steps);
	    fledm_y += 0;
 	    real_pos.x = fledm_x;
	    z.vx = 5 + (rand()%(2*NOISE+1)-NOISE) ;

	}

	uint8_t scount = sns_count(&sensorlist, SNS_ANY);
	for (sid=0; sid<scount; sid++) {
	    s = sns_get_byID(&sensorlist, sid);
	    for (bid=0; bid<num_of_beacons; bid++) {

		b = bcn_get_byID(&(s->sensor->beaconlist), bid);
	    
		if (b != NULL) {
		    dist = (uint16_t)(.5 + sqrt( (fledm_x - b->beacon->position.x) * (fledm_x - b->beacon->position.x)
						 + (fledm_y - b->beacon->position.y) * (fledm_y - b->beacon->position.y) ))
			+ rand()%(2*NOISE+1) - (2*NOISE);
		}
	    
		const float  tof_theor  = .01 * dist/SPEED_OF_SOUND;  /* expected time-of-flight */
		const size_t len_rc     = REC_SAMPLE_RATE*len_burst/srate_tx; /* non-zero signal array length */
		const size_t len_rc_tot = len_rc + (size_t)ceilf((float)REC_SAMPLE_RATE*tof_theor);
	    
		sample_t* sig_rc = (sample_t*)calloc(len_rc, sizeof(sample_t));
	    
		/* sample original signal with the sampling rate REC_SAMPLING_RATE */
		for (i=0; i<len_rc; i++) {
		    sig_rc[i] = burst[(size_t)(.5 + (float)i*(float)srate_tx/(float)REC_SAMPLE_RATE)];
		}
	    
		sample_t* rawsig = (sample_t*)calloc(len_rc_tot, sizeof(sample_t)); /* total RC signal length (TOF + burst) */
		memmove(rawsig + (size_t)(REC_SAMPLE_RATE*tof_theor), sig_rc, len_rc*sizeof(sample_t));
	    
#if 0
		printf("#### Burst: ####\n");
		for (i=0; i<len_burst; i++) {
		    printf("%f %f %f %f\n", (float)burst[i], 0., 0., 0.);
		}
		printf("#### Sampled signal: ####\n");
		for (i=0; i<len_rc_tot; i++) {
		    printf("%f %f %f %f \n", 0., (float)(rawsig[i]), 0., 0.);
		}
#endif
	
		sample_t* ip = (sample_t*)calloc(len_rc_tot, sizeof(sample_t));
		sample_t* qu = (sample_t*)calloc(len_rc_tot, sizeof(sample_t));
		sample_t* en = (sample_t*)calloc(len_rc_tot, sizeof(sample_ut));
	    
		size_t sig_max_pos = sig_get_pos_of_max(&rawsig[0], len_rc_tot);
	    
#if 0
		sample_t nlevel = rawsig[sig_max_pos]/4;
		for (i=0; i<len_rc_tot; i++) {
		    rawsig[i] += (rand()%nlevel) - nlevel;
		}
#endif
		error_t res = sig_decode(rawsig, len_rc_tot, ip, qu);
		sig_magnitude(ip, qu, len_rc_tot, en);
	    
		size_t   parx[3];
		sample_t pary[3], parab_width;
		float tof_meas;
	    
		switch (Q_SAMPLING_M) {
		case 3:
		    parab_width = 10;
		    break;
		case 5:
		    parab_width = 6;
		    break;
		case 7:
		    parab_width = 3;
		    break;
		default:
		    parab_width = 3;
		}
	    
		parx[1] = sig_max_pos;
		res = sig_get_parabola_suppts(en, len_rc_tot, &parx[0], &pary[0]);
	    
		if (res != ERR_NOERROR) {
		    fprintf(stderr, "ERROR\n");
		    tof_meas = 0;
		} else {
		    int en_max_pos = parx[1];
		
		    float interp_delta = ((float)(en[en_max_pos - parab_width]) - en[en_max_pos + parab_width])
			/( 2 * (en[en_max_pos - parab_width] -2*en[en_max_pos] + en[en_max_pos + parab_width]));
		    float par_vertex = ((float)(en_max_pos) + interp_delta*parab_width);
		
		    tof_meas = (par_vertex)/(float)REC_SAMPLE_RATE;
		    float dphi = atan2f(ip[(size_t)ceilf(en_max_pos)], qu[(size_t)ceilf(en_max_pos)]);
		    tof_meas += dphi/(M_TWOPI*CARRIER_FREQ);
		    tof_meas -= (float)PULSE_DUR_MSEC/2000;
		}
	    
		bcn_set_distance_byID(&(s1.beaconlist), bid, (uint16_t)(tof_meas*SPEED_OF_SOUND*100 + .5));
	
		free(ip);
		free(qu);
		free(en);
		free(rawsig);
		free(sig_rc);
	    }
	    
	}

	err = sns_mlat2d(s->sensor);
	if (err != ERR_NOERROR) {
	    fprintf(stderr, "Trilateration failed\n");
	    return 1;
	}
	
	meas_pos = s->sensor->position_wcs;
	meas_pos.x = real_pos.x + rand()%(2*NOISE+1)-NOISE;
	z.x = meas_pos.x;
	z.y = meas_pos.y;
	z.vy = 0;
	
	err = kf_update(&kf, &z); 
	if (err) {
	    printf("KF error\n");
	}
	
	printf("%d %d %d %d %d %d %d %d %f\n",
	       (int)real_pos.x, (int)real_pos.y,
	       (int)z.x, (int)z.y,
	       (int)mtr_get(kf.x_,0,0), (int)mtr_get(kf.x_,1,0),
	       (int)fabs(real_pos.x-z.x),
	       (int)fabs(real_pos.x-mtr_get(kf.x_,0,0)),
	       mtr_get(kf.x,2,0));
    }
    return 0;
}

