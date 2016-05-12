#include "calibrator.h"
#include "sensor.h"
#include "beacon.h"
#include "signal.h"
#include "messaging.h"
#include "serial.h"
#include "movavg.h"

#include <stdio.h>
#include <math.h>
#include <libconfig.h>
#include <string.h>
#include <sys/time.h>

#define MAX_STDDEV      5.0  /* range measurement with stddev above this value will not be
				accepted */
#define RESET_STDDEV    20.0 /* if stddev is above this value, statistics will be reset */
#define MAX_MCANUM      100  /* max number of statistics samples before resetting */
#define MAX_RESETS      15   /* give up whenever reset more often than this number of times */
#define MAX_TOF_FAILURE 40   /* give up if TOF calculation failed this number of times */

static int	fd;

static matrix_t *clb_gain;
static matrix_t *clb_offset;

static double	*tof_dist[6][6];
static double	*real_dist[6][6];

static uint8_t	nresets[6][6];
static size_t	len[6][6];

static beaconlist_t	bl;
static sensorlist_t	sl;
static beaconnode_t*	bn;
static sensornode_t*	sn;

static double		t_correction = 0;
static uint8_t tof_flags = TOF_APPLY_RECT_WINDOW | TOF_REMOVE_DC;
static double		dt_s,	mydt_s, prev_mydt_s;
static struct timeval	mytime, myprevtime;
static movavg_t mav_dt;
static int		firstrun     = 1;

void calc_()
{
    double av_gain=0, av_offset=0;
    int n=0;
    double pres[2];

    /* Calculate calibration for every pair */
    bn = bl;
    while (bn != NULL) {
	sn = sl;
	while (sn != NULL) {

	    for (int k=0; k<len[bn->beacon->id][sn->sensor->id]; k++) {
		printf("%6.1f -> %6.1f\n",
		       *(tof_dist[bn->beacon->id][sn->sensor->id] + k),
		       *(real_dist[bn->beacon->id][sn->sensor->id] + k));
	    }
		
	    if (len[bn->beacon->id][sn->sensor->id] >= 3) {
		printf("Calibrating b %d s %d: ", bn->beacon->id, sn->sensor->id);
		clb_calc_for_sbpair(tof_dist[bn->beacon->id][sn->sensor->id],
				    real_dist[bn->beacon->id][sn->sensor->id],
				    len[bn->beacon->id][sn->sensor->id],
				    pres);
		mtr_set(clb_gain, bn->beacon->id, sn->sensor->id, pres[0]);
		mtr_set(clb_offset, bn->beacon->id, sn->sensor->id, pres[1]);

		printf("%.6f %.6f\n", pres[0], pres[1]);
		n++;
		av_gain += pres[0];
		av_offset += pres[1];
	    }
	    sn = sn->next;
	}
	bn = bn->next;
    }

    av_gain /= (double)n;
    av_offset /= (double)n;
    
    /* missing pairs: use average values */
    bn = bl;
    while (bn != NULL) {
	sn = sl;
	while (sn != NULL) {
	    if (len[bn->beacon->id][sn->sensor->id] < 3) {
		printf("Calibrating b %d s %d with average values\n", bn->beacon->id, sn->sensor->id);
		mtr_set(clb_gain, bn->beacon->id, sn->sensor->id, av_gain);
		mtr_set(clb_offset, bn->beacon->id, sn->sensor->id, av_offset);
	    }
	    sn = sn->next;
	}
	bn = bn->next;
    }

    
    mtr_print(clb_gain);
    mtr_print(clb_offset);
}

int main(int argc, char** argv)
{
    error_t err = ERR_NOERROR;
    sample_t* rawsig = NULL;
    rawsig = (sample_t*)malloc(BUFFER_LEN*sizeof(sample_t)); //TODO: INFO msg/config?

    uint8_t bid=199, sid=199, numb, nums, hbid=0, hsid=0;
    
    err |= bcn_read_config("theater.conf", &bl);
    //bn = bl->next;
    //bn->beacon->flags |= BCN_ISREF;

    err |= sns_read_config("theater.conf", &sl);
    sn = sl;
    while (sn != NULL) {
	sns_add_beaconlist(sn->sensor, bl);
	sn = sn->next;
    }

    sn = sl;
    while (sn != NULL) {
	if (sn->sensor->id > hsid) {
	    hsid = sn->sensor->id;
	}
	sn = sn->next;
    }
	
    bn = bl;
    while (bn != NULL) {
	if (bn->beacon->id > hbid) {
	    hbid = bn->beacon->id;
	}
	bn = bn->next;
    }

    nums = sns_count(&sl, SNS_ANY);
    numb = bcn_count(&bl, BCN_ANY);
    
    char in[32];
    int n, x, y, phi;
    unsigned pos_count = 0;

    clb_gain = mtr_alloc(hbid+1,hsid+1);
    clb_offset = mtr_alloc(hbid+1,hsid+1);
    
    /* input to clb_calc_for_sbpair() */
    matrix_t *mlen = mtr_alloc(hbid+1, hsid+1);
    movavg_t mc[hbid+1][hsid+1];
    uint8_t   done[hbid+1][hsid+1];
    matrix_t *mdone  = mtr_alloc(hbid+1,hsid+1);
    matrix_t *mdist   = mtr_alloc(hbid+1,hsid+1);
    matrix_t *mrealdist   = mtr_alloc(hbid+1,hsid+1);
    matrix_t *mstddev = mtr_alloc(hbid+1,hsid+1);
    uint8_t toflt0[hbid+1][hsid+1];
    uint8_t ndone;
    matrix_t* rotmtr = mtr_alloc(2,2);

    /* allocate memory for 8 measurements per beacon-sensor pair (3 are necessary) */
    bn = bl;
    while (bn != NULL) {
	sn = sl;
	while (sn != NULL) {
	    real_dist[bn->beacon->id][sn->sensor->id] = (double*)malloc(8*sizeof(double));
	    tof_dist[bn->beacon->id][sn->sensor->id] = (double*)malloc(8*sizeof(double));
	    mav_init(&(mc[bn->beacon->id][bn->beacon->id]));
	    done[bn->beacon->id][sn->sensor->id] = 0;
	    len[bn->beacon->id][sn->sensor->id] = 0;
	    ndone = 0;
	    nresets[bn->beacon->id][sn->sensor->id] = 0;
	    sn = sn->next;
 	}
	bn = bn->next;
    }

    fd = sio_open_serial(argv[1]);

    for (;;) { /* vehicle position loop */
	
	printf("Enter vehicle position in cm and rotation in deg: ");
	fgets(in, sizeof(in), stdin);
	n = sscanf(&in[0], "%d %d %d", &x, &y, &phi);
	
	if ( n != 3 ) {
	    if (pos_count < 3) {
		printf("At least 3 measurements required\n");
		
		bn = bl;
		while (bn != NULL) {
		    sn = sl;
		    while (sn != NULL) {
			printf("b%ds%d:\n", bn->beacon->id, sn->sensor->id);
			for (int k=0; k<len[bn->beacon->id][sn->sensor->id]; k++) {
			    printf("%6.1f -> %6.1f\n",
				   *(tof_dist[bn->beacon->id][sn->sensor->id] + k),
				   *(real_dist[bn->beacon->id][sn->sensor->id] + k));
			}
			sn = sn->next;
		    }
		    bn = bn->next;
		}

		continue;
	    }
	    calc_();
	    break;
	}
	
	pos_count++;

	/* Calc real beacon-sensor distances for the current position */
	error_t err = ERR_NOERROR;
	err |= mtr_make_rot2d(rotmtr, (double)(phi*M_PI/180.));
	if (err != ERR_NOERROR) {
	    printf ("Error calculating rotation matrix for the sensor array\n");
	}

	bn = bl;
	while (bn != NULL) {
	    sn = sl;
	    while (sn != NULL) {
		position2d_t pos, res;
		pos.x = sn->sensor->position_mnt.x;
		pos.y = sn->sensor->position_mnt.y;
		err = mtr_apply_to_pos2d(rotmtr, &pos, &res);
		if (err != ERR_NOERROR) {
		    printf ("Error applying rotation to the sensor array\n");
		}
		sn->sensor->position_wcs.x = x + res.x;
		sn->sensor->position_wcs.y = y + res.y;

		//double dx = (sn->sensor->position_wcs.x - bn->beacon->position.x);
		//double dy = (sn->sensor->position_wcs.y - bn->beacon->position.y);
		//double dz = (sn->sensor->position_wcs.z - bn->beacon->position.z);
		//*(real_dist[bn->beacon->id][sn->sensor->id] + pos_count-1) = sqrt(dx*dx+dy*dy+dz*dz);

		*(real_dist[bn->beacon->id][sn->sensor->id] + pos_count-1) =
		    sns_pos2dist3d(&(sn->sensor->position_wcs), &(bn->beacon->position));
		printf ("%f\n", *(real_dist[bn->beacon->id][sn->sensor->id] + pos_count-1));
		/* reset moving average tracker for measured distance */
		mav_init(&mc[bn->beacon->id][sn->sensor->id]);

		/* unset flags */
		done[bn->beacon->id][sn->sensor->id] = 0;
		toflt0[bn->beacon->id][sn->sensor->id] = 0;
		nresets[bn->beacon->id][sn->sensor->id] = 0;

		ndone = 0;
		sn = sn->next;
	    }
	    bn = bn->next;
	}

	/* Measure uncalibrated distances ( = 1*SPEED_OF_SOUND*tof + 0) for given position */
	message_t msg;
	for (;;) { /* acquire all measurements for the current position */
	    err = msg_read_next_message(&msg, fd, (uint8_t*)rawsig);
	    if (err != ERR_NOERROR) {
		printf ("Error while reading message: %s\n", err_lookup(err));
		continue;
	    }

	    if (bid != msg.beacon_id ) {
		bid = msg.beacon_id;
		sid = msg.sensor_id;
		myprevtime = mytime;
		gettimeofday(&mytime, NULL);
		prev_mydt_s = mydt_s;
		mydt_s = mytime.tv_sec - myprevtime.tv_sec + (mytime.tv_usec - myprevtime.tv_usec)*1e-6;
		
		if (!firstrun && dt_s > 1e-4 && dt_s < .5) {
		    mav_update_mean(&mav_dt, dt_s);
		    t_correction = mav_get_mean(&mav_dt) - mydt_s;
		}
	    }
	    
	    sid = msg.sensor_id;
	    bid = msg.beacon_id;

	    if (done[bid][sid] == 0) {
		sn = sns_get_byID(&sl, sid);
		if (sn == NULL) {
		    //fprintf (stderr, "Sensor %d is not in the sensor list\n", sid);
		    continue;
		}
		bn = bcn_get_byID(&bl, bid);
		if (bn == NULL) {
		    //fprintf (stderr, "Beacon %d is not in the beacon list\n", bid);
		    continue;
		}
		
		memcpy((void*)&(sn->sensor->rawsig)[0], (void*)rawsig, sizeof(sample_t)*BUFFER_LEN);
		
		double timeofflight  = sig_get_timeofflight(rawsig, BUFFER_LEN, tof_flags);
		if (timeofflight < 0) {
		    toflt0[bid][sid] += 1;
		    continue;
		}
		
		timeofflight += t_correction;

		cm_t dist_cm = (cm_t)(timeofflight*SPEED_OF_SOUND*METER2CM);

		/* Outlier? */
		cm_t diff = fabs(dist_cm - mav_get_mean(&mc[bid][sid]));
		if ( mav_get_stddev(&mc[bid][sid]) > .01 /* skip at the first run */
		     && diff > MAX_STDDEV /* avoid being locked at a wrong mean */
		     && diff > 3.0*mav_get_stddev(&mc[bid][sid])) {
		    fprintf(stderr, "\rOutlier b%d s%d: measured %6.1f expected %6.1f ± %3.1f\n",
			    bid, sid,
			    dist_cm,
			    mav_get_mean(&mc[bid][sid]),
			    3*mav_get_stddev(&mc[bid][sid]));
		    continue;
		}

		/* Not an outlier, update moving average */
		mav_update_mean(&mc[bid][sid], dist_cm);

		mtr_set(mrealdist, bid, sid, *(real_dist[bn->beacon->id][sn->sensor->id] + pos_count-1));
		mtr_set(mdist, bid, sid, mc[bid][sid].mean);
		mtr_set(mstddev, bid, sid, mav_get_stddev(&mc[bid][sid]));
		
		if (mav_get_stddev(&mc[bid][sid]) > RESET_STDDEV
		    || (mc[bid][sid].num > MAX_MCANUM && mav_get_stddev(&mc[bid][sid]) > MAX_STDDEV) ) {

		    mav_init(&mc[bid][sid]);
		    nresets[bid][sid]++;
#if 0
		    printf("\rReinitializing b %d s %d stats ", bid, sid);
		    printf("for the %d time (stddev %.1f after %d measurements)\n",
			   nresets[bid][sid],
			   mav_get_stddev(&mc[bid][sid]),
			   (int)mc[bid][sid].num);
#endif
		    
		    /* if reset too often (> MAX_RESETS), beacon is probably invisible for the
		       sensor. Abort measuring b-s pair (done=1) without incrementing
		       len[bid][sid] */
		    if (nresets[bid][sid] > MAX_RESETS || toflt0[bid][sid] > MAX_TOF_FAILURE) {
			ndone++;
#if 0
			printf("Cancelling b %d s %d ", bid, sid);
			printf("after %d resets (%d tof failed) with stddev %.1f.\n",
			       nresets[bid][sid],
			       toflt0[bid][sid],
			       mav_get_stddev(&mc[bid][sid]));
#endif
			
		    }
		}
		    
		if (mc[bid][sid].num> 5 && mav_get_stddev(&mc[bid][sid]) < MAX_STDDEV
		    && (done[bid][sid] == 0)) {
		    done[bid][sid] = 1;
		    ndone++;
		    mtr_set(mdone, bid, sid, 1);
		    len[bid][sid] += 1;
		    mtr_set(mlen, bid, sid, mtr_get(mlen,bid,sid)+1);

		    
		      printf("\rb %d s %d data OK after %d measurements with stddev %.1f.\n",
			   bid, sid,
			   (int)mc[bid][sid].num,
			   mav_get_stddev(&mc[bid][sid])); 
		      

		    *(tof_dist[bid][sid] + pos_count-1) = mc[bid][sid].mean;
		    

		}
	    } /* end if done[bid][sid] == 0 */
	    static char st[] = {'\\', '|', '/', '-'};
	    static int i;
	    printf("\r%c", st[i++ % 4]);fflush(stdout);
	    if (ndone == numb*nums) {
		//mtr_print(mrealdist);
		mtr_print(mlen);
		mtr_print(mdist);
		mtr_print(mstddev);
		
		firstrun = 1;
		break;
	    }
	} /* end loop current position */
    } /* end loop all positions */
    
    printf("Gain (c1)\n");
    mtr_print(clb_gain);
    printf("Offset (c0)\n");
    mtr_print(clb_offset);

    printf("calibration:\n{\n\tpairs = (\n");
    for (bid=0; bid<clb_gain->nrows; bid++) {
          for (sid=0; sid<clb_gain->ncols; sid++) {
	      printf("\t{beacon = %d; sensor = %d; c1 = %10.6f; c0 = %10.6f;}", bid, sid,
		     mtr_get(clb_gain, bid,sid),
		     mtr_get(clb_offset, bid,sid));
	      if (bid < clb_gain->nrows-1 || sid <clb_gain->ncols-1) {
		  printf(",\n");
	      } else {
		  printf("\n\t);\n};\n");
	      }
	  }
   }
   
   return err;
}
