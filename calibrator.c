/* Last modified: <19-Oct-2015 15:10:58 CEST by Dmitry Ebel> */

#include "calibrator.h"
#include "matrix.h"
#include <math.h>

error_t clb_read_config(const char* fn, config_t* cfg, matrix_t** c1m, matrix_t** c0m)
{
    config_setting_t *cpairs, *cpair; 
    int bid, sid, npairs, maxbid=0, maxsid=0;
    double c0, c1;
    
    config_init(cfg);
    
    if(! config_read_file(cfg, fn)) {
	fprintf(stderr, "%s:%d - %s\n", config_error_file(cfg),
		config_error_line(cfg), config_error_text(cfg));
	config_destroy(cfg);
	return(1);
    }

    /* get list of beacons */
    cpairs = config_lookup(cfg, "calibration.pairs");

    if(cpairs == NULL) {
	return ERR_NOTENOUGHDATA;
    }

    /* get number of beacons in the list */
    npairs = config_setting_length(cpairs);
    
    /* find highest bid, sid and allocate the calibration matrix */
    for(int pcnt = 0; pcnt < npairs; pcnt++) {
	/* get single beacon */
	cpair = config_setting_get_elem(cpairs, pcnt);	
	
	/* get pair */
	config_setting_lookup_int(cpair, "beacon", &bid);
	config_setting_lookup_int(cpair, "sensor", &sid);
	if (bid>maxbid) {
	    maxbid=bid;
	}
	if (sid>maxsid) {
	    maxsid=sid;
	}
    }

    printf("%d pairs found (%d beacons x %d sensors)\n", npairs, maxbid+1, maxsid+1);
    
    *c1m = mtr_alloc(maxbid+1,maxsid+1);
    *c0m = mtr_alloc(maxbid+1,maxsid+1);
    printf("Allocated %dx%d calibration\n", (*c1m)->nrows, (*c1m)->ncols); 
    
    for(int pcnt = 0; pcnt < npairs; pcnt++) {
	/* get single beacon */
	cpair = config_setting_get_elem(cpairs, pcnt);	
	
	/* get pair */
	config_setting_lookup_int(cpair, "beacon", &bid);
	config_setting_lookup_int(cpair, "sensor", &sid);
	config_setting_lookup_float(cpair, "c1", &c1);
	config_setting_lookup_float(cpair, "c0", &c0);
	mtr_set(*c1m, bid, sid, c1);
	mtr_set(*c0m, bid, sid, c0);
    }
    
    return ERR_NOERROR;
}


error_t clb_calc_for_sbpair(const double const* tof_dist,
			     const double const* real_dist,
			     const size_t len,
			     double pres[2])
{
    error_t err = ERR_NOERROR;
    matrix_t* A  = mtr_alloc(len,  2);
    matrix_t* b  = mtr_alloc(len,  1);
    matrix_t* At = mtr_alloc(2, len);
    matrix_t* Pinv = mtr_alloc(2, 2);
    matrix_t* Ainv = mtr_alloc(2, len);
    matrix_t* P    = mtr_alloc(2, 2);
    matrix_t* res  = mtr_alloc(2, 1);
    uint8_t m = 0;
    
    for (m=0; m<len; m++) {
	mtr_set(A, m, 0, tof_dist[m]);
	mtr_set(A, m, 1, 1.0);
	mtr_set(b, m, 0, real_dist[m]);
    }

    mtr_print(A);
    mtr_print(b);
    
    err |= mtr_transpose(A, At);
    err |= mtr_product(At, A, P);
    err |= mtr_inverse2x2(P, Pinv);
    err |= mtr_product(Pinv, At, Ainv);
    err |= mtr_product(Ainv, b, res);

    if (err != ERR_NOERROR) {
	pres[0] = 0;
	pres[1] = 0;
	return err;
    }
    pres[0] = mtr_get(res, 0, 0);
    pres[1] = mtr_get(res, 1, 0);
    
    mtr_free(A);
    mtr_free(b);
    mtr_free(At);
    mtr_free(Pinv);
    mtr_free(P);
    mtr_free(Ainv);
    mtr_free(res);
    
    return err;
}

int clb_make_clb_conf()
{
    return 0;
}
