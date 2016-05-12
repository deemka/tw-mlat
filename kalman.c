/* Last modified: <08-Oct-2015 17:37:26 CEST by Dmitry Ebel> */
#include "kalman.h"
#include <math.h>
#include <stdlib.h>
extern void* memcpy(void*, const void*, size_t);

static double change_sign(const double x)
{
    return -1.0*x;
}

error_t kf_init(kalman_filter_t* kf) {
    
    error_t err = ERR_NOERROR;

    kf->counter           = 0; 
    kf->dt_ms             = DT_MS;
    kf->transition_noise  = TRANS_NOISE;
    kf->measurement_noise = MEAS_NOISE;
    kf->w_noise           = W_NOISE;
    kf->steady            = 0;
    
    /* vehicle state initialized with zeros */ 
    kf->x  = mtr_alloc(4,1);
    kf->x_ = mtr_alloc(4,1);
    kf->_x = mtr_alloc(4,1);
    if (kf->x == NULL || kf->x_ == NULL || kf->_x == NULL) {
        return ERR_MALLOC;
    }
    
    /* transition matrix initialized as U */
    kf->F  = mtr_alloc(4,4);
    if (kf->F == NULL) {
        return ERR_MALLOC;
    }
    err |= mtr_set(kf->F, 0, 0, (double)1);
    err |= mtr_set(kf->F, 0, 2, (double)kf->dt_ms/1000.);
    err |= mtr_set(kf->F, 1, 1, (double)1);
    err |= mtr_set(kf->F, 1, 3, (double)kf->dt_ms/1000.);
    err |= mtr_set(kf->F, 2, 2, (double)1);
    err |= mtr_set(kf->F, 3, 3, (double)1);
    if (err) return err;
    
    /* control input initialized as identity */
    kf->B  = mtr_alloc(4,4);
    if (kf->B == NULL) {
        return ERR_MALLOC;
    }
    err |= mtr_set(kf->B, 0, 0, (double)1);
    err |= mtr_set(kf->B, 1, 1, (double)1);
    err |= mtr_set(kf->B, 2, 2, (double)1);
    err |= mtr_set(kf->B, 3, 3, (double)1);
    if (err) return err;
    
    kf->u  = mtr_alloc(4,1);
    if (kf->u == NULL) {
        return ERR_MALLOC;
    }
    
    kf->H  = mtr_alloc(4,4);
    if (kf->H == NULL) {
        return ERR_MALLOC;
    }
    err |= mtr_set(kf->H, 0, 0, (double)1);
    err |= mtr_set(kf->H, 1, 1, (double)1);
    err |= mtr_set(kf->H, 2, 2, (double)1);
    err |= mtr_set(kf->H, 3, 3, (double)1);
    kf->H->type = MTR_ID;
    if (err) return err;
    
    
    /* measurement noise is 1.5 cm */
    kf->w  = mtr_alloc(4,1);
    mtr_set(kf->w, 0, 0, W_NOISE);
    mtr_set(kf->w, 1, 0, W_NOISE);
    mtr_set(kf->w, 2, 0, W_NOISE);
    mtr_set(kf->w, 3, 0, W_NOISE);
    
    if (kf->w == NULL) {
        return ERR_MALLOC;
    }
    

    /**********************************************/
    /********************* Q, R *******************/
    kf->Q  = mtr_alloc(4,4);
    if (kf->Q == NULL) {
        return ERR_MALLOC;
    }
    err |= mtr_set(kf->Q, 0, 0, (double)TRANS_NOISE);
    err |= mtr_set(kf->Q, 1, 1, (double)TRANS_NOISE);
    err |= mtr_set(kf->Q, 2, 2, (double)TRANS_NOISE);
    err |= mtr_set(kf->Q, 3, 3, (double)TRANS_NOISE);
    kf->Q->type = MTR_DIAG;

    kf->R  = mtr_alloc(4,4);
    if (kf->R == NULL) {
        return ERR_MALLOC;
    }
    err |= mtr_set(kf->R, 0, 0, (double)MEAS_NOISE);
    err |= mtr_set(kf->R, 1, 1, (double)MEAS_NOISE);
    err |= mtr_set(kf->R, 2, 2, (double)MEAS_NOISE);
    err |= mtr_set(kf->R, 3, 3, (double)MEAS_NOISE);
    kf->R->type = MTR_DIAG;
    /***********************************************/


    
    kf->P  = mtr_alloc(4,4);
    kf->P_ = mtr_alloc(4,4);
    if (kf->P == NULL || kf->P_ == NULL) {
        return ERR_MALLOC;
    }
    err |= mtr_set(kf->P, 0, 0, (double)1600);
    err |= mtr_set(kf->P, 1, 1, (double)1600);
    err |= mtr_set(kf->P, 2, 2, (double)1600);
    err |= mtr_set(kf->P, 3, 3, (double)1600);
    
    kf->K  = mtr_alloc(4,4);
    if (kf->K == NULL) {
        return ERR_MALLOC;
    }

    kf->z  = mtr_alloc(4,1);
    kf->dz  = mtr_alloc(4,1);
    if (kf->z == NULL || kf->dz == NULL) {
        return ERR_MALLOC;
    }
    
    kf->dummy1_41 = mtr_alloc(4,1);
    kf->dummy2_41 = mtr_alloc(4,1);
    kf->dummy1_44 = mtr_alloc(4,4);
    kf->dummy2_44 = mtr_alloc(4,4);
    kf->dummy1_14 = mtr_alloc(1,4);
    kf->dummy2_14 = mtr_alloc(1,4);
    if (kf->dummy1_44 == NULL || kf->dummy1_14 == NULL || kf->dummy1_41 == NULL ||
        kf->dummy2_44 == NULL || kf->dummy2_14 == NULL || kf->dummy2_41 == NULL) {
        return ERR_MALLOC;
    }
        
    return err;
}
void kf_set_trans_noise(kalman_filter_t * const kf, const double val)
{
    kf->transition_noise = val;
}

void kf_set_meas_noise(kalman_filter_t * const kf, const double val)
{
    kf->measurement_noise = val;
}

error_t kf_predict_state(kalman_filter_t* const kf)
{
    error_t err = ERR_NOERROR;
    /* predict state: x_ = Fx(prev) + Bu + w */
    err |= mtr_product(kf->F, kf->_x, kf->x_);
    if (err) return err;
#if 0
    err |= mtr_product_ (kf->B, kf->u, kf->dummy1_41);
    if (err) return err;
    err |= mtr_add_(kf->dummy1_41, kf->w, kf->dummy2_41);
    if (err) return err;
    err |= mtr_add_(kf->dummy1_41, kf->dummy2_41, kf->x_);
    if (err) return err;
#endif    
    return ERR_NOERROR;
}

error_t kf_predict_cov(kalman_filter_t* const kf)
{
    error_t err = ERR_NOERROR;
    
    /* predict error covariance P_ = F*P*F' + Q */
    err |= mtr_product_withT(kf->P, kf->F, kf->dummy1_44);
    if (err) return err;
    err |= mtr_product (kf->F, kf->dummy1_44, kf->dummy2_44);
    if (err) return err;
    err |= mtr_add(kf->dummy2_44, kf->Q, kf->P_);
    if (err) return err;
    
    return ERR_NOERROR;
}

error_t kf_adjust_gain(kalman_filter_t* const kf)
{
    error_t err = ERR_NOERROR;
    
    err |= mtr_product_withT(kf->P_, kf->H, kf->dummy1_44);
    if (err) return err;
    err |= mtr_product(kf->H, kf->dummy1_44, kf->dummy2_44);
    if (err) return err;
    err |= mtr_add(kf->dummy2_44, kf->R, kf->K);
    if (err) return err;
    err |= mtr_inverse4x4(kf->K, kf->dummy2_44);
    if (err) return err;
    err |= mtr_product(kf->dummy1_44, kf->dummy2_44, kf->K);
    if (err) return err;

    return ERR_NOERROR;
}

error_t kf_adjust_state(kalman_filter_t* const kf)
{
    error_t err = ERR_NOERROR;
    err |= mtr_product(kf->H, kf->_x, kf->dummy1_41);
    err |= mtr_foreach(kf->dummy1_41, &change_sign);
    err |= mtr_add(kf->z, kf->dummy1_41, kf->dz);
    if (err) return err;
    err |= mtr_product(kf->K, kf->dz, kf->dummy2_41);
    if (err) return err;
    err |= mtr_add(kf->x_, kf->dummy2_41, kf->x);
    if (err) return err;
    
    return ERR_NOERROR;
}

error_t kf_adjust_cov(kalman_filter_t* const kf)
{
    error_t err = ERR_NOERROR;
    err |= mtr_product(kf->H, kf->P_, kf->dummy1_44);
    if (err) return err;
    err |= mtr_product(kf->K, kf->dummy1_44, kf->dummy2_44);
    if (err) return err;
    err |= mtr_foreach(kf->dummy2_44, &change_sign);
    err |= mtr_add(kf->P_, kf->dummy2_44, kf->P);
    return err;
}

error_t kf_update(kalman_filter_t* const kf, const kf_state_t* const z)
{
    error_t err = ERR_NOERROR;
    /* backup previous estimated state */
    memcpy(kf->_x->data, kf->x->data, sizeof(double)*kf->_x->nrows);
    
    /* set current measured state */
    err |= mtr_set(kf->z, 0, 0, (double)(z->x));
    err |= mtr_set(kf->z, 1, 0, (double)(z->y));
    err |= mtr_set(kf->z, 2, 0, (double)(z->vx));
    err |= mtr_set(kf->z, 3, 0, (double)(z->vy));
    if (err) return err;

    /* First run, initialize state with input */
    if (kf->counter == 0) {
	memcpy(kf->_x->data, kf->z->data, sizeof(double)*kf->_x->nrows);
	memcpy(kf->x->data,  kf->z->data, sizeof(double)*kf->_x->nrows);
	memcpy(kf->x_->data, kf->z->data, sizeof(double)*kf->_x->nrows);
    }
    
    /* predict state */
    err |= kf_predict_state(kf);
    if (err) return err;

    /* predict error covariance */
    err |= kf_predict_cov(kf);
    if (err) return err;
    //mtr_print(kf->P_);
    
    /* compute Kalman gain from predicted cov */
    err |= kf_adjust_gain(kf);
    if (err) return err;
    //mtr_print(kf->K);

    /* compute state estimate using both predicted and measured state */
    err |= kf_adjust_state(kf);
    if (err) return err;

    /* compute error covariance */
    err |= kf_adjust_cov(kf);
    if (err) return err;
    //    mtr_print(kf->P);

    kf->counter++;
    if ( kf->counter > 50
	 && fabs(z->x - mtr_get(kf->x_, 0, 0))
	 +  fabs(z->y - mtr_get(kf->x_, 1, 0)) < KF_STEADY_DELTA ) {
	kf->steady_cnt++;
    } else {
	kf->steady_cnt = 0;
	kf->steady = 0;
    }

    if (kf->steady_cnt > KF_SAMPLES_STEADY) {
	kf->steady = 1;
    }

    return ERR_NOERROR;
}

error_t kf_is_steady(kalman_filter_t const* kf)
{
    return kf->steady;
}

void kf_free(kalman_filter_t* kf)
{
    mtr_free(kf->x);
    mtr_free(kf->F);
    mtr_free(kf->B);
    mtr_free(kf->u);
    mtr_free(kf->H);
    mtr_free(kf->K);
    mtr_free(kf->w);
    mtr_free(kf->Q);
    mtr_free(kf->P);
    mtr_free(kf->R);
    mtr_free(kf->x_);
    mtr_free(kf->_x);
    mtr_free(kf->P_);
    mtr_free(kf->z);
    mtr_free(kf->dz);
    mtr_free(kf->dummy1_41);
    mtr_free(kf->dummy1_44);
    mtr_free(kf->dummy1_14);
    mtr_free(kf->dummy2_41);
    mtr_free(kf->dummy2_44);
    mtr_free(kf->dummy2_14);
    //free(kf);
    kf=NULL;
}
