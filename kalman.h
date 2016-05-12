/* Last modified: <08-Oct-2015 17:19:42 CEST by Dmitry Ebel> */
#ifndef KALMAN_H
#define KALMAN_H

#include "position.h"
#include "matrix.h"
#include "errors.h"

#define DT_MS 164
#define TRANS_NOISE   0.1
#define MEAS_NOISE    100000.0
#define W_NOISE       2.0
#define KF_STEADY_DELTA      4
#define KF_SAMPLES_STEADY    10 

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct {
	cm_t x;
	cm_t y;
	cmps_t vx;
	cmps_t vy;
    } kf_state_t;
    
    typedef struct {
	int16_t  dt_ms;
	uint64_t counter;
	uint8_t  steady;
	uint16_t steady_cnt;
	double   transition_noise;
	double   measurement_noise;
	double   w_noise;
	matrix_t* x; /* estimated sensor state */
	matrix_t* F; /* transition matrix */
	matrix_t* B; /* control input matrix */
	matrix_t* u; /* control inputs */
	matrix_t* H; /* transform matrix state domain -> measurement domain */
	matrix_t* K; /* Kalman gain */
	matrix_t* w; /* process (=transition) noise */
	matrix_t* Q; /* process noise covariance matrix (used for prediction) */
	matrix_t* P; /* estimated covariance matrix */
	matrix_t* R; /* measurement noise covariance matrix (used for estimation/adjustment)*/

	matrix_t* x_; /* predicted state */
	matrix_t* _x; /* previous state */
	matrix_t* P_; /* predicted covariance matrix */

	matrix_t* z;  /* measured state */
	matrix_t* dz; /* measurement error */

	matrix_t* dummy1_41;
	matrix_t* dummy1_44;
	matrix_t* dummy1_14;
	matrix_t* dummy2_41;
	matrix_t* dummy2_44;
	matrix_t* dummy2_14;

    } kalman_filter_t;

    error_t kf_init(kalman_filter_t* kf);
    error_t kf_update(kalman_filter_t* const kf, const kf_state_t* const z);
    error_t kf_predict_state(kalman_filter_t* const kf);
    error_t kf_predict_cov(kalman_filter_t* const kf);
    error_t kf_adjust_state(kalman_filter_t* const kf);
    error_t kf_adjust_gain(kalman_filter_t* const kf);
    error_t kf_adjust_cov(kalman_filter_t* const kf);
    void    kf_free(kalman_filter_t* kf);
    error_t kf_is_steady(kalman_filter_t const* kf);
    void    kf_set_trans_noise(kalman_filter_t * const kf, const double val);
    void    kf_set_meas_noise(kalman_filter_t * const kf, const double val);
    
#ifdef __cplusplus
}
#endif

#endif
