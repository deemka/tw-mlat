/* Last modified: <09-Oct-2015 11:16:13 CEST by Dmitry Ebel>
 */

#ifndef LINREG_H
#define LINREG_H

#include "errors.h"
#include "matrix.h"
#include "math.h"

#define LR_LEN 8

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct linreg_t {
	double data[LR_LEN];
	double gain;
	double offset;
	double stddev;
	uint8_t cnt;
	matrix_t* A;
	matrix_t* b;
	matrix_t* At;
	matrix_t* Pinv;
	matrix_t* Ainv;
	matrix_t* P;
	matrix_t* res;
    } linreg_t;

    error_t lr_init(linreg_t* const);
    error_t lr_calc(linreg_t* const);
    void    lr_update_data(linreg_t* const, const double);
#ifdef __cplusplus
}
#endif

#endif
