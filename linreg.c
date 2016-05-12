/* Last modified: <09-Oct-2015 11:16:09 CEST by Dmitry Ebel>
 */
#include "linreg.h"

error_t lr_init(linreg_t* const lr)
{
    lr->A   = mtr_alloc(LR_LEN,  2);
    lr->b   = mtr_alloc(LR_LEN,  1);
    lr->At  = mtr_alloc(2, LR_LEN);
    lr->Pinv = mtr_alloc(2, 2);
    lr->Ainv = mtr_alloc(2, LR_LEN);
    lr->P    = mtr_alloc(2, 2);
    lr->res  = mtr_alloc(2, 1);

    return ERR_NOERROR;
}

void lr_update_data(linreg_t* const lr, const double v)
{
    lr->data[lr->cnt] = v;
    lr->cnt++;
    lr->cnt = lr->cnt % LR_LEN;
}

error_t lr_calc(linreg_t* const lr)
{
    error_t err = ERR_NOERROR;

    lr->stddev = 0;

    uint8_t m = 0;
    for (m=0; m < LR_LEN; m++) {
	mtr_set(lr->A, m, 0, (double)m);
	mtr_set(lr->A, m, 1, 1.0);
	mtr_set(lr->b, m, 0, lr->data[(lr->cnt+m)%LR_LEN]);
    }
    err |= mtr_transpose(lr->A, lr->At);
    err |= mtr_product(lr->At, lr->A, lr->P);
    err |= mtr_inverse2x2(lr->P, lr->Pinv);
    err |= mtr_product(lr->Pinv, lr->At, lr->Ainv);
    err |= mtr_product(lr->Ainv, lr->b, lr->res);

    lr->gain   = mtr_get(lr->res, 0, 0);
    lr->offset = mtr_get(lr->res, 1, 0);

    /* Stddev */
    for (m=0; m < LR_LEN; m++) {
	double est = lr->gain * (double)m + lr->offset;
	double meas = lr->data[(lr->cnt+m)%LR_LEN];
	lr->stddev += (est - meas) * (est - meas);
    }
    lr->stddev = sqrt(lr->stddev/(double)LR_LEN);

    return err;
}
