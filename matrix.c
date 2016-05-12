/* Last modified: <21-Oct-2015 15:21:20 CEST by Dmitry Ebel> */
#include "matrix.h"

#ifndef __SAM3X8E__
#include <assert.h>
#endif

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

matrix_t* mtr_alloc(const uint8_t nrows, const uint8_t ncols)
{
    matrix_t* res = NULL;
    res = (matrix_t*)malloc(sizeof(matrix_t));
    if(res == NULL) {
	return res;
    }
    res->type  = 0;
    res->ncols = ncols;
    res->nrows = nrows;
    res->data  = (double*)malloc(ncols*nrows*sizeof(double));
    if(res->data == NULL) {
	free(res);
	res = NULL;
	return NULL;
    }
    memset((void*)(res->data), 0, ncols*nrows*sizeof(double));
    return res;
}

void mtr_free(matrix_t* mtr)
{
    free(mtr->data);
    mtr->data=NULL;
    free(mtr);
    mtr=NULL;
}

error_t mtr_set(matrix_t* m, const uint8_t row, const uint8_t col, const double val)
{
#ifndef __SAM3X8E__
    assert((m->nrows > row) && (m->ncols > col));
#endif
    *(m->data + m->ncols*row + col) = val;
    return ERR_NOERROR;
}

double mtr_get(const matrix_t* m, const uint8_t row, const uint8_t col)
{
#ifndef __SAM3X8E__
    assert(m->nrows>row && m->ncols>col);
#endif
    
    return *(m->data + m->ncols*row + col);
}

/* A -> A' with pre-allocated result */
error_t mtr_transpose(const matrix_t* m, matrix_t* const res)
{
#ifndef __SAM3X8E__
    assert(m->nrows*m->ncols > 0);
#endif
    if ( !(m->nrows*m->ncols > 0) ) {
        return ERR_MTR_DIM;
    }

    if ( !( m->nrows == res->ncols && m->ncols == res->nrows) ) {
        return ERR_MTR_DIM;
    }
    
    uint8_t ccnt, rcnt;

    /* just copy if m is diagonal */
    if (m->type & MTR_DIAG) {
	memcpy((void*)(res->data), (void*)m->data, res->ncols*res->nrows*sizeof(double));
	res->type |= MTR_DIAG;
	return ERR_NOERROR;
    }
    
    for (rcnt=0; rcnt<res->nrows; rcnt++) {
        for (ccnt=0; ccnt<res->ncols; ccnt++) {
            mtr_set(res, rcnt, ccnt, mtr_get(m, ccnt, rcnt));
        }
    }
    return ERR_NOERROR;
}

/* A -> A' */
matrix_t* mtr_transpose_(const matrix_t* m)
{
    assert(m->nrows*m->ncols > 0);
    matrix_t* res = mtr_alloc(m->ncols, m->nrows);
    uint8_t ccnt, rcnt;
    for (rcnt=0; rcnt<res->nrows; rcnt++) {
	for (ccnt=0; ccnt<res->ncols; ccnt++) {
	    mtr_set(res, rcnt, ccnt, mtr_get(m, ccnt, rcnt));
	}
    }
    return res;
}

error_t mtr_add(const matrix_t* const left, const matrix_t* const right, matrix_t* const res)
{
#ifndef __SAM3X8E__
    assert(left->ncols == right->ncols && left->nrows == right->nrows);
    assert(left->nrows == res->nrows   && left->ncols == res->ncols);
#endif
    if ( left->ncols != right->ncols || left->nrows != right->nrows) {
        return ERR_MTR_DIM;
    }
    
    if ( left->nrows != res->nrows || left->ncols != res->ncols) {
        return ERR_MTR_DIM;
    }
    
    //TODO: optimize if possible
    uint8_t ccnt, rcnt;
    for (rcnt=0; rcnt<left->nrows; rcnt++) {
        for (ccnt=0; ccnt<left->ncols; ccnt++) {
            mtr_set(res, rcnt, ccnt,
                    mtr_get(left, rcnt, ccnt) 
                    + mtr_get(right, rcnt, ccnt));
        }
    }
    
    return ERR_NOERROR;
}

/* A,B -> AB with pre-allocated result */
error_t mtr_product(const matrix_t* const left, const matrix_t* const right, matrix_t* const res)
{
#ifndef __SAM3X8E__
    assert(left->ncols == right->nrows);
    assert(left->nrows == res->nrows);
    assert(right->ncols == res->ncols);
#endif
    memset((void*)(res->data), 0, res->ncols*res->nrows*sizeof(double));
    if (left->ncols != right->nrows) {
        return ERR_MTR_DIM;
    }
    if ( !( left->nrows == res->nrows && right->ncols == res->ncols)) {
        return ERR_MTR_DIM;
    }
    
    //TODO: optimize if possible
    uint8_t ccnt, rcnt, i;
    double dummy;
    for (rcnt=0; rcnt<res->nrows; rcnt++) {
        for (ccnt=0; ccnt<res->ncols; ccnt++) {
            for (i=0; i<left->ncols; i++) {
		dummy = mtr_get(res, rcnt, ccnt)
		    + mtr_get(left, rcnt, i)
		    * mtr_get(right, i, ccnt);
		if (isnan(dummy)) {
		    return ERR_NOTANUMBER;
		} else { 
		    mtr_set(res, rcnt, ccnt, dummy);
		}
            }
        }
    }
    
    return ERR_NOERROR;
}

/* A,B -> AB */
matrix_t* mtr_product_(const matrix_t* left, const matrix_t* right)
{
    assert(left->ncols == right->nrows);
    matrix_t* res = mtr_alloc(left->nrows, right->ncols);

    //TODO: optimize if possible
    uint8_t ccnt, rcnt, i;
    double dummy;
    for (rcnt=0; rcnt<res->nrows; rcnt++) {
	for (ccnt=0; ccnt<res->ncols; ccnt++) {
	    for (i=0; i<left->ncols; i++) {
		dummy = mtr_get(res, rcnt, ccnt)
		    + mtr_get(left, rcnt, i)
		    * mtr_get(right, i, ccnt);
		if (isnan(dummy)) {
		    mtr_free(res);
		    return NULL;
		} else { 
		    mtr_set(res, rcnt, ccnt, dummy);
		}
	    }
	}
    }

    return res;
}

error_t  mtr_apply_to_pos2d(const matrix_t* const m, const position2d_t* const v2d, position2d_t* const res)
{
#ifndef __SAM3X8E__
    assert(m->ncols  == 2);
    assert(m->nrows  == 2);
#endif
    if (m->ncols != 2) {
        return ERR_MTR_DIM;
    }
    if (m->nrows != 2) {
        return ERR_MTR_DIM;
    }

    error_t err = ERR_NOERROR;
    res->x = mtr_get(m,0,0) * v2d->x + mtr_get(m,0,1) * v2d->y;
    res->y = mtr_get(m,1,0) * v2d->x + mtr_get(m,1,1) * v2d->y;

    return err;
}

/* A,B -> AB' */
error_t mtr_product_withT(const matrix_t* const left, const matrix_t* const right, matrix_t* const res)
{
#ifndef __SAM3X8E__
    assert(left->ncols  == right->ncols);
    assert(left->nrows  == res->nrows);
    assert(right->ncols == res->ncols);
#endif
    if (left->ncols != right->ncols) {
        return ERR_MTR_DIM;
    }
    if (left->nrows != right->nrows) {
        return ERR_MTR_DIM;
    }
    if ( left->nrows != res->nrows) {
        return ERR_MTR_DIM;
    }
    if ( left->ncols != res->ncols) {
        return ERR_MTR_DIM;
    }

    memset((void*)(res->data), 0, res->ncols*res->nrows*sizeof(double));

    //TODO: optimize if possible
    uint8_t ccnt, rcnt, i;
    for (rcnt=0; rcnt<res->nrows; rcnt++) {
        for (ccnt=0; ccnt<res->ncols; ccnt++) {
            for (i=0; i<left->ncols; i++) {
                mtr_set(res, rcnt, ccnt,
                        mtr_get(res, rcnt, ccnt)
                        + mtr_get(left,  rcnt, i)
                        * mtr_get(right, ccnt, i));
            }
        }
    }

    return ERR_NOERROR;
}

matrix_t* mtr_inverse2x2_(const matrix_t* const m)
{
    assert(m->nrows == 2 && m->ncols == 2);
    matrix_t* res = mtr_alloc(2,2);

    double det = mtr_get(m, 0,0)*mtr_get(m, 1,1)
	- mtr_get(m, 1,0)*mtr_get(m, 0,1);

    det = 1/det;
    
    mtr_set(res, 0,0,  det*mtr_get(m, 1,1));
    mtr_set(res, 1,0, -det*mtr_get(m, 0,1));
    mtr_set(res, 0,1, -det*mtr_get(m, 1,0));
    mtr_set(res, 1,1,  det*mtr_get(m, 0,0));
    
    return res;
}

error_t mtr_inverse2x2(const matrix_t* const m, matrix_t* const res)
{
    if (m->nrows != 2
        || m->ncols != 2
        || res->nrows != 2
        || res->ncols != 2) {
        return ERR_MTR_DIM;
    }

    /* if diagonal matrix*/
    if (0/*m->type &  MTR_DIAG*/) {
	memset((void*)(res->data), 0, res->nrows*res->ncols*sizeof(double));
	mtr_set(res, 0, 0, 1./mtr_get(m,0,0));
	mtr_set(res, 1, 1, 1./mtr_get(m,1,1));
	res->type = MTR_DIAG;
	return ERR_NOERROR;
    }
    
    double det = mtr_get(m, 0,0)*mtr_get(m, 1,1)
	- mtr_get(m, 1,0)*mtr_get(m, 0,1);
    
    det = 1/det;
    
    mtr_set(res, 0,0,  det*mtr_get(m, 1,1));
    mtr_set(res, 1,0, -det*mtr_get(m, 0,1));
    mtr_set(res, 0,1, -det*mtr_get(m, 1,0));
    mtr_set(res, 1,1,  det*mtr_get(m, 0,0));
    
    return ERR_NOERROR;
}

error_t mtr_inverse3x3(const matrix_t* const m, matrix_t* const res)
{
    if (m->nrows != 3
        || m->ncols != 3
        || res->nrows != 3
        || res->ncols != 3) {
        return ERR_MTR_DIM;
    }

    /* if diagonal matrix*/
    if (m->type &  MTR_DIAG) {
	memset((void*)res->data, 0, res->nrows*res->ncols*sizeof(double));
	mtr_set(res, 0, 0, 1./mtr_get(m,0,0));
	mtr_set(res, 1, 1, 1./mtr_get(m,1,1));
	mtr_set(res, 2, 2, 1./mtr_get(m,2,2));
	res->type = MTR_DIAG;
	return ERR_NOERROR;
    }
        
    double det
        = mtr_get(m,0,0) * mtr_get(m,1,1) * mtr_get(m,2,2)
        + mtr_get(m,0,1) * mtr_get(m,1,2) * mtr_get(m,2,0)
        + mtr_get(m,0,2) * mtr_get(m,1,0) * mtr_get(m,2,1)
        - mtr_get(m,0,0) * mtr_get(m,1,2) * mtr_get(m,2,1)
        - mtr_get(m,0,1) * mtr_get(m,1,0) * mtr_get(m,2,2)
        - mtr_get(m,0,2) * mtr_get(m,1,1) * mtr_get(m,2,0);
        
    if (fabs(det) < 1e-6) {
	return ERR_NOTANUMBER;
    }
        
    det = 1./det;
        
    mtr_set(res, 0,0, det*(mtr_get(m,1,1)*mtr_get(m,2,2)
			   - mtr_get(m,1,2)*mtr_get(m,2,1)));
    mtr_set(res, 0,1, det*(mtr_get(m,0,2)*mtr_get(m,2,1)
			   - mtr_get(m,0,1)*mtr_get(m,2,2)));
    mtr_set(res, 0,2, det*(mtr_get(m,0,1)*mtr_get(m,1,2)
			   - mtr_get(m,0,2)*mtr_get(m,1,1)));
    mtr_set(res, 1,0, det*(mtr_get(m,1,2)*mtr_get(m,2,0)
			   - mtr_get(m,1,0)*mtr_get(m,2,2)));
    mtr_set(res, 1,1, det*(mtr_get(m,0,0)*mtr_get(m,2,2)
			   - mtr_get(m,0,2)*mtr_get(m,2,0)));
    mtr_set(res, 1,2, det*(mtr_get(m,0,2)*mtr_get(m,1,0)
			   - mtr_get(m,0,0)*mtr_get(m,1,2)));
    mtr_set(res, 2,0, det*(mtr_get(m,1,0)*mtr_get(m,2,1)
			   - mtr_get(m,1,1)*mtr_get(m,2,0)));
    mtr_set(res, 2,1, det*(mtr_get(m,0,1)*mtr_get(m,2,0)
			   - mtr_get(m,0,0)*mtr_get(m,2,1)));
    mtr_set(res, 2,2, det*(mtr_get(m,0,0)*mtr_get(m,1,1)
			   - mtr_get(m,0,1)*mtr_get(m,1,0)));
        
    return ERR_NOERROR;
}

error_t mtr_inverse4x4(const matrix_t* const m, matrix_t* const res)
{
    if (m->nrows != 4
        || m->ncols != 4
        || res->nrows != 4
        || res->ncols != 4) {
        return ERR_MTR_DIM;
    }

    /* if diagonal matrix*/
    if (m->type &  MTR_DIAG) {
	memset((void*)res->data, 0, res->nrows*res->ncols*sizeof(double));
	mtr_set(res, 0, 0, 1./mtr_get(m,0,0));
	mtr_set(res, 1, 1, 1./mtr_get(m,1,1));
	mtr_set(res, 2, 2, 1./mtr_get(m,2,2));
	mtr_set(res, 3, 3, 1./mtr_get(m,3,3));
	res->type = MTR_DIAG;
	return ERR_NOERROR;
    }

    error_t err = ERR_NOERROR;
	
    double det
	= mtr_get(m,0,0) * mtr_get(m,1,1) * mtr_get(m,2,2) * mtr_get(m,3,3)
	+ mtr_get(m,0,0) * mtr_get(m,1,2) * mtr_get(m,2,3) * mtr_get(m,3,1)
	+ mtr_get(m,0,0) * mtr_get(m,1,3) * mtr_get(m,2,1) * mtr_get(m,3,2)
    
	+ mtr_get(m,0,1) * mtr_get(m,1,0) * mtr_get(m,2,3) * mtr_get(m,3,2)
	+ mtr_get(m,0,1) * mtr_get(m,1,2) * mtr_get(m,2,0) * mtr_get(m,3,3)
	+ mtr_get(m,0,1) * mtr_get(m,1,3) * mtr_get(m,2,2) * mtr_get(m,3,0)
    
	+ mtr_get(m,0,2) * mtr_get(m,1,0) * mtr_get(m,2,1) * mtr_get(m,3,3)
	+ mtr_get(m,0,2) * mtr_get(m,1,1) * mtr_get(m,2,3) * mtr_get(m,3,0)
	+ mtr_get(m,0,2) * mtr_get(m,1,3) * mtr_get(m,2,0) * mtr_get(m,3,1)
    
	+ mtr_get(m,0,3) * mtr_get(m,1,0) * mtr_get(m,2,2) * mtr_get(m,3,1)
	+ mtr_get(m,0,3) * mtr_get(m,1,1) * mtr_get(m,2,0) * mtr_get(m,3,2)
	+ mtr_get(m,0,3) * mtr_get(m,1,2) * mtr_get(m,2,1) * mtr_get(m,3,0)
    
	- mtr_get(m,0,0) * mtr_get(m,1,1) * mtr_get(m,2,3) * mtr_get(m,3,2)
	- mtr_get(m,0,0) * mtr_get(m,1,2) * mtr_get(m,2,1) * mtr_get(m,3,3)
	- mtr_get(m,0,0) * mtr_get(m,1,3) * mtr_get(m,2,2) * mtr_get(m,3,1)
    
	- mtr_get(m,0,1) * mtr_get(m,1,0) * mtr_get(m,2,2) * mtr_get(m,3,3)
	- mtr_get(m,0,1) * mtr_get(m,1,2) * mtr_get(m,2,3) * mtr_get(m,3,0)
	- mtr_get(m,0,1) * mtr_get(m,1,3) * mtr_get(m,2,0) * mtr_get(m,3,2)
    
	- mtr_get(m,0,2) * mtr_get(m,1,0) * mtr_get(m,2,3) * mtr_get(m,3,1)
	- mtr_get(m,0,2) * mtr_get(m,1,1) * mtr_get(m,2,0) * mtr_get(m,3,3)
	- mtr_get(m,0,2) * mtr_get(m,1,3) * mtr_get(m,2,1) * mtr_get(m,3,0)
    
	- mtr_get(m,0,3) * mtr_get(m,1,0) * mtr_get(m,2,1) * mtr_get(m,3,2)
	- mtr_get(m,0,3) * mtr_get(m,1,1) * mtr_get(m,2,2) * mtr_get(m,3,0)
	- mtr_get(m,0,3) * mtr_get(m,1,2) * mtr_get(m,2,0) * mtr_get(m,3,1);
    
    if (fabs(det) < 1e-6) {
        return ERR_NOTANUMBER;
    }
    
    det = 1./det;
    
    mtr_set(res,0,0,
	    det * (mtr_get(m,1,2) * mtr_get(m,2,3) * mtr_get(m,3,1) 
		  - mtr_get(m,1,3) * mtr_get(m,2,2) * mtr_get(m,3,1) 
		  + mtr_get(m,1,3) * mtr_get(m,2,1) * mtr_get(m,3,2)
		  - mtr_get(m,1,1) * mtr_get(m,2,3) * mtr_get(m,3,2)
		  - mtr_get(m,1,2) * mtr_get(m,2,1) * mtr_get(m,3,3)
		  + mtr_get(m,1,1) * mtr_get(m,2,2) * mtr_get(m,3,3)));
    mtr_set(res,0,1,
	    det * (mtr_get(m,0,3) * mtr_get(m,2,2) * mtr_get(m,3,1) 
		   - mtr_get(m,0,2) * mtr_get(m,2,3) * mtr_get(m,3,1)
		   - mtr_get(m,0,3) * mtr_get(m,2,1) * mtr_get(m,3,2)
		   + mtr_get(m,0,1) * mtr_get(m,2,3) * mtr_get(m,3,2)
		   + mtr_get(m,0,2) * mtr_get(m,2,1) * mtr_get(m,3,3)
		   - mtr_get(m,0,1) * mtr_get(m,2,2) * mtr_get(m,3,3)));
    mtr_set(res,0,2,
	    det * (mtr_get(m,0,2) * mtr_get(m,1,3) * mtr_get(m,3,1)
		   - mtr_get(m,0,3) * mtr_get(m,1,2) * mtr_get(m,3,1)
		   + mtr_get(m,0,3) * mtr_get(m,1,1) * mtr_get(m,3,2)
		   - mtr_get(m,0,1) * mtr_get(m,1,3) * mtr_get(m,3,2)
		   - mtr_get(m,0,2) * mtr_get(m,1,1) * mtr_get(m,3,3)
		   + mtr_get(m,0,1) * mtr_get(m,1,2) * mtr_get(m,3,3)));
    mtr_set(res,0,3,
	    det * (mtr_get(m,0,3) * mtr_get(m,1,2) * mtr_get(m,2,1)
		   - mtr_get(m,0,2) * mtr_get(m,1,3) * mtr_get(m,2,1)
		   - mtr_get(m,0,3) * mtr_get(m,1,1) * mtr_get(m,2,2)
		   + mtr_get(m,0,1) * mtr_get(m,1,3) * mtr_get(m,2,2)
		   + mtr_get(m,0,2) * mtr_get(m,1,1) * mtr_get(m,2,3)
		   - mtr_get(m,0,1) * mtr_get(m,1,2) * mtr_get(m,2,3)));
    mtr_set(res,1,0,
	    det * (mtr_get(m,1,3) * mtr_get(m,2,2) * mtr_get(m,3,0)
		   - mtr_get(m,1,2) * mtr_get(m,2,3) * mtr_get(m,3,0)
		   - mtr_get(m,1,3) * mtr_get(m,2,0) * mtr_get(m,3,2)
		   + mtr_get(m,1,0) * mtr_get(m,2,3) * mtr_get(m,3,2)
		   + mtr_get(m,1,2) * mtr_get(m,2,0) * mtr_get(m,3,3)
		   - mtr_get(m,1,0) * mtr_get(m,2,2) * mtr_get(m,3,3)));
    mtr_set(res,1,1,
	    det * (mtr_get(m,0,2) * mtr_get(m,2,3) * mtr_get(m,3,0)
		   - mtr_get(m,0,3) * mtr_get(m,2,2) * mtr_get(m,3,0)
		   + mtr_get(m,0,3) * mtr_get(m,2,0) * mtr_get(m,3,2)
		   - mtr_get(m,0,0) * mtr_get(m,2,3) * mtr_get(m,3,2)
		   - mtr_get(m,0,2) * mtr_get(m,2,0) * mtr_get(m,3,3)
		   + mtr_get(m,0,0) * mtr_get(m,2,2) * mtr_get(m,3,3)));
    mtr_set(res,1,2,
	    det * (mtr_get(m,0,3) * mtr_get(m,1,2) * mtr_get(m,3,0)
		   - mtr_get(m,0,2) * mtr_get(m,1,3) * mtr_get(m,3,0)
		   - mtr_get(m,0,3) * mtr_get(m,1,0) * mtr_get(m,3,2)
		   + mtr_get(m,0,0) * mtr_get(m,1,3) * mtr_get(m,3,2)
		   + mtr_get(m,0,2) * mtr_get(m,1,0) * mtr_get(m,3,3)
		   - mtr_get(m,0,0) * mtr_get(m,1,2) * mtr_get(m,3,3)));
    mtr_set(res,1,3,
	    det * (mtr_get(m,0,2) * mtr_get(m,1,3) * mtr_get(m,2,0)
		   - mtr_get(m,0,3) * mtr_get(m,1,2) * mtr_get(m,2,0)
		   + mtr_get(m,0,3) * mtr_get(m,1,0) * mtr_get(m,2,2)
		   - mtr_get(m,0,0) * mtr_get(m,1,3) * mtr_get(m,2,2)
		   - mtr_get(m,0,2) * mtr_get(m,1,0) * mtr_get(m,2,3)
		   + mtr_get(m,0,0) * mtr_get(m,1,2) * mtr_get(m,2,3)));
    mtr_set(res,2,0,
	    det * (mtr_get(m,1,1) * mtr_get(m,2,3) * mtr_get(m,3,0)
		   - mtr_get(m,1,3) * mtr_get(m,2,1) * mtr_get(m,3,0)
		   + mtr_get(m,1,3) * mtr_get(m,2,0) * mtr_get(m,3,1)
		   - mtr_get(m,1,0) * mtr_get(m,2,3) * mtr_get(m,3,1)
		   - mtr_get(m,1,1) * mtr_get(m,2,0) * mtr_get(m,3,3)
		   + mtr_get(m,1,0) * mtr_get(m,2,1) * mtr_get(m,3,3)));
    mtr_set(res,2,1,
	    det * (mtr_get(m,0,3) * mtr_get(m,2,1) * mtr_get(m,3,0) 
		   - mtr_get(m,0,1) * mtr_get(m,2,3) * mtr_get(m,3,0)
		   - mtr_get(m,0,3) * mtr_get(m,2,0) * mtr_get(m,3,1)
		   + mtr_get(m,0,0) * mtr_get(m,2,3) * mtr_get(m,3,1)
		   + mtr_get(m,0,1) * mtr_get(m,2,0) * mtr_get(m,3,3)
		   - mtr_get(m,0,0) * mtr_get(m,2,1) * mtr_get(m,3,3)));
    mtr_set(res,2,2,
	    det * (mtr_get(m,0,1) * mtr_get(m,1,3) * mtr_get(m,3,0)
		   - mtr_get(m,0,3) * mtr_get(m,1,1) * mtr_get(m,3,0)
		   + mtr_get(m,0,3) * mtr_get(m,1,0) * mtr_get(m,3,1)
		   - mtr_get(m,0,0) * mtr_get(m,1,3) * mtr_get(m,3,1)
		   - mtr_get(m,0,1) * mtr_get(m,1,0) * mtr_get(m,3,3)
		   + mtr_get(m,0,0) * mtr_get(m,1,1) * mtr_get(m,3,3)));
    mtr_set(res,2,3,
	    det * (mtr_get(m,0,3) * mtr_get(m,1,1) * mtr_get(m,2,0)
		   - mtr_get(m,0,1) * mtr_get(m,1,3) * mtr_get(m,2,0)
		   - mtr_get(m,0,3) * mtr_get(m,1,0) * mtr_get(m,2,1)
		   + mtr_get(m,0,0) * mtr_get(m,1,3) * mtr_get(m,2,1)
		   + mtr_get(m,0,1) * mtr_get(m,1,0) * mtr_get(m,2,3)
		   - mtr_get(m,0,0) * mtr_get(m,1,1) * mtr_get(m,2,3)));
    mtr_set(res,3,0,
	    det * (mtr_get(m,1,2) * mtr_get(m,2,1) * mtr_get(m,3,0)
		   - mtr_get(m,1,1) * mtr_get(m,2,2) * mtr_get(m,3,0)
		   - mtr_get(m,1,2) * mtr_get(m,2,0) * mtr_get(m,3,1)
		   + mtr_get(m,1,0) * mtr_get(m,2,2) * mtr_get(m,3,1)
		   + mtr_get(m,1,1) * mtr_get(m,2,0) * mtr_get(m,3,2)
		   - mtr_get(m,1,0) * mtr_get(m,2,1) * mtr_get(m,3,2)));
    mtr_set(res,3,1,
	    det * (mtr_get(m,0,1) * mtr_get(m,2,2) * mtr_get(m,3,0)
		   - mtr_get(m,0,2) * mtr_get(m,2,1) * mtr_get(m,3,0)
		   + mtr_get(m,0,2) * mtr_get(m,2,0) * mtr_get(m,3,1)
		   - mtr_get(m,0,0) * mtr_get(m,2,2) * mtr_get(m,3,1)
		   - mtr_get(m,0,1) * mtr_get(m,2,0) * mtr_get(m,3,2)
		   + mtr_get(m,0,0) * mtr_get(m,2,1) * mtr_get(m,3,2)));
    mtr_set(res,3,2,
	    det * (mtr_get(m,0,2) * mtr_get(m,1,1) * mtr_get(m,3,0)
		   - mtr_get(m,0,1) * mtr_get(m,1,2) * mtr_get(m,3,0)
		   - mtr_get(m,0,2) * mtr_get(m,1,0) * mtr_get(m,3,1)
		   + mtr_get(m,0,0) * mtr_get(m,1,2) * mtr_get(m,3,1)
		   + mtr_get(m,0,1) * mtr_get(m,1,0) * mtr_get(m,3,2)
		   - mtr_get(m,0,0) * mtr_get(m,1,1) * mtr_get(m,3,2)));
    mtr_set(res,3,3,
	    det * (mtr_get(m,0,1) * mtr_get(m,1,2) * mtr_get(m,2,0)
		   - mtr_get(m,0,2) * mtr_get(m,1,1) * mtr_get(m,2,0)
		   + mtr_get(m,0,2) * mtr_get(m,1,0) * mtr_get(m,2,1)
		   - mtr_get(m,0,0) * mtr_get(m,1,2) * mtr_get(m,2,1)
		   - mtr_get(m,0,1) * mtr_get(m,1,0) * mtr_get(m,2,2)
		   + mtr_get(m,0,0) * mtr_get(m,1,1) * mtr_get(m,2,2)));
    
    return err;
}

error_t mtr_foreach(matrix_t* const m, double(*cb)(const double))
{
    error_t err = ERR_NOERROR;
    uint8_t col,row;
    for (row=0; row<m->nrows; row++) {
	for (col=0; col<m->ncols; col++) {
	    mtr_set(m, row, col, (*cb)(mtr_get(m, row,col)));
	}
    }
    return err;
}

error_t mtr_make_rot2d(matrix_t* const m, const double phi)
{
    if (m->ncols !=2 || m->nrows !=2) {
	return ERR_MTR_DIM;
    }

    mtr_set(m, 0, 0, cos(phi));
    mtr_set(m, 0, 1, -sin(phi));
    mtr_set(m, 1, 0, sin(phi));
    mtr_set(m, 1, 1, cos(phi));
    return ERR_NOERROR;
}

error_t mtr_append_rows(matrix_t* const m, const matrix_t* const r)
{
    assert(m->ncols == r->ncols);
    double* newdata = (double*)malloc( (m->nrows+r->nrows)*m->ncols*sizeof(double));
    if(newdata == NULL) {
	return ERR_MALLOC;
    }
    memcpy((void*)newdata, (void*)(m->data), sizeof(double)*(m->nrows*m->ncols));
    memcpy((void*)(newdata + m->nrows*m->ncols), (void*)(r->data), sizeof(double)*(r->nrows*r->ncols));
    m->data = (double*)realloc((void*)(m->data), (m->nrows+r->nrows)*m->ncols*sizeof(double));
    if(m->data == NULL) {
	return ERR_MALLOC;
    }
    memcpy((void*)(m->data), (void*)newdata, sizeof(double)*(m->nrows+r->nrows)*m->ncols);
    m->nrows = m->nrows+r->nrows;
    return ERR_NOERROR;
}

#ifndef __SAM3X8E__
error_t mtr_print(const matrix_t* m)
{
#ifndef __SAM3X8E__
    assert(m->nrows*m->ncols > 0);
#endif
    unsigned r,c;
    
    printf("    ");
    for (c=0; c<m->ncols; c++) {
	printf("%5s%d]", "[", c);
    }
    printf("\n");
    for (r=0; r<m->nrows; r++) {
        printf("| [%d] ", r);
	for (c=0; c<m->ncols; c++) {
            printf("%7.2f", mtr_get(m,r,c));
        }
        printf("    [%d]\n", r);
    }
    printf("+-\n");
    return ERR_NOERROR;
}
#endif
