/* Last modified: <08-Sep-2015 13:08:56 CEST by Dmitry Ebel> */
#include "position.h"
#include "matrix.h"
#include <stdio.h>

error_t pos_mlat2d(const position2d_t* const positions,
		   const cm_t* const distances,
		   const uint8_t count,
		   position2d_t* const respos)
{
    if (count < 3) {
	return ERR_ARRAY_DIM;
    }
	
    error_t err = ERR_NOERROR;

    matrix_t* A  = mtr_alloc(count-1,2);
    matrix_t* b  = mtr_alloc(count-1,1);
    matrix_t* At = mtr_alloc(2, count-1);
    matrix_t* Pinv = mtr_alloc(2, 2);
    matrix_t* Ainv = mtr_alloc(2, count-1);
    matrix_t* P    = mtr_alloc(2, 2);
    matrix_t* res  = mtr_alloc(2, 1);
    
    const double xt1 = positions[0].x;
    const double yt1 = positions[0].y;
    const double dt1 = distances[0];
    
    double dummy;
    
    for (uint8_t i=0; i<count-1; i++) {
	mtr_set(A, i, 0, xt1 - (positions[i+1].x));
	mtr_set(A, i, 1, yt1 - (positions[i+1].y));

	dummy = xt1*xt1 - (positions[i+1].x)*(positions[i+1].x)
	    + yt1*yt1 - (positions[i+1].y)*(positions[i+1].y)
	    - dt1*dt1 + distances[i+1]*distances[i+1];

	mtr_set(b, i, 0, .5*dummy);
    }
    
    err |= mtr_transpose(A, At);
    if (err != ERR_NOERROR) return err;
    err |= mtr_product(At, A, P);
    if (err != ERR_NOERROR) return err;
    err |= mtr_inverse2x2(P, Pinv);
    if (err != ERR_NOERROR) return err;
    err |= mtr_product(Pinv, At, Ainv);
    if (err != ERR_NOERROR) return err;
    err |= mtr_product(Ainv, b, res);

    respos->x = (int16_t)(.5 + mtr_get(res, 0, 0));
    respos->y = (int16_t)(.5 + mtr_get(res, 1, 0));

    mtr_free(A);
    mtr_free(b);
    mtr_free(At);
    mtr_free(Pinv);
    mtr_free(P);
    mtr_free(Ainv);
    mtr_free(res);

    return err;
}    

error_t pos_mlat3d(const position3d_t* const positions,
		   const cm_t* const distances,
		   const uint8_t count,
		   position3d_t* const respos)
{
    if (count < 4) {
	return ERR_ARRAY_DIM;
    }
	
    error_t err = ERR_NOERROR;

    matrix_t* A  = mtr_alloc(count-1,3);
    matrix_t* b  = mtr_alloc(count-1,1);
    matrix_t* At = mtr_alloc(3, count-1);
    matrix_t* Pinv = mtr_alloc(3, 3);
    matrix_t* Ainv = mtr_alloc(3, count-1);
    matrix_t* P    = mtr_alloc(3, 3);
    matrix_t* res  = mtr_alloc(3, 1);
    
    const double xt1 = positions[0].x;
    const double yt1 = positions[0].y;
    const double zt1 = positions[0].z;
    const double dt1 = distances[0];

    double dist, dummy;
    
    for (uint8_t i=0; i<count-1; i++) {
	mtr_set(A, i, 0, xt1 - (positions[i].x));
	mtr_set(A, i, 1, yt1 - (positions[i].y));
	mtr_set(A, i, 2, zt1 - (positions[i].z));
	
	dist = distances[i];

	dummy = xt1*xt1 - (positions[i].x)*(positions[i].x)
	    + yt1*yt1 - (positions[i].y)*(positions[i].y)
	    + zt1*zt1 - (positions[i].z)*(positions[i].z)
	    - dt1*dt1 + dist*dist;

	mtr_set(b, i, 0, .5*dummy);
    }
    
    err |= mtr_transpose(A, At);
    err |= mtr_product(At, A, P);
    
    err |= mtr_inverse3x3(P, Pinv);
    err |= mtr_product(Pinv, At, Ainv);
    err |= mtr_product(Ainv, b, res);

    if (err) {
	return err;
    }

    respos->x = (cm_t)(.5 + mtr_get(res, 0, 0));
    respos->y = (cm_t)(.5 + mtr_get(res, 1, 0));
    respos->z = (cm_t)(.5 + mtr_get(res, 2, 0));

    mtr_free(A);
    mtr_free(b);
    mtr_free(At);
    mtr_free(Pinv);
    mtr_free(P);
    mtr_free(Ainv);
    mtr_free(res);

    return err;
}    
