//Last modified: <16-Sep-2015 16:31:50 CEST by Dmitry Ebel>
#ifndef MATRIX_H
#define MATRIX_H
#include "errors.h"
#include "position.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    enum {
	MTR_ID   = (1<<1), /* identity matrix */
	MTR_DIAG = (1<<2), /* diagonal matrix */
	MTR_ZERO = (1<<3)  /* zero matrix */
    };

    typedef struct matrix_t {
	uint8_t type;
	uint8_t nrows;
        uint8_t ncols;
        double* data;
    } matrix_t;
    
    matrix_t* mtr_alloc(const uint8_t, const uint8_t);
    error_t   mtr_product(const matrix_t* const, const matrix_t* const, matrix_t* const);
    error_t   mtr_apply_to_pos2d(const matrix_t* const, const position2d_t* const, position2d_t* const);
    error_t   mtr_product_withT(const matrix_t* const, const matrix_t* const, matrix_t* const);
    error_t   mtr_transpose(const matrix_t*, matrix_t* const);
    error_t   mtr_add(const matrix_t* const, const matrix_t* const, matrix_t* const);
    error_t   mtr_inverse2x2(const matrix_t* const, matrix_t* const);
    matrix_t* mtr_inverse2x2_(const matrix_t* const);
    error_t   mtr_inverse3x3(const matrix_t* const, matrix_t* const);
    error_t   mtr_inverse4x4(const matrix_t* const, matrix_t* const);
    matrix_t* mtr_left_inverse(const matrix_t*);
    void      mtr_free(matrix_t*);

    /* set element */
    error_t   mtr_set(matrix_t*, const uint8_t, const uint8_t, const double);

    /* get element */
    double    mtr_get(const matrix_t*, const uint8_t, const uint8_t);

    /** Applies a callback to each element of the matrix 
     *  \arg m pointer to matrix
     *  \arg cb pointer to callback function
     *  \return error code
     */
    error_t   mtr_foreach(matrix_t* const m, double(*cb)(const double));

    /** Makes matrix to a rotation matrix for the given angle 
     *  \arg m pointer to matrix
     *  \arg phi angle in RAD
     *  \return error code
     */ 
    error_t   mtr_make_rot2d(matrix_t* const m , const double phi);
    error_t   mtr_append_rows(matrix_t* const m, const matrix_t* const r);

#ifndef __SAM3X8E__
    error_t   mtr_print(const matrix_t* m);
#endif

#ifdef __cplusplus
}
#endif
#endif

