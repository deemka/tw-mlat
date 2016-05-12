/* Last modified: <27-Nov-2015 12:03:57 CET by Dmitry Ebel> */
#ifndef PACK_H
#define PACK_H

#include "errors.h"
#include <stdint.h>
#include <stdlib.h>

#define SAMPLE_BITSIZE 12U

#ifdef __cplusplus
extern "C" {
#endif
    
    typedef struct {
	size_t  buffer_len_samples;
	size_t  buffer_len_bytes;
	size_t  sample_counter;
	uint8_t *data;
    } packed_data_t;
    
    error_t  pd_init(packed_data_t* const, const size_t nsamples);
    error_t  pd_put_sample(packed_data_t* const pd, const int16_t smp16);
    int16_t  pd_get_sample(packed_data_t const * const pd, const size_t pos);
    error_t  pd_pack_buffer(packed_data_t* const pd, int16_t const * const buf16, const size_t len);
#ifndef __SAM3X8E__
    error_t  pd_unpack_buffer(packed_data_t const * const pd, int16_t* const smp16);
#endif
    
#ifdef __cplusplus
}
#endif

#endif
