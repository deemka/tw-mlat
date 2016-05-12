/* Last modified: <27-Nov-2015 12:04:14 CET by Dmitry Ebel> */
#include "pack.h"

#ifndef __SAM3X8E__
#include <assert.h>
#include <malloc.h>
#endif

error_t pd_init(packed_data_t * const pd, const size_t nsamples)
{
    pd->sample_counter = 0;
    pd->buffer_len_samples = nsamples;
    pd->buffer_len_bytes   = (size_t)( .5 + (float)nsamples*(float)SAMPLE_BITSIZE / 8.0);
    pd->data = (uint8_t*)calloc(pd->buffer_len_bytes, sizeof(uint8_t));
    if (pd->data == NULL) {
	pd->buffer_len_samples = 0;
	pd->buffer_len_bytes = 0;
	return ERR_MALLOC;
    }

    return ERR_NOERROR;
}

error_t pd_put_sample(packed_data_t* const pd, const int16_t smp16)
{
    if (pd->sample_counter == pd->buffer_len_samples) {
	return ERR_BUFFEROVERRUN;
    }

    if (smp16 >= (1<<(SAMPLE_BITSIZE+1))) {
	return ERR_NOTANUMBER;
    }
    
    uint8_t hb  = (smp16>>8) & 0xff;
    uint8_t lb  = smp16 & 0xff;
    size_t  pos = pd->sample_counter*SAMPLE_BITSIZE/8;

    if (!(pd->sample_counter%2)) { /* left-align to byte, split hb */
	pd->data[pos]   = lb;
	pd->data[pos+1] = hb<<4;
    } else {                    /* right-align to byte, split lb */
	pd->data[pos]  |= hb;
	pd->data[pos+1] = lb;
    }

    pd->sample_counter++;

    return ERR_NOERROR;
}


int16_t pd_get_sample(packed_data_t const * const pd, const size_t spos)
{
    assert(spos < pd->sample_counter);

    int16_t res = 0;
    uint8_t hb, lb = 0;

    size_t byte_pos = 3U*((spos)/2U);

    if ( !(spos%2) ) { /* left-aligned to byte, split hb */
	lb = pd->data[byte_pos];
	hb = (pd->data[byte_pos+1] >> 4) & 0x0f;
    } else {    /* right-aligned to byte, split lb */
	hb = pd->data[byte_pos+1] & 0x0f;
	lb = pd->data[byte_pos+2];
    }

    res = hb<<8 | lb;
    return res;
}

error_t pd_pack_buffer(packed_data_t* const pd, int16_t const * const buf16, const size_t len)
{
    size_t idx;
    error_t err = ERR_NOERROR;

    pd->sample_counter = 0;
    
    for (idx=0; idx<len; idx++) {
	err |= pd_put_sample(pd, buf16[idx]);
	if (err != ERR_NOERROR) {
	    return err;
	}
    }

    return err;
}

#ifndef __SAM3X8E__
error_t pd_unpack_buffer(packed_data_t const * const pd, int16_t* const buf16)
{
    //    if ( sizeof(int16_t)*malloc_usable_size((void*)*buf16) < pd->sample_counter) {
    //	*buf16 = (int16_t*)realloc((void*)*buf16, sizeof(int16_t)*pd->sample_counter);
    //}
    
    size_t idx;
    
    for (idx=0; idx<pd->sample_counter; idx++) {
	buf16[idx] = pd_get_sample(pd, idx);
    }

    return ERR_NOERROR;
}
#endif
