/* A simple circular buffer 
 * Last modified: <2015-04-10 14:04:33 dmitry>
 */
#include "circbuf.h"
#include <stdlib.h>
#include <string.h>
#include <assert.h>

size_t cb_space_available(circbuf_t const* cbuf)
{
    return (cbuf->capacity - cbuf->size);
}

error_t cb_init(circbuf_t** const cbuf, const size_t capacity)
{
    size_t size_req = sizeof(circbuf_t) + capacity;
    
    *cbuf = (circbuf_t*)calloc(1, size_req);
    
    if (*cbuf == NULL) {
        return ERR_MALLOC;
    }
    (*cbuf)->in_cnt = 0;
    (*cbuf)->out_cnt = 0;
    (*cbuf)->capacity = capacity;
    (*cbuf)->size = 0;
    (*cbuf)->data = (char*)(*cbuf+1); /* data starts immediately after struct */
    return ERR_NOERROR;
}

error_t cb_free(circbuf_t** const cbuf)
{
    free(*cbuf);
    *cbuf = NULL;
    return ERR_NOERROR;
}

error_t cb_put_data(circbuf_t* const cbuf, const char* const src, const size_t inlen) {
    if (cb_space_available(cbuf) < inlen) {
        return ERR_BUFFEROVERRUN;
    }
    
    if (inlen == 0) {
        return ERR_NOERROR;
    }
    
    size_t tail = cbuf->capacity - cbuf->in_cnt;
    
    if ( tail < inlen ) { /* have to split input */
        size_t first = (cbuf->in_cnt-1)%(cbuf->capacity-1);
        memcpy(&(cbuf->data[first]), src, tail); 
        memcpy(&(cbuf->data[0]), src+tail, inlen-tail); 
        cbuf->in_cnt += inlen;
        cbuf->in_cnt %= cbuf->capacity;
    } else {
        memcpy(&(cbuf->data[cbuf->in_cnt]), src, inlen); 
        cbuf->in_cnt += inlen;
    }
    
    assert(cb_space_available(cbuf) >= inlen);
    cbuf->size += inlen;
    return ERR_NOERROR;
}

error_t cb_get_data(char* const dest, circbuf_t* const cbuf, const size_t outlen)
{
    if ( cbuf->size < outlen ) {
        return ERR_BUFFERUNDERRUN;
    }
    
    if (outlen == 0) {
        return ERR_NOERROR;
    }
    
    size_t tail = cbuf->capacity - cbuf->out_cnt;
    
    if ( tail < outlen) { // have to loop around 
        size_t len1 = tail;
        size_t len2 = outlen-len1;
        memcpy(dest, &(cbuf->data[cbuf->out_cnt]), len1);
        memcpy(dest+len1, &(cbuf->data[0]), len2);
    } else {
        memcpy(dest, &(cbuf->data[cbuf->out_cnt]), outlen);
    }
    
    cbuf->out_cnt += outlen;
    cbuf->out_cnt %= cbuf->capacity;
    
    assert(cbuf->size >= outlen);
    cbuf->size -= outlen;
    return ERR_NOERROR;
}

static char const* err_no_error         = "No error";
static char const* err_buffer_overrun   = "Buffer overrun";
static char const* err_buffer_onderrun  = "Buffer onderrun";
static char const* err_allocation_error = "Could not allocate memory";

char const* cb_lookup_error(short err)
{
    switch(err) {
        case ERR_NOERROR:
            return err_no_error;
        case ERR_BUFFEROVERRUN:
            return err_buffer_overrun;
        case ERR_BUFFERUNDERRUN:
            return err_buffer_onderrun;
        case ERR_MALLOC:
            return err_allocation_error;
    }
    return "Not specified";
}
