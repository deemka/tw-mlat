/* A simple circular buffer 
 * Last modified: <23-Sep-2015 13:07:49 CEST by Dmitry Ebel>
 */

#ifndef CIRCBUF_H
#define CIRCBUF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "errors.h"
#include <stddef.h>
    
    typedef struct circbuf_t {
	unsigned int in_cnt;
	unsigned int out_cnt;
	size_t       capacity;
	size_t       size;
	char         *data;
    } circbuf_t;

    error_t cb_init(circbuf_t** const, const size_t);
    error_t cb_free(circbuf_t**);
    error_t cb_put_data(circbuf_t* const, const char* const, const size_t);
    error_t cb_get_data(char* const, circbuf_t* const, const size_t);
    size_t  cb_space_available(const circbuf_t* const);
    char const* cb_lookup_error(short);
    
#ifdef __cplusplus
}
#endif

#endif
