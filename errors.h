//Last modified: <08-Dec-2015 16:47:39 CET by Dmitry Ebel>
#ifndef ERRORS_H
#define ERRORS_H

//#include "types.h"
#include <stdint.h>

typedef int16_t  error_t;

#define ERR_NOERROR         0
#define ERR_MALLOC         (1<<1)
#define ERR_NODENOTFOUND   (1<<2)
#define ERR_NOTANUMBER     (1<<3)
#define ERR_READERROR      (1<<4)
#define ERR_WRITEERROR     (1<<5)
#define ERR_COMMUNICATION  (1<<6)
#define ERR_MTR_DIM        (1<<7)
#define ERR_ARRAY_DIM      (1<<8)
#define ERR_NOTENOUGHDATA  (1<<9)
#define ERR_BUFFEROVERRUN  (1<<10)
#define ERR_BUFFERUNDERRUN (1<<11)
#define ERR_CONFIG         (1<<12)

char const* err_lookup(const error_t code);
#endif
