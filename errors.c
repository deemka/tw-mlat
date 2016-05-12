//Last modified: <24-Nov-2015 16:01:17 CET by Dmitry Ebel>
#include "errors.h"

static char* errNoErr = "\"No Error.\"";
static char* errMalloc = "\"Malloc Error.\"";
static char* errNodeNotFound = "\"Node not found.\"";
static char* errNotANumber = "\"Not a number or no real-valued roots.\"";
static char* errRead = "\"Read Error.\"";
static char* errWrite = "\"Write Error.\"";
static char* errCommunication = "\"Communication Error.\"";
static char* errMtrDim = "\"Wrong Matrix Size.\"";
static char* errArrDim = "\"Wrong Array Size.\"";
static char* errNotEnoughData = "\"Not enough data.\"";
static char* errBufferOverrun = "\"Buffer Overrun.\"";
static char* errBufferUnderrun = "\"Buffer Underrun.\"";

char const * err_lookup(const error_t code)
{
    char const* res = "\"Undefined Error\"";
    switch (code) {
    case ERR_NOERROR:
	res = errNoErr;
	break;
    case ERR_MALLOC:
	res = errMalloc;
	break;
    case ERR_NODENOTFOUND:
	res = errNodeNotFound;
	break;
    case ERR_NOTANUMBER:
	res = errNotANumber;
	break;
    case ERR_READERROR:
	res = errRead;
	break;
    case ERR_WRITEERROR:
	res = errWrite;
	break;
    case ERR_COMMUNICATION:
	res = errCommunication;
	break;
    case ERR_MTR_DIM:
	res = errMtrDim;
	break;
    case ERR_ARRAY_DIM:
	res = errArrDim;
	break;
    case ERR_NOTENOUGHDATA:
	res = errNotEnoughData;
	break;
    case ERR_BUFFEROVERRUN:
	res = errBufferOverrun;
	break;
    case ERR_BUFFERUNDERRUN:
	res = errBufferUnderrun;
	break;
    }
    return res;
}

