/*Last modified: <25-Nov-2015 14:55:46 CET by Dmitry Ebel> */
#include "messaging.h"
#include "signal.h"
#include "errors.h"
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#undef DEBUG 

#ifdef DEBUG
#include <stdio.h>
#endif

int msg_read_next_message(message_t* const msg, const int fd, uint8_t* const data)
{
    ssize_t sz;
    uint8_t dummy = 0;
    uint16_t ui16dummy;
    uint32_t ui32dummy;
    unsigned offset;
    
    /* start flag */
    while (dummy != MSG_START) {
	do {
	    sz = read(fd, &dummy, sizeof(char));
	    if (sz <= -1)
		return ERR_COMMUNICATION;
	} while (sz != sizeof(char));
    }
#ifdef DEBUG
    printf("Found message start byte %02hhx \n", dummy);  fflush(stdout);
#endif

    sz = 0;
    /* message type */
    do {
	sz = read(fd, &dummy, sizeof(uint8_t));
	if (sz == -1) {
	    return ERR_COMMUNICATION;
	}
    } while (sz != sizeof(uint8_t)); 
#ifdef DEBUG
    printf("Read %d bytes type: %02hhx\n", (int)sz,  dummy);  fflush(stdout);
#endif

    if ( dummy != MSG_TYPE_RAW && dummy != MSG_TYPE_CMP) {
#ifdef DEBUG
    printf("Invalid message type %02hhx, message skipped\n", dummy);  fflush(stdout);
#endif
	return ERR_COMMUNICATION; /* skip */
    }
    msg->type = dummy;
    

    sz = 0;
    /* beacon_id */
    do {
	sz = read(fd, &dummy, sizeof(char));
	if (sz == -1)
	    return ERR_COMMUNICATION;
    } while (sz != sizeof(msg->type)); 
    
    msg->beacon_id = dummy;
#ifdef DEBUG
    printf("Read %d bytes bid: %02hhx\n", (int)sz,  msg->beacon_id);  fflush(stdout);
#endif

    sz = 0;
    /* sensor_id */
    do {
	sz = read(fd, &dummy, sizeof(msg->beacon_id));
	if (sz == -1)
	    return ERR_COMMUNICATION;
    } while (sz != sizeof(char)); 
    
    msg->sensor_id = dummy;
#ifdef DEBUG
    printf("Read %d bytes sid: %02hhx\n", (int)sz,  msg->sensor_id);  fflush(stdout);
#endif

    sz = 0;
    /* data size */
    do {
	sz = read(fd, &ui16dummy, 2);
	if (sz == -1)
	    return ERR_COMMUNICATION;
    } while (sz != 2); 
#ifdef DEBUG
    printf("Read %d bytes data_size: %d\n", (uint16_t)sz, ui16dummy);  fflush(stdout);
#endif
    
        if (ui16dummy >BUFFER_LEN*2) {
#ifdef DEBUG
	printf("Invalid data size %d, message skipped\n", ui16dummy);  fflush(stdout);
#endif
	return ERR_NOERROR; /* skip */
    }

    msg->data_size = ui16dummy;
    
    sz = 0;
    /* timestamp */
    do {
	sz = read(fd, &ui32dummy, 4);
	if (sz == -1) {
#ifdef DEBUG
	printf("Could not read timestamp\n");  fflush(stdout);
#endif
	return ERR_COMMUNICATION;
	}
    } while (sz != 4); 
    msg->timestamp = ui32dummy;
    
#ifdef DEBUG
    printf("Read %d bytes timestamp: %d\n", (uint16_t)sz, msg->timestamp);  fflush(stdout);
#endif
    
    sz = 0;
    offset = 0;
    if (msg->data_size >0) {
	/* read data */
	do {
	    sz = read(fd, data+offset, msg->data_size-offset);
	    if (sz == -1)
		return ERR_COMMUNICATION;
	    offset += sz;
	} while (offset <  msg->data_size);

#ifdef DEBUG
	printf("Read %d bytes data ", (int)offset);
	//for (unsigned i=0; i<offset; i++) {
	//    printf("%02hhx ", data[i]);
	//}
	//	printf("\n");
	fflush(stdout);
#endif
    
    } else {
#ifdef DEBUG
	    printf("No data\n");
#endif
   }

#ifdef DEBUG
    printf("\n ** Message complete\n");
#endif
    return ERR_NOERROR;
}
