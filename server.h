/* Last modified: <14-Sep-2015 15:01:03 CEST by Dmitry Ebel> */
#ifndef SERVER_H
#define SERVER_H

#include "beacon.h"
#include "sensor.h"
#include "messaging.h"
#include "stage.h"

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct {
	stage_t  stage;
	sample_t* rawsig;
	
	message_t msg;
	char serialdev[32];
	int fd;

	uint8_t stop;

    } server_t;
    
    error_t srv_init(server_t*);
    error_t srv_get_next_message(server_t*);
    error_t srv_open_serial_socket(server_t* srv);
    void    srv_open_read_socket(server_t*, const char*);
    void    srv_stop(server_t*);
#ifdef __cplusplus
    }
#endif

#endif
