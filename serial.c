/* Last modified: <07-Oct-2015 15:31:15 CEST by Dmitry Ebel> */
#include "serial.h"
#include "messaging.h"
#include "errors.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#define NDEBUG 
#ifdef NDEBUG
#include <string.h>
#endif


int sio_open_serial(const char* dev)
{
    int fd;
  
    //    fd = open(dev, O_RDWR | O_NDELAY | O_NOCTTY);
    fd = open(dev, O_NDELAY | O_NOCTTY);
    if (fd == -1) {
#ifdef NDEBUG
	printf("open_port: Unable to open %s\n", dev);
#endif
	return -1;
    }
    else {
#ifdef NDEBUG
	printf("Opened %s\n", dev);
#endif
	fcntl(fd, F_SETFL, 0);
    }

    /* get current serial port settings */
    struct termios toptions;
    tcgetattr(fd, &toptions);
  
    toptions.c_cflag |= (CREAD | CLOCAL);
    //    toptions.c_cflag |= EXTA;
    //toptions.c_cflag |= EXTB;
    cfsetispeed(&toptions, B115200);
    cfsetospeed(&toptions, B115200);

    /* from arduino-serial.c    http://todbot.com/blog/  */
    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    //toptions.c_cc[VMIN]  = 0;
    //toptions.c_cc[VTIME] = 20;
    
    if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

#if 0
    cfsetispeed(&toptions, B115200);
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    //    toptions.c_lflag |= ICANON;


    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    /* no hardware flow control */
    toptions.c_cflag &= ~CRTSCTS;
    /* enable receiver, ignore status lines */
    toptions.c_cflag |= CREAD | CLOCAL;
    /* disable input/output flow control, disable restart chars */
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
    /* disable canonical input, disable echo,
       disable visually erase chars,
       disable terminal-generated signals */
    toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /* raw data, disable output processing */
    toptions.c_oflag &= 0;//~OPOST;

    
    /* commit the serial port settings */
    #endif

    tcsetattr(fd, TCSANOW, &toptions);
    return (fd);
}
