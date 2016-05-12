CC=gcc
SHARED_SOURCES=errors.c solve.c \
	beacon.c matrix.c \
	position.c \
	signal.c math_fixed.c \
	math_float.c sensor.c \
	kalman.c planimetrics.c \
	calibrator.c vehicle.c \
	movavg.c kissfft/kiss_fft.c \
	pack.c
SHARED_DEPS=errors.h beacon.h \
	solve.h matrix.h \
	messaging.h position.h \
	signal.h \
	math_fixed.h math_float.h \
	sensor.h kalman.h planimetrics.h \
	calibrator.h vehicle.h server.h \
	movavg.h kissfft/kiss_fft.h \
	pack.h

TRI_SOURCES=$(SHARED_SOURCES) main.c
SERVER_SOURCES=$(SHARED_SOURCES) server.c messaging.c serial.c linreg.c stage.c
CALIBRATION_SOURCES=$(SHARED_SOURCES) clbr_test.c messaging.c serial.c

TARGETS=tri server calibration

TRI_OBJECTS=$(TRI_SOURCES:.c=.o)
SERVER_OBJECTS=$(SERVER_SOURCES:.c=.o)
CALIBRATION_OBJECTS=$(CALIBRATION_SOURCES:.c=.o)
CFLAGS=-Wall -Wpedantic -std=gnu99

LDFLAGS=-lm -lconfig -fopenmp

all: $(TARGETS)

%.o: %.c %.h
	$(CC) $(CFLAGS) -c -o $@ $<

tri: $(TRI_OBJECTS)
	$(CC) $(LDFLAGS) $(TRI_OBJECTS) -o $@

server: $(SERVER_OBJECTS)
	$(CC) $(LDFLAGS) $(SERVER_OBJECTS) -o $@

calibration: $(CALIBRATION_OBJECTS)
	$(CC) $(LDFLAGS) $(CALIBRATION_OBJECTS) -o $@

clean:
	rm -rf *.o $(TARGETS)

debug: CFLAGS+= -DDEBUG -ggdb 
debug: $(TARGETS)

optimize: CFLAGS+= -O3 -ffast-math -funroll-loops
optimize: $(TARGETS)
