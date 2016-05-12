/* Last modified: <13-Nov-2015 10:32:09 CET by Dmitry Ebel> */
#include "VirtualWireUNO.h"

#define RF_DATA_PIN    9
const int8_t id_pins[] = {3,4,6,7,8,11};

#define DEBUG 0

uint8_t idx         = 0;
volatile uint8_t i  = 0;
volatile unsigned cnt = 0;

#define RF_CNT               164  
#define SYNC_PULSE_FIRST_CNT 120
#define SYNC_PULSE_LAST_CNT  121

void setup_tx()
{
    vw_set_tx_pin(RF_DATA_PIN);
    vw_setup(2000);
}

ISR (TIMER2_COMPA_vect)
{
    if (cnt == RF_CNT) { /* 320 ms ~ 100 m
			 164 ms ~ 55m, 6 beacons/secons
			 80  ms ~ 27 m 
			Must be > ~47 ms (16 meters)
		     */
	idx = (idx+1)%sizeof(id_pins);
#if DEBUG
	static volatile unsigned m,pm;
	pm = m; m=micros();
	Serial.println(m-pm);
#endif
	vw_send(&idx, 1);
	cnt = 0;
    }

    if (cnt >= SYNC_PULSE_FIRST_CNT && cnt <=SYNC_PULSE_LAST_CNT) {  // 2*.5ms = 1 ms sync pulse (RF)
	digitalWrite(RF_DATA_PIN, HIGH);
    } else {
	digitalWrite(RF_DATA_PIN, LOW);
    }
    
    if (cnt == SYNC_PULSE_LAST_CNT + 1) {
	digitalWrite(id_pins[idx], HIGH); // trigger US pulse on arduino nano 
	delayMicroseconds(50);
	digitalWrite(id_pins[idx], LOW);
    }
    
    cnt++;
}

void setup()
{
#if DEBUG
    Serial.begin(115200);
#endif
    // Timer 2 (second 8-bit timer):
    TCCR2A = _BV (WGM20) | _BV (WGM21) | _BV (COM2B1);   // Fast PWM mode
    TCCR2B = _BV (WGM22) | _BV (CS21) | _BV (CS20);  // prescaler of 64 (datasheet p.162)
    // p=256 => clock cycle = 256/16000000 = 16us
    // p=64  => clock cycle = 64/16000000  = 4us
    // p=32  => clock cycle = 32/16000000  = 2us
    // p=8   => clock cycle = 8/16000000   = .5us
    // We wait 250 clock cycles = 250*2us = .5ms
    OCR2A  = 250 - 1;
    TIMSK2 |= (1 << OCIE2A);

    setup_tx();

    sei();

    for (idx=0; idx<sizeof(id_pins); idx++) {
	pinMode(id_pins[idx], OUTPUT);
    }
}

void loop()
{
    
}
