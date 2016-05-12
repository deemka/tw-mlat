/* Last modified: <2015-08-04 11:05:53 dmitry> */
#include "VirtualWire.h"

const int8_t id_pins[] = {5,6,7,8,9,10};
const int receive_pin = 11;
const int receive_en_pin = 3;

uint8_t buf[VW_MAX_MESSAGE_LEN];
uint8_t buflen = VW_MAX_MESSAGE_LEN;
volatile unsigned accum;
bool b1;
unsigned long t11, t12;

#define DEBUG 0

void setup()
{
#if DEBUG
    Serial.begin(115200);
#endif
    
    unsigned i;
    for (i=0; i<sizeof(id_pins); i++) {
	pinMode(id_pins[i], OUTPUT);
    }
    vw_set_rx_pin(receive_pin);
    vw_set_ptt_pin(receive_en_pin);
    vw_set_ptt_inverted(false);
    vw_setup(2000);

    vw_rx_start();
    digitalWrite(11,HIGH);
    //pinMode(4,INPUT);

    // Timer 2 (second 8-bit timer):
    // 1 tick @ 1024/16MHz = 64 us
    // 1 tick @ 8/16MHz    = 0.5 us
    TCCR2A = _BV (WGM20) | _BV (WGM21) | _BV (COM2B1);   // Fast PWM mode
    //    TCCR2B = _BV (WGM22) | _BV (CS20) | _BV (CS21) | _BV (CS22) ;  // prescaler of 1024 (datasheet p.162)

    // We wait 250 clock cycles = 250*0.5us = 125us
    TCCR2B = _BV (WGM22) | _BV(CS21);  // prescaler of 8 (datasheet p.162)
    OCR2A  = 250 - 1;
    TIMSK2 |= (1 << OCIE2A);
}

ISR (TIMER2_COMPA_vect)
{
    //vw_wait_rx();
    if (vw_get_message(buf, &buflen)) {
	accum=0; b1=true;
	Serial.print("# msg:");
	Serial.println(buf[0]);
    }
    
    buf[1] = digitalRead(receive_pin);
#if 0
    Serial.println(buf[1]);
#else    
    if (b1) {
	if (buf[1] == LOW) {
	    accum=0;
	}
	if (buf[1] == HIGH) {
	    accum++;
	}
	
	if (accum >=64 && b1) {   //128*125us = 16 ms
	    digitalWrite(id_pins[*buf], HIGH);
	    t11 = t12;
	    t12 = micros();
	    digitalWrite(id_pins[*buf], LOW);
#if DEBUG
	    Serial.println(t12-t11);
#endif
	    accum=0; b1=false;
	}
    }
#endif
}

void loop()
{
}
    
