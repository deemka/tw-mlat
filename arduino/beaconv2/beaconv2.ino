/* Last modified: <13-Nov-2015 10:32:07 CET by Dmitry Ebel> */
#include "VirtualWire.h"

#define RF_DATA_PIN  3
#define RF_ENABLE_PIN  4
volatile byte bid;
#define MY_ID 0

#define CLOCK_FREQ   (16000000UL)
#define SIG_FREQ     (40000UL)
#define SONAR_PIN   10

uint8_t buf[VW_MAX_MESSAGE_LEN];
uint8_t buflen = VW_MAX_MESSAGE_LEN;
volatile unsigned accum;
volatile bool b1, b2;
volatile unsigned long t11, t12;
volatile unsigned long cnt;

/* sync_samples = #counts(beaconcontroller) * prescaler(beaconcontroller) / prescaler(beaconv2) */
unsigned sync_samples = 8;   //8*125us = 1 ms

#define DEBUG 0
#define WIRQ  0

#if DEBUG
volatile unsigned acmax;
#endif


void pulse_start(void)
{
#if WIRQ
    detachInterrupt(digitalPinToInterrupt(RF_DATA_PIN));
#endif
    digitalWrite(5, HIGH);
    TCCR1A |= _BV (COM1B0);   // toggle  OC1B on CM (datasheet 13.11.1)
    delayMicroseconds(1000);
    TCCR1A &= ~( _BV (COM1B0)); // disconnect OC1B

    digitalWrite (SONAR_PIN, LOW);
    digitalWrite (5, LOW);
}

void setup_timer2() {

    TCCR2A = _BV (WGM20) | _BV (WGM21) | _BV (COM2B1);   // Fast PWM mode
    TCCR2B = _BV (WGM22) | _BV(CS21);  // prescaler of 8 (datasheet p.162)
    OCR2A  = 250 - 1;        // 'time resolution' reading sync pulse (250*8/16 MHz = 125us)
    TIMSK2 |= (1 << OCIE2A); // timer interrupt to read vw messages and to count the length of the sync pulse
    
    // TMR1 is set in VirtualWire.cpp
    // with TCCR1A = 0; TCCR1B = _BV(WGM12); TCCR1B |= prescaler /* = 1 */; OCR1A = nticks;
    // the same as needed for ultrasound generation,
    // i.e. we are able to use T1B for utrasound
    // setting OC1B on Compare Match with the appropriate OCR1B
    OCR1B =  ((CLOCK_FREQ/(2L*SIG_FREQ))) - 1; // = 199
}

void setup()
{
#if DEBUG
    Serial.begin(115200);
#endif

    /* VW uses timer1 */
    vw_set_rx_pin(RF_DATA_PIN);
    vw_set_ptt_pin(RF_ENABLE_PIN);
    vw_set_ptt_inverted(false);
    vw_setup(2000);

    vw_rx_start();
    
    pinMode (SONAR_PIN, OUTPUT);   // ultrasound Tx1, pin 9
    pinMode (5, OUTPUT);           // control LED
    pinMode (6, OUTPUT);           // control LED
    setup_timer2();
}


ISR (TIMER2_COMPA_vect)
{
    //vw_wait_rx();
    if (vw_get_message(buf, &buflen)) {
	accum=0; b1=false;
#if WIRQ
    detachInterrupt(digitalPinToInterrupt(RF_DATA_PIN));
    digitalWrite(6, LOW);
#endif

	if (buf[0] == MY_ID) {
	    cnt =0;
	    digitalWrite(6, HIGH);
	    b1 = true;
	} else {
	    cnt = 10000;
	    b1 = false;
	}
    }

#if WIRQ
    if (b1 && cnt == sync_samples/2) {
	attachInterrupt(digitalPinToInterrupt(RF_DATA_PIN), pulse_start, FALLING);
    }
#else
    if (b1) {
	buf[1] = digitalRead(RF_DATA_PIN);
	if (buf[1] == LOW) {
	    accum=0;
	    digitalWrite(6, LOW);
	}
	if (buf[1] == HIGH) {
	    digitalWrite(6, HIGH);
	    accum++;
	}
	if (accum == sync_samples && b1) { 
	    b1=false;
	    digitalWrite(6, LOW);
	    pulse_start();
	}
    }
#endif

    cnt++;
}
    
void loop() {}
