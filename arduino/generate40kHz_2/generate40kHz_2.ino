/* File: svn/tnm/branches/de/trilateration-nano/arduino/generate40kHz_2/generate40kHz_2.ino */
/* Last modified: <2015-08-04 14:58:00 dmitry> */

#define CLOCK_FREQ   (16000000L)
#define SIG_FREQ     (40000L)
#define CTRL_PIN1   2
#define CTRL_PIN2   3
#define SONAR_PIN1   9
#define SONAR_PIN2   10

void pulse1_start(void)
{
    TCCR1A |=  _BV (COM1A0);  // OC1A (pin 9) on Compare Match ON
    delayMicroseconds(1000);
    TCCR1A &= ~(_BV (COM1A0));  // OC1A (pin 9) on Compare Match OFF
    digitalWrite (SONAR_PIN1, LOW);
}
void pulse2_start(void)
{
    TCCR1A |=  _BV (COM1B0);  // OC1B (pin 10) on Compare Match ON
    delayMicroseconds(1000);
    TCCR1A &= ~(_BV (COM1B0));  // OC1B (pin 10) on Compare Match OFF
    digitalWrite (SONAR_PIN2, LOW);
}
    
void setup() {
    pinMode (SONAR_PIN1, OUTPUT);   // ultrasound Tx1, pin 9
    pinMode (SONAR_PIN2, OUTPUT);   // ultrasound Tx1, pin 10
    pinMode (CTRL_PIN1, INPUT);
    pinMode (CTRL_PIN2, INPUT);
    
    attachInterrupt(INT0, pulse1_start, RISING);   // external interrupt on pin 2 (coming from beaconcontroller)
    attachInterrupt(INT1, pulse2_start, RISING);   // external interrupt on pin 3 (coming from beaconcontroller)

    ////////////////////////////
    // Timer 1 (16bit) to 40 kHz
    /* control register (operation mode */
    TCCR1A = 0; 
    TCCR1B = _BV(WGM12) | _BV (CS10);   // CTC, No prescaler
    /* output compare register (always compared to current counter value) */
    OCR1A =  ((CLOCK_FREQ/(2L*SIG_FREQ))) - 1;
}

void loop()
{
}
