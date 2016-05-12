/* Last modified: <06-Nov-2015 14:17:59 CET by Dmitry Ebel> */

#include "parameters.h"
#include "messaging.h"
#include "VirtualWire.h"

unsigned long t11,t12;
#define DEBUG 0
#define REPORT(x) printf("# [%-20s]: %s\n", __func__, x);fflush(stdout);

static const long CLOCK_FREQ    = F_CPU;
#define CALC_PIN   9 				/* software interrupt to start TOF calculation = acq buffer full */

#define LED_RED    41				/* debug LED */
#define LED_BLUE   43				/* debug LED */
#define LED_GREEN  45

#define RF_PIN1    32				/* hardware interrupt to start acquisition */
#define RF_PIN2    34				/* hardware interrupt to start acquisition */
#define RF_PIN3    36				/* hardware interrupt to start acquisition */
#define RF_PIN4    38				/* hardware interrupt to start acquisition */
#define RF_PIN5    40				/* hardware interrupt to start acquisition */
#define RF_PIN6    42				/* hardware interrupt to start acquisition */

#define RF_TX_PIN    52

//#define ADC_CHANNELS ((1<<7) | (1<<6))
#define ADC_CHANNELS ((1<<7) | (1<<6) | (1<<5) | (1<<4) | (1<<3) | (1<<2) )

static volatile byte beacon_id;
static volatile sample_t adc_val7, adc_val6, adc_val5, adc_val4, adc_val3, adc_val2;			/* ADC is 12 bits, so a short is actually large enough */
static sample_t    rawsig7[BUFFER_LEN];		/* raw signal A0, S0 */
static sample_t    rawsig6[BUFFER_LEN];		/* raw signal A1, S1 */
static sample_t    rawsig5[BUFFER_LEN];		/* raw signal A2, S2 */
static sample_t    rawsig4[BUFFER_LEN];		/* raw signal A3, S3 */
static sample_t    rawsig3[BUFFER_LEN];		/* raw signal A4, S4 */
static sample_t    rawsig2[BUFFER_LEN];		/* raw signal A5, S5 */
static sample_t    *rawsig;
static int         writeloc = 0;			/* write position offset */
static volatile size_t    cnt = 0;		/* loop counter */
static volatile bool      acq_flag = false;	        /* busy with acquisition */
static volatile bool      calc_flag = false;	/* busy with DSP */

static volatile message_t msg;

inline void set_acq_flag1(void)   {acq_flag = 1; beacon_id = 0;}
inline void set_acq_flag2(void)   {acq_flag = 1; beacon_id = 1;}
inline void set_acq_flag3(void)   {acq_flag = 1; beacon_id = 2;}
inline void set_acq_flag4(void)   {acq_flag = 1; beacon_id = 3;}
inline void set_acq_flag5(void)   {acq_flag = 1; beacon_id = 4;}
inline void set_acq_flag6(void)   {acq_flag = 1; beacon_id = 5;}
inline void unset_acq_flag(void) {acq_flag = 0; }

inline void digitalWriteDirect(int pin, boolean val){
    if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
    else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

void set_sample_rate(const int srate){
    /*Timer created from an int value (in Hz) for the desired sample frequency to be used with ADC and DACs.
      This frequency should be 2x the maximum frequency that will be samples (Nyquist).
      Desired frequency clock is made available on TI0A Ch0

  
      ******** TABLE OF PARAMETERS FOR CLOCKS ***********

      TC      Chan  	NVIC "irq"  	IRQ handler   	PMC id

      TC0	0	TC0_IRQn	TC0_Handler	ID_TC0
      TC0	1	TC1_IRQn	TC1_Handler	ID_TC1
      TC0	2	TC2_IRQn	TC2_Handler	ID_TC2
      TC1	0	TC3_IRQn	TC3_Handler	ID_TC3
      TC1	1	TC4_IRQn	TC4_Handler	ID_TC4
      TC1	2	TC5_IRQn	TC5_Handler	ID_TC5
      TC2	0	TC6_IRQn	TC6_Handler	ID_TC6
      TC2	1	TC7_IRQn	TC7_Handler	ID_TC7
      TC2	2	TC8_IRQn	TC8_Handler	ID_TC8
    */
  
    pmc_enable_periph_clk (ID_TC0) ;         // start up TC0 Channel 0
    TcChannel * t = &(TC0->TC_CHANNEL)[0] ;  // create a pointer to TC0, Chan 0
   
    t->TC_CCR = TC_CCR_CLKDIS ;  // disable internal clocking while setup regs
    t->TC_IDR = 0xFFFFFFFF ;     // disable TC interrupts
    t->TC_SR ;                   // read int status reg to clear pending
  
    /*Setup clock speed
      TCCLKS selects the timer clock based on a scaler of the 84 MHz MCK
      TIMER_CLOCK1 = MCK/2
      TIMER_CLOCK2 = MCK/8
      TIMER_CLOCK3 = MCK/32
      TIMER_CLOCK4 = MCK/128
      TIMER_CLOCK5(1) = SLCK
    */
    t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |   // set timer clock at 42 MHz
	TC_CMR_WAVE |                  // waveform mode
	TC_CMR_WAVSEL_UP_RC;           // count-up PWM using RegisterC as threshold to reset
  
  
    /*Create a slower clock based on the desired sample rate*/
  
    int cycle = (CLOCK_FREQ/2)/REC_SAMPLE_RATE;  // 2 MHz/desired frequency, yields the counter used by registers
    t->TC_RC =  cycle ;              // RegisterC counts at the specificed cycle speed (ticks high each cycle)
    t->TC_RA =  cycle/2 ;            // RegisterA counts twice as fast as RC, 
    t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) |   
	TC_CMR_ACPC_SET |            // RC set compare on TIOA, creating a new clock from RC and RA cycle times
	TC_CMR_ACPA_CLEAR ;          // RA clear on TIOA.
  
    t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // re-enable local clocking and switch to hardware trigger source. 
}


void adc0_setup ()
{
    /* SETUP ADC to run with TI0A Ch0 as the trigger, with 12 bit accuracy
       Only A0 is activated by this setup, with end of conversion interrupt too
       All other analog inputs disabled, will need reactivating in setup.
    */
    adc_disable_all_channel(ADC); // disable all ADC channels (less noise)
    eAnalogReference(DEFAULT);    // set ADC reference voltage to the 3.3V default on the Due
    NVIC_EnableIRQ (ADC_IRQn) ;   // enable ADC interrupt vector
    NVIC_SetPriority(ADC_IRQn, 0);

    ADC->ADC_IDR  = 0xFFFFFFFF ;  // disable ADC interrupts
    ADC->ADC_IER  = ADC_CHANNELS;   // enable AD7 End-Of-Conv interrupt (Arduino pin A0), used to check for completed conversions
    ADC->ADC_CHER = ADC_CHANNELS;   // enable channel A0 (AD7 1<<7=0x80) and A1 (AD6 1<<6=0x40)

    /*CHANNEL ASSIGNMENTS ARE DIFFERENT TO WHAT IS MARKED ON THE DUE
      A0 = AD7, A1 = AD6, A2 = AD5, A3 = AD4, A4 = AD3, A5 = AD2,
      A6 = AD1, A7 = AD0, A8 = AD10, A9 = AD11, A10 = AD12, A11 = AD13.*/

    ADC->ADC_CGR = 0 ;            /* Set all gains to 1

GAINx | Gain DIFFx = 0 | Gain DIFFx = 1
0 0           1             0.5
0 1           1               1
1 0           2               2
1 1           4               2 */

    ADC->ADC_COR = 0x00000000;              // All offsets off
    ADC->ADC_MR = ADC_MR_ANACH |            // allow for different analog settings on each channel (if desired)
	ADC_MR_LOWRES_BITS_12 |   // enforce 12 bit capture
	ADC_MR_FREERUN_OFF |      // turn off freerun to force wait for trigger
	ADC_MR_TRGEN |            // enable external trigger mode for ADC
	ADC_MR_TRGSEL_ADC_TRIG1 ; /* ADC_TRIG1 = Use TIOA output from TC0 as the trigger

ADC_TRIG0   External : ADCTRG
ADC_TRIG1   TIOA Output of TC Channel 0
ADC_TRIG2   TIOA Output of TC Channel 1
ADC_TRIG3   TIOA Output of TC Channel 2
ADC_TRIG4   PWM Event Line 0
				  */
}

void ADC_Handler (void) {
    while (  (ADC->ADC_ISR & ADC_CHANNELS) != ADC_CHANNELS ) {;}    // initialize when ADC end-of-conversion is flagged
    adc_val7 = (sample_t)(*(ADC->ADC_CDR + 7)); /* AD0, S0 */
    adc_val6 = (sample_t)(*(ADC->ADC_CDR + 6)); /* AD1, S1 */
    adc_val5 = (sample_t)(*(ADC->ADC_CDR + 5)); /* AD2, S2 */
    adc_val4 = (sample_t)(*(ADC->ADC_CDR + 4)); /* AD3, S3 */
    adc_val3 = (sample_t)(*(ADC->ADC_CDR + 3)); /* AD4, S4 */
    adc_val2 = (sample_t)(*(ADC->ADC_CDR + 2)); /* AD5, S5 */
    if (acq_flag) {
	rawsig7[writeloc] = adc_val7;
	rawsig6[writeloc] = adc_val6;
	rawsig5[writeloc] = adc_val5;
	rawsig4[writeloc] = adc_val4;
	rawsig3[writeloc] = adc_val3;
	rawsig2[writeloc] = adc_val2;
	writeloc++;
	if (writeloc == BUFFER_LEN) {
	    writeloc = 0;      /* reset write pointer */
	    unset_acq_flag();  /* stop acquisition */
	    digitalWriteDirect(CALC_PIN, HIGH); // call ISR calc_tof() if it isn't still outstanding
	}
    }
}

inline void send_rf()
{
    digitalWriteDirect(LED_GREEN, HIGH);
    static byte idx = 0;
    idx = (idx+1)%6;

#if DEBUG
    SerialUSB.print("send ");
    SerialUSB.println(idx);
#endif

    idx=0;vw_send(&idx, 1);
    vw_wait_tx();

    switch (idx) {
    case 0:
	set_acq_flag1();
	break;
    case 1:
	set_acq_flag1();
	break;
    case 2:
	set_acq_flag1();
	break;
    case 3:
	set_acq_flag1();
	break;
    case 4:
	set_acq_flag1();
	break;
    case 5:
	set_acq_flag1();
	break;
    }
    digitalWriteDirect(LED_GREEN, LOW);
}


void calc_tof(void)
{
    digitalWriteDirect(LED_BLUE, HIGH);

#define CHUNK_SIZE 8 /*bytes*/

    msg.timestamp = micros();
    //Serial.print(msg.timestamp);
    
    msg.beacon_id = beacon_id;
    msg.header = MSG_START;

    byte* data;
    size_t i;
#if DEBUG
    SerialUSB.print("measuring signal from beacon ");
    SerialUSB.println(beacon_id);
#else
    msg.sensor_id = 0;
    SerialUSB.write((byte*)&msg, sizeof(msg));
    data = (byte*)(&rawsig7[0]);
    for (i=0; i<msg.data_size/CHUNK_SIZE; i++) {
	SerialUSB.write(data+CHUNK_SIZE*i, CHUNK_SIZE);
    }
    SerialUSB.write(data+CHUNK_SIZE*(i+1), msg.data_size-i*CHUNK_SIZE);
    
    /*    msg.sensor_id = 1;
    SerialUSB.write((byte*)&msg, sizeof(msg));
    data = (byte*)(&rawsig6[0]);
        for (i=0; i<msg.data_size/CHUNK_SIZE; i++) {
	SerialUSB.write(data+CHUNK_SIZE*i, CHUNK_SIZE);
    }
    SerialUSB.write(data+CHUNK_SIZE*(i+1), msg.data_size-i*CHUNK_SIZE);
        
    msg.sensor_id = 2;
    SerialUSB.write((byte*)&msg, sizeof(msg));
    data = (byte*)(&rawsig5[0]);
    for (i=0; i<msg.data_size/CHUNK_SIZE; i++) {
	SerialUSB.write(data+CHUNK_SIZE*i, CHUNK_SIZE);
    }
    SerialUSB.write(data+CHUNK_SIZE*(i+1), msg.data_size-i*CHUNK_SIZE);

    msg.sensor_id = 3;
    SerialUSB.write((byte*)&msg, sizeof(msg));
    data = (byte*)(&rawsig4[0]);
    for (i=0; i<msg.data_size/CHUNK_SIZE; i++) {
	SerialUSB.write(data+CHUNK_SIZE*i, CHUNK_SIZE);
    }
    SerialUSB.write(data+CHUNK_SIZE*(i+1), msg.data_size-i*CHUNK_SIZE);

    msg.sensor_id = 4;
    SerialUSB.write((byte*)&msg, sizeof(msg));
    data = (byte*)(&rawsig3[0]);
    for (i=0; i<msg.data_size/CHUNK_SIZE; i++) {
	SerialUSB.write(data+CHUNK_SIZE*i, CHUNK_SIZE);
    }
    SerialUSB.write(data+CHUNK_SIZE*(i+1), msg.data_size-i*CHUNK_SIZE);

    msg.sensor_id = 5;
    SerialUSB.write((byte*)&msg, sizeof(msg));
    data = (byte*)(&rawsig2[0]);
    for (i=0; i<msg.data_size/CHUNK_SIZE; i++) {
	SerialUSB.write(data+CHUNK_SIZE*i, CHUNK_SIZE);
    }
    SerialUSB.write(data+CHUNK_SIZE*(i+1), msg.data_size-i*CHUNK_SIZE);
    */
#endif
    digitalWriteDirect(LED_BLUE, LOW);
    digitalWriteDirect(CALC_PIN, LOW);
}

void setup() { 
 
    Serial.begin(115200);
    SerialUSB.begin(115200);
    while (!SerialUSB) ;
    set_sample_rate(REC_SAMPLE_RATE);
    adc0_setup();

    pinMode(CALC_PIN, OUTPUT);
    digitalWriteDirect(CALC_PIN, LOW);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED,  OUTPUT);

    pinMode(LED_BLUE,  OUTPUT);
    pinMode(RF_PIN1,   OUTPUT);
    pinMode(RF_PIN2,   OUTPUT);
    pinMode(RF_PIN3,   INPUT);
    pinMode(RF_PIN4,   INPUT);
    pinMode(RF_PIN5,   INPUT);
    pinMode(RF_PIN6,   INPUT);

    attachInterrupt(RF_PIN1, set_acq_flag1, RISING);
    attachInterrupt(RF_PIN2, set_acq_flag2, RISING);
    attachInterrupt(RF_PIN3, set_acq_flag3, RISING);
    attachInterrupt(RF_PIN4, set_acq_flag4, RISING);
    attachInterrupt(RF_PIN5, set_acq_flag5, RISING);
    attachInterrupt(RF_PIN6, set_acq_flag6, RISING);
    attachInterrupt(CALC_PIN, calc_tof,     RISING);

    unset_acq_flag();
    acq_flag = false;
    writeloc = 0;
    calc_flag = false;

    msg.header = MSG_START;
    msg.type   = MSG_TYPE_RAW;
    msg.data_size = BUFFER_LEN*sizeof(sample_t);

    vw_set_tx_pin(RF_TX_PIN);
    vw_setup(2000);
}

void loop()
{
    send_rf();
    delay(1000/2);
}
