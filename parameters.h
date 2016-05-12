#ifndef PARAMETERS_H
#define PARAMETERS_H

/* transmitter carrier frequency */
#define CARRIER_FREQ  40000U

/* sampling frequency (quadrature sampling)
 *           4 CARRIER_FREQ
 *  F_s = --------------------
 *         2*Q_SAMPLING_M - 1
 *
 *      !
 *  F_s >= 2W    =>   3 =< M =< 9
 */
#define Q_SAMPLING_M  3U

/* receiver (digitizer's) sampling rate 
   M = 3 -> 32000 Hz
   M = 5 -> 17777 Hz
   M = 7 -> 12308 Hz
*/
#define REC_SAMPLE_RATE   (4U*CARRIER_FREQ/(2U*Q_SAMPLING_M-1U))
#define PULSE_DUR_MSEC    (1)
const static unsigned PULSE_LEN_SMPL = ((PULSE_DUR_MSEC*REC_SAMPLE_RATE/1000));
#define DISTMAX 16U /* meters */
#define BUFFER_LEN  ( 8U*((DISTMAX*(REC_SAMPLE_RATE/340U))/8U))
#define SIG_THRESHOLD 16
#define FIXED_POINT_MATH 1

#define BCN_BEAM_ANGLE 85
#define SNS_BEAM_ANGLE 85



/* sampling precision */
#if FIXED_POINT_MATH
#define sample_t      int16_t
#define sample_lt     int32_t
#define sample_ut     uint16_t
#define sample_ult    uint32_t
#else
#define sample_t      float
#define sample_lt     double
#define sample_ut     sample_t
#define sample_ult    sample_lt
#endif

#endif
