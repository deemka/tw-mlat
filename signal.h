/* Last modified: <11-Nov-2015 16:16:56 CET by Dmitry Ebel> */
/* @file signal.h 
 * @brief signal processing functions 
 */
#ifndef SIGNAL_H
#define SIGNAL_H

#include "errors.h"
#include "kissfft/kiss_fft.h"

#include <stdint.h>
#include <stdlib.h>

#include "parameters.h"

#define METER2CM 100

#define VEH_MAX_SPEED_CMPS 100

#ifndef BITS_PER_UNIT
#define BITS_PER_UNIT 8
#endif

#define SPEED_OF_SOUND (331.3 * sqrt(1.0 + 21.5/ 273.15))

#define TOF_CORRECT_PHASE     (uint8_t)(1<<0)
#define TOF_APPLY_RECT_WINDOW (uint8_t)(1<<1)
#define TOF_REMOVE_DC         (uint8_t)(1<<2)

/* max value of the integer type with the given digital resolution */
#if FIXED_POINT_MATH
#define SAMPLE_MAX ( (((sample_t)(-1)) & (sample_t)(~(1<<(BITS_PER_UNIT*sizeof(sample_t)-1))))/4 )
#else
#define SAMPLE_MAX (1.0)
#endif

#define ABS(a)	       \
    __extension__ \
    ({ __typeof__ (a) _a = (a);			\
	((_a) > 0) ? (_a) : (-(_a)); })

#define MAX(a,b)	       \
    __extension__ \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#define MIN(a,b)	       \
    __extension__ \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct {
	double        sr_Hz;
	double        bw_Hz;
	size_t        idx_center;
	size_t        idx_bw_delta;
	kiss_fft_cfg  cfg;
	kiss_fft_cfg  icfg;
	kiss_fft_cpx* sig;
	kiss_fft_cpx* spectrum;
	kiss_fft_cpx* isig;
	sample_t      fsig[BUFFER_LEN];
    } sig_filter_t;

    void sig_filter_init(sig_filter_t* const f);
    void sig_filter_apply(sig_filter_t* const f, sample_t* rawsig);

    /*! Returns the index of the array element of the maximal magnitude 
      \param  in  nput array (pointer)
      \param  len Number of elements in the array
      \return     Index of element with max magnitude
    */ 
    size_t sig_get_pos_of_max (const sample_t* in, const size_t len);
    
#ifndef __SAM3X8E__
    /*! Allocates and calculates a sine wave and returns
      pointer to it (in order to simulate a puls on a PC)
      \param  freq_Hz Signal frequency [Hz] 
      \param  srate   Digitizer's sample rate (should be >2*freq_Hz) 
      \param  len_us  Pulse duration in microseconds 
      \return         Pointer to the first element
    */
    sample_t* generate_burst (const unsigned freq_Hz, const unsigned srate, const unsigned len_us, const float phase);

    void  sig_apply_gauss(sample_t* in, const size_t len);
#endif
    
    /*! Normalizes amplitude of a real-valued signal to 1.0 
     *                       of a int-valued signal to MAX_INT/2
     */
    void sig_normalize(sample_t* const in, const size_t len);

    /*! Applies in-phase filter using 7 precalculated coefficients
      [ 0, 4, 0, 8, 0, 4, 0 ] rather than the exact filter form
      \param  in  Input signal (quadrature sampled)
      \param  len Number of samples 
      \param  out In-phase filtered signal (must be preallocated outside)
      \return     Nothing
      \sa inphase_signal(), inphase_filter_exact()
    */
    void sig_inphase_filter_simplified   (const sample_t* in, const size_t len, sample_t* out);

    /*! Applies quadrature filter using 7 precalculated coefficients
      [ 1, 0, 7, 0, 7, 0, 1 ] rather than the exact filter form
      \param  in  Input signal (quadrature sampled)
      \param  len Number of samples 
      \param  out Quadrature-filtered signal (must be preallocated outside)
      \return     Nothing
      \sa quadrature_signal(), quadrature_filter_exact()
    */
    void sig_quadrature_filter_simplified(const sample_t* in, const size_t len, sample_t* out);

    void sig_inphase_filter_exact        (const sample_t* in, const size_t len, sample_t* out);
    void sig_quadrature_filter_exact     (const sample_t* in, const size_t len, sample_t* out);

    
    /*! Calculates in-phase signal from in-phase-filtered signal
     */
    void sig_inphase_signal(const sample_t* const restrict in,
			    const size_t len,
			    sample_t* const restrict out);

    /*! Calculates quadrature signal from quadrature-filtered signal
     */
    void sig_quadrature_signal(const sample_t* const restrict in,
			       const size_t len,
			       sample_t* const restrict out);


    void sig_inphase_component           (const sample_t* in, const size_t len, sample_t* out);
    void sig_quadrature_component        (const sample_t* in, const size_t len, sample_t* out);

    /*! Calculates signal magnitude as sqrt(inphase component + quadrature component)
     */ 
    void sig_magnitude(const sample_t* const restrict in_ip,
		       const sample_t* const restrict in_qu,
		       const size_t len,
		       sample_t* const restrict out);

    /*! Reconstructs magnitude of a quadrature-sampled signal
     */
    error_t sig_decode(const sample_t* const restrict in,
		       const size_t len,
		       sample_t* const restrict out_inph,
		       sample_t* const restrict out_qu);

    /*! Scales amplitude of a real-valued signal
      \param  in     Pointer to the first element of the array
      \param  len    Number of elements in the array
      \param  valmax Desired max signal amplitude
      \return        Nothing 
    */
    void sig_scale(sample_t* const in, const size_t len, const sample_t valmax);

    /*! Finds 3 max peaks in the array in order to do parabola interpolation
     */
    error_t sig_get_3max_mag(const sample_t* in, const size_t len, size_t out_t[3], sample_t out_m[3]);

    /*! Returns speed of sound [m/s] for a temperature in deg Celsius
     */
    double sig_speed_of_sound_mps(const double T /*deg C*/);


    size_t sig_get_tof_us(const sample_t* in, const size_t len);

    sample_t* sig_generate_pwm (const unsigned freq_Hz,
				const unsigned srate, const
				unsigned len_us);

    void sig_apply_window(sample_t* const, const sample_t*, const size_t, const size_t, const size_t);
    void sig_derivative(const sample_t* in, sample_t* const out, const size_t len);

    void sig_envelope(const sample_t* const restrict in,
		      sample_t* const restrict out,
		      const size_t len);

    void sig_mabs(const sample_t* const restrict arr,
		  sample_t* const restrict ab,
		  const size_t len);

    uint8_t sig_get_width(const sample_t* const restrict en,
			  const size_t maxpos,
			  const size_t len,
			  const sample_t threshold,
			  size_t* const restrict newmaxpos);

    void    sig_smooth(sample_t* in, sample_t* out, const size_t len);

    error_t sig_get_parabola_suppts(const sample_t* in, const size_t len,
				    size_t out_t[3], sample_t out_m[3]);

    float   sig_zcr(const sample_t*, const size_t);

    void    sig_apply_rectangular_window(sample_t* const frame,
					 const size_t framelen,
					 const size_t center,
					 const size_t winlen);

    float   sig_get_timeofflight(sample_t* const rawsig, size_t len, const uint8_t flags);
    
#ifdef __cplusplus
}
#endif
#endif
    
    

