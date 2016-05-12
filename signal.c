/* Last modified: <12-Nov-2015 11:37:06 CET by Dmitry Ebel> */
#include "signal.h"
#include "math_float.h"

#include <math.h>
#include <limits.h>
#include <string.h>

#ifndef M_PI
#define M_PI    3.14159265358979323846
#endif
#ifndef M_TWOPI
#define M_TWOPI 6.28318530717958647692
#endif

#if FIXED_POINT_MATH
#include "math_fixed.h"
//#define sqrt(x) fx_sqrt((x))
#endif

const float relfac = 2;
static volatile size_t i;
static volatile size_t left,right;
static sample_t ip_tmp[BUFFER_LEN];
static sample_t qu_tmp[BUFFER_LEN];

#ifndef __SAM3X8E__
/* Generates a sin-wave of len_us milliseconds length @ freq_Hz Herz 
 * This is used for simulations only, so not completely floating point free */
sample_t* generate_burst(const unsigned freq_Hz, const unsigned srate, const unsigned len_us, const float phase)
{
    size_t nsamples = len_us*srate/1000000U;
    size_t nperiods = len_us*freq_Hz/1000000U;
    if (nsamples < 1)
	return NULL;

    sample_t* res  = (sample_t*)malloc(nsamples*sizeof(sample_t));
    unsigned  cnt;
    double omega = (M_TWOPI*nperiods)/(double)nsamples;
    for (cnt=0; cnt<nsamples; cnt++) {
	res[cnt] = (sample_t)(SAMPLE_MAX*sin(omega*cnt+phase));
    }

    return res;
}

/* This is used for simulations only, so not completely floating point free */
void sig_apply_gauss(sample_t* in, const size_t len)
{
    float sigma = (float)len/M_TWOPI;
    for (i=0; i<len; i++) {
	//FIXME: gauss output is OoM ~ 1e-4 WTF? */
	in[i] = (sample_t)( (float)(in[i]) * 1000 * gauss( ((float)i-(float)len*.5), sigma));
    }
}
#endif

void sig_normalize(sample_t* const in, const size_t len)
{
    sample_t magmax = 0;

    for (i=0; i<len; i++) {
	magmax = MAX (ABS(in[i]), magmax); 
    }

    sample_t mul = SAMPLE_MAX/magmax;

    for (i=0; i<len; i++) {
	in[i] *= mul;
    }
}

void sig_inphase_filter_simplified(const sample_t* restrict in, const size_t len,
				   sample_t* restrict out)
{
    size_t n;
    memset(out, 0, len*sizeof(sample_t));
    for (n=2; n<len-2; n++) {
	out[n] = (sample_t) ( (8*(sample_lt)in[n] - 4*((sample_lt)in[n-2] + (sample_lt)in[n+2]))) ;
	
    }
}

void sig_quadrature_filter_simplified(const sample_t* restrict in,
				      const size_t len,
				      sample_t* restrict out)
{
    size_t n;
    memset(out, 0, len*sizeof(sample_t));
    for (n=3; n<len-3; n++) {
	out[n] = (sample_t) ( (7*((sample_lt)in[n+1] - (sample_lt)in[n-1]) + (sample_lt)in[n-3] + (sample_lt)in[n+3]));
    }
}

void sig_inphase_signal(const sample_t* restrict in,
			const size_t len,
			sample_t* restrict out)
{
    sig_inphase_filter_simplified(in, len, out);
    size_t n;
    size_t m;
    for (n=0; n<len; n++) {
	m = n%4;
	switch(m) {
	case 0:
	    break;
	case 1:
	    break;
	case 2:
	    if (Q_SAMPLING_M % 2U) {
		out[n] *= 1;
	    } else {
		out[n] *=-1;
	    }
	    break;
	case 3:
	    if (Q_SAMPLING_M % 2U) {
		out[n] *= 1;
	    } else {
		out[n] *=-1;
	    }
	    break;
	}
    }
}

void sig_quadrature_signal(const sample_t* restrict in, const size_t len, sample_t* restrict out)
{
    sig_quadrature_filter_simplified(in, len, out);
    size_t n;
    size_t m;
    for (n=0; n<len; n++) {
	m = n%4;
	switch(m) {
	case 0:
	    break;
	case 1:
	    if (Q_SAMPLING_M % 2U) {
		out[n] *= 1;
	    } else {
		out[n] *=-1;
	    }
	    break;
	case 2:
	    if (Q_SAMPLING_M % 2U) {
		out[n] *= 1;
	    } else {
		out[n] *=-1;
	    }
	    break;
	case 3:
	    break;
	}
    }
}

void sig_inphase_component(const sample_t* restrict in, const size_t len, sample_t* restrict out)
{
    //memset(out, 0, len*sizeof(sample_t));
    
    size_t n, k, kmax;
    for (n=0; n<len; n++) {
	kmax = (PULSE_LEN_SMPL > n) ? n  : PULSE_LEN_SMPL;
	for (k=0; k<kmax; k++) {
	    out[n] = (sample_t) ( ((sample_lt)in[n-k] + (sample_lt)out[n]));
	}
    }
}

void sig_quadrature_component(const sample_t* restrict in, const size_t len, sample_t* restrict out)
{
    sig_inphase_component(in, len, out);
}

void sig_magnitude(const sample_t* const restrict in_ip,
		   const sample_t* const restrict in_qu,
		   const size_t len,
		   sample_t* const restrict out)
{
    size_t n;
    sample_lt prod1, prod2, sum;
    for (n=0; n<len; n++) {
	prod1 = (sample_lt)in_qu[n] * (sample_lt)in_qu[n];
	prod2 = (sample_lt)in_ip[n] * (sample_lt)in_ip[n];
	sum = prod1 + prod2;
	out[n] = (sample_t)sqrt((sample_ult)sum);
    }
}

error_t sig_decode(const sample_t* const restrict in,
		   const size_t len,
		   sample_t* const restrict out_inph,
		   sample_t* const restrict out_qu)
{
    memset(out_inph, 0, len*sizeof(sample_t));
    memset(out_qu, 0, len*sizeof(sample_t));

    sig_inphase_signal   (in, len, &ip_tmp[0]);
    sig_quadrature_signal(in, len, &qu_tmp[0]);

    sig_inphase_component   (&ip_tmp[0], len, out_inph);
    sig_quadrature_component(&qu_tmp[0], len, out_qu);
    
    return ERR_NOERROR;
}

void sig_scale(sample_t* const in, const size_t len, const sample_t valmax)
{
    sig_normalize(in, len);
    size_t i;
    sample_t div = SAMPLE_MAX/valmax;
    
    for (i=0; i<len; i++) {
	in[i] /= div;
    }
}

size_t sig_get_pos_of_max(const sample_t* in, const size_t len)
{
    sample_t m = 0;
    size_t res = 0;
    for (i=len-1; i>0; i--) {
	if (m < ABS(in[i])) {
	    m = ABS(in[i]);
	    res = i;
	}
    }
    return res;
}

error_t sig_get_3max_mag(const sample_t* in, const size_t len, size_t out_t[3], sample_t out_m[3])
{
    out_t[1] = sig_get_pos_of_max(in, len);  
    if (out_t[1] < 3) {
	return ERR_ARRAY_DIM;
    }
    
    out_t[0] = sig_get_pos_of_max(in, out_t[1]);
    out_t[2] = out_t[1] + 1 + sig_get_pos_of_max(in+out_t[1]+1, len-out_t[1]);

    out_m[0] = in[out_t[0]];
    out_m[1] = in[out_t[1]];
    out_m[2] = in[out_t[2]];

    return ERR_NOERROR;
}

error_t sig_get_parabola_suppts(const sample_t* in, const size_t len, size_t out_t[3], sample_t out_m[3])
{
    //out_t[1] = get_pos_of_max(in, len);
    out_t[0] = out_t[1];
    out_t[2] = out_t[1];
    
    sample_t threshold = in[out_t[1]]/5;

    if (out_t[1] < 3) {
	return ERR_ARRAY_DIM;
    }

    if (out_t[1] < PULSE_LEN_SMPL/2) {
	left = 1;
    } else {
	left = out_t[1] - PULSE_LEN_SMPL/2;
    }

    if (out_t[1] + PULSE_LEN_SMPL/2 > len) {
	right = len-1;
    } else {
	right = out_t[1] + PULSE_LEN_SMPL/2;
    }
    
    for (i=out_t[1]; i>left; i--) {
	if (in[i] >= threshold) {
	    out_t[0] = i;
	}
    }

    for (i=out_t[1]; i<right; i++) {
	if (in[i] > threshold) {
	    out_t[2] = i;
	}
    }

    out_m[0] = in[out_t[0]];
    out_m[1] = in[out_t[1]];
    out_m[2] = in[out_t[2]];

    return ERR_NOERROR;
}

double sig_speed_of_sound_mps(const double T /*deg C*/)
{
    return 20.05*sqrt(T+273.15);
}

void sig_apply_window(sample_t* const buf, const sample_t* win, const size_t buflen, const size_t winlen, const size_t center) {
    if (center<winlen/2) {
	left = 0;
    } else {
	left=center-winlen/2;
    }

    if (center+winlen/2>buflen) {
	right = buflen;
    } else {
	right = center + winlen/2;
    }

    size_t n, j=0;
    sample_lt dummy;

    for (n=left; n<right; n++) {
	dummy = (sample_lt)(buf[n]) * (sample_lt)(win[j]);
	j++;
	buf[n] = (sample_t)(dummy/32);
    }
}

void sig_derivative(const sample_t* sig, sample_t* const deriv, const size_t len)
{
    
    for (i=0; i<len-1; i++) {
	deriv[i] = sig[i+1] - sig[i];
    }
    deriv[len-1] = 0;
}

void sig_envelope(const sample_t* const restrict sig,
		  sample_t* const restrict envel,
		  const size_t len)
{
    sample_t curr = abs(sig[0]);
    size_t cnt = 0;
    for (i=1; i<len; i++) {
	if (abs(sig[i]) + relfac*cnt >= curr ) {
	    curr = abs(sig[i]);
	    cnt = 0;
	    envel[i] = curr;
	} else {
	    cnt++;
	    envel[i] = curr - relfac*cnt;
	}
    }
}

void sig_mabs(const sample_t* const restrict arr, sample_t* const restrict ab, const size_t len)
{
    for (i=0; i<len; i++) {
	ab[i] = (arr[i] >= 0) ? arr[i] : (sample_t)(-1)*arr[i];
    }
}

uint8_t sig_get_width(const sample_t* const restrict en,
		      const size_t maxpos,
		      const size_t len,
		      sample_t threshold,
		      size_t* const restrict newmaxpos)
{
    uint8_t res = 0;
    
    if (maxpos < PULSE_LEN_SMPL/2) {
	left = 0;
    } else {
	left = maxpos - PULSE_LEN_SMPL/2;
    }

    if (maxpos + PULSE_LEN_SMPL/2 > len) {
	right = len-1;
    } else {
	right = maxpos + PULSE_LEN_SMPL/2;
    }
    
    i=maxpos;
    while (i>left && en[i] >= threshold) {
	    res++;
	    i--;
    }
    left = i;

    i=maxpos;
    while(i<right && en[i] >= threshold) {
	res++;
	i++;
    }
    right = i;
    
    *newmaxpos = (left+right)/2;

    return res;
}

void sig_smooth(sample_t* in, sample_t* out, const size_t len)
{
    out[0] = in[0];
    for (i=1; i<len-1; i++) {
	out[i] = in[i-1] + in[i] + in[i+1];
    }

    out[len-1] = in[len-1];
}

float sig_zcr(const sample_t* frame, const size_t len)
{
    sample_t lastval = frame[0];
    float res = 0;
    for (i=1; i<len; i++) {
	if (lastval*frame[i] < 0) {
	    res += 1.;
	}
	lastval = frame[i];
    }
    return (res/(float)len);
}

void sig_apply_rectangular_window(sample_t* const sig, const size_t framelen, const size_t center, const size_t winlen)
{
    if (center < winlen/2) {
	left = 0;
    } else {
	left = center - winlen/2;
    }
    
    if (center > framelen - winlen/2 -1) {
	right = framelen -1 ;
    } else {
	right = center + winlen/2 +1;
    }
    memset(sig,     0, sizeof(sample_t)*left);
    memset(sig+right, 0, sizeof(sample_t)*(framelen-right));
}

void sig_remove_dc(sample_t* rawsig, size_t len)
{
/* Subtract DC offset */
    sample_t mean = fx_mean(&rawsig[1], len-1);
    for(size_t cnt=0; cnt<len-1; cnt++) { 
    	rawsig[cnt] -= mean; 
    }
    rawsig[len-1] = rawsig[0];
}

float sig_get_timeofflight(sample_t* const rawsig, size_t len, const uint8_t flags)
{
    error_t   res;			/* return value */
    float     dphi;			/* phase correction */
    float     tof;			/* time of flight */
    size_t en_max_pos;			/* position of max energy */
    size_t sig_max_pos;                    /* position of max signal */
    float  par_vertex;                     /* interpolated parabola max position */   
    size_t parab_width;                    /* half width of pulse [# of samples] */
    float interp_delta;                    /* distance between en_max_pos and interpolated max */
    sample_t    ip[len];		/* in phase component */
    sample_t    qu[len];		/* in quadratur component */
    sample_t    en[len];		/* magnitude */
    size_t      parx[3];			/* position of 3 peaks */
    sample_t    pary[3];			/* values of 3 peaks */

    switch (Q_SAMPLING_M) {
    case 3:
	parab_width = 7;
	break;
    case 5:
	parab_width = 6;
	break;
    case 7:
	parab_width = 3;
	break;
    default:
	parab_width = 3;
    }

    if (flags & TOF_REMOVE_DC) {
	sig_remove_dc(rawsig, len);
    }

    sig_max_pos = sig_get_pos_of_max(&rawsig[0], BUFFER_LEN-1) + 0;
    if ( ABS(rawsig[sig_max_pos]) < SIG_THRESHOLD ) { 
	return -1;
    } else {
	/* Apply rectangular window */
	/* (There is nothing interesting outside of the window) */
	if (flags & TOF_APPLY_RECT_WINDOW) {
	    sig_apply_rectangular_window(rawsig, len, sig_max_pos, (3*PULSE_LEN_SMPL/2));
	}
	/* Reconstruct signal envelope using the band sampling schema */
	res = sig_decode(&rawsig[0], len, ip, qu);
	sig_magnitude(ip, qu, len, en);
		
	if (flags & TOF_APPLY_RECT_WINDOW) {
	    sig_apply_rectangular_window(en, len, sig_max_pos, (3*PULSE_LEN_SMPL/2));
	}
	
	/* Parabola */
	parx[1] = sig_max_pos;
	res = sig_get_parabola_suppts(en, len, &parx[0], &pary[0]);
	if (res != ERR_NOERROR) {
	    return -1;
	} else {
	    en_max_pos = parx[1];
		    
	    interp_delta = ((float)(en[en_max_pos - parab_width]) - en[en_max_pos + parab_width])
		/( 2 * (en[en_max_pos - parab_width] -2*en[en_max_pos] + en[en_max_pos + parab_width]));
	    par_vertex = ((float)(en_max_pos) + interp_delta*parab_width);
		    
	    tof = (par_vertex)/(float)REC_SAMPLE_RATE;
	    if (flags & TOF_CORRECT_PHASE) {
		dphi = atan2f(ip[(size_t)ceilf(en_max_pos)], qu[(size_t)ceilf(en_max_pos)]);
		tof += dphi/(M_TWOPI*CARRIER_FREQ);
	    }
	}
    }
    return tof;
}

void sig_filter_init(sig_filter_t* const f)
{
    f->sig = (kiss_fft_cpx*)(KISS_FFT_MALLOC(BUFFER_LEN*sizeof(kiss_fft_cpx)));
    f->spectrum = (kiss_fft_cpx*)(KISS_FFT_MALLOC((BUFFER_LEN)*sizeof(kiss_fft_cpx)));
    f->isig = (kiss_fft_cpx*)(KISS_FFT_MALLOC((BUFFER_LEN)*sizeof(kiss_fft_cpx)));
    f->cfg  = kiss_fft_alloc(BUFFER_LEN, 0, 0, 0);
    f->icfg = kiss_fft_alloc(BUFFER_LEN, 1, 0, 0);
    
    f->sr_Hz = REC_SAMPLE_RATE;
    f->idx_bw_delta = 1400*BUFFER_LEN/f->sr_Hz;
    f->idx_center   = (int)((double)8140*.5*BUFFER_LEN/f->sr_Hz);
}


void sig_filter_apply(sig_filter_t* const f, sample_t* rawsig)
{
    for (int i=0; i<BUFFER_LEN; i++) {
	f->sig[i].r = 2*(kiss_fft_scalar)(rawsig[i])/BUFFER_LEN;
	f->sig[i].i = 0;
    }
    
    kiss_fft(f->cfg, f->sig, f->spectrum);
    f->spectrum[0].r = 0;
    f->spectrum[0].i = 0;
    f->spectrum[1].r = 0;
    f->spectrum[1].i = 0;
    
    for (int i = 0; i<1080; i++) {
	f->spectrum[i].r = 0;
	f->spectrum[i].i = 0;
    }
    
    for (int i = 440; i<1080; i++) {
	f->spectrum[i].r = 0;
	f->spectrum[i].i = 0;
    }

    for (int i = 1180; i<BUFFER_LEN; i++) {
	f->spectrum[i].r = 0;
	f->spectrum[i].i = 0;
    }
    
    kiss_fft(f->icfg, f->spectrum, f->isig);
    
    for (int i=0; i<BUFFER_LEN; i++) {
	//f->fsig[i] = (sample_t)((f->isig[i]).r);
	rawsig[i] = (sample_t)((f->isig[i]).r);
    }
    
}
