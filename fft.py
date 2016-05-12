#!/usr/bin/python
import numpy as np
import scipy as sy
import scipy.fftpack as syfp
import pylab as pyl

# Read in data from file here
sr = 32000.0
sig = np.loadtxt("/data/tmp/rawsig_b2s0.txt")
length = len(sig)
# Create time data for x axis based on sig length
x = sy.linspace(1./sr, length/sr, num=length)

# Do FFT analysis of sig
FFT = syfp.fft(sig)
# Getting the related frequencies
freqs = syfp.fftfreq(sig.size, d=1/sr)
B=1400
FFT[ (np.abs(freqs)<50) ] = 1e-3
FFT[ (np.abs(freqs)>(sr/2-60)) ] = 1e-3

FFT[(np.abs(freqs)<8140-B)] = 1e-3
FFT[(np.abs(freqs)>8140+B)] = 1e-3

print freqs
# Create subplot windows and show plot
pyl.subplot(311)
pyl.plot(x, sig)

pyl.subplot(312)
#pyl.plot(freqs, sy.log10(FFT))
pyl.plot(np.abs(FFT))

fsig = syfp.ifft(FFT)
pyl.subplot(313)
pyl.plot(x, fsig)

pyl.show()

