import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import time
from scipy import signal
from scipy.io import wavfile



# Author: Arnie Larson
# File: alarso_doppler
#
# About: Simple module to process IF radar data and look for doppler signals.
#        Generate simulations as wav files

# Globals
rng = np.random.default_rng()
c=3e8
mph=2.23694



# Process wavfile
def process_file(fname=None):
    fs, s1 = wavfile.read(fname)
    return process_signal(s1, fs)
    
    
def _get_peak(S, f, threshold=0.0):
    index=-1; peak=0
    for i in range(S.size):
        if S[i] > peak and S[i] > threshold:
            index = i; peak=S[i]
    return (f[index], peak) if index != -1 else (index, peak)

# Note hard coding
def _get_threshold(S):
    dimf = S[:,0].size
    if dimf != 4097:
        print("can't calculate threshold, only set for N = 8*1024 FFT")
        return 1.0
    threshold=(S[4000,:].sum()/ S[4000,:].size)
    return threshold
    
def _plot_spectrogram(f, t, S, title="Spectrogram"):
    plt.pcolormesh(t, f, S, shading="auto", norm=colors.LogNorm(vmin=S.min() + 1e-6, vmax=S.max()))
    plt.xlabel("Time [s]"), plt.ylabel("Frequency [Hz]")
    plt.title(title)
    plt.show()
    
# Process Signal, note fairly hard coded to sample rate 8000 and 1 second intervals
def process_signal(s1, fs=8000):
    
    f, t, S = signal.spectrogram(s1, fs, nperseg=1024*8) #, nfft=256*2)
    _plot_spectrogram(f, t, S)
    threshold=_get_threshold(S)
    print("Average noise power at 3900 Hz: {:.3f}".format(threshold))
    print("Setting target threshold to: {:.2f}".format(threshold*15))
    for i in range(S.shape[1]):
        s=S[:,i]
        ti = 1+i    
        freq, peak = _get_peak(s, f, threshold=15*threshold)
        if freq==-1:
            print("t = {}, no target".format(ti)) 
        else:
            time.sleep(0.4)
            v=_get_doppler_vel(freq)
            print("t = {} s, v = {:.2f} [m/s], v = {:.2f} mph, target: {:.2f}, fD: {:.2f} [Hz]"
              .format(ti, v, v*mph, peak, freq))
    return (f, t, S)
   
def _get_doppler_vel(fD, ftx=10.5e9):
    return fD*c/(2*ftx)
   
# Signal generation subroutines
def _get_amplitude(cs, r):
    return np.sqrt( cs / r**4)

def _get_doppler(v, ftx=10.5e9):
    return 2*v*ftx/c

def _generate_freq_signal(t, a, f, phase=0):
    S=a*np.cos(2*np.pi*f*t + phase)
    return S
    
# Generate signal with doppler shift, for time domain linspace t
#
# f(t) = dphi(t)/dt => phi(t) = f_0*t + 1/2 f_1*t^2 + phi_0
# f0 = 2*v0*ftx/c;  f1 = 2*a0*ftx/c 
def _generate_motion_signal(t, ftx=10.5e9, cs=1, x0=1, v0=28.6, a0=0.1, ircs=False):
    x = x0+v0*t+0.5*a0*t**2
    v = v0+a0*t
    w0 = 2*np.pi*2*v0*ftx/c
    w1 = 2*np.pi*2*a0*ftx/c 
    if ircs:  #infinite radar cross section, amplitude does not diminish
        S=_get_amplitude(cs, 1)*np.cos(w0*t + 0.5*w1*t**2)
    else:
        S=_get_amplitude(cs, x)*np.cos(w0*t + 0.5*w1*t**2)
    return S
    
def _scale_int16(S):
    return 32767*S/S.max()
    
# Add GWN to signal at dB relative to max, db = 20 * log10 ( Signal )
def _generate_noise(S, db=90):
    N = S.max() * 10**(-1*db/20)
    print("Generating {} dB noise, S_max: {:.3f}, sigma: {:.3f}".format(db, S.max(), N))
    noise = rng.normal(0,N,S.size)
    return (S+noise)
    
# Simulation signal generate routines
def generate( sim='chirp', fs=8000, seconds=15, fname=None):
    
    t=np.linspace(0,seconds,num=fs*seconds)
    sig=None
    Amax = np.iinfo(np.int16).max; 
    if sim == 'chirp':
        f_note=440
        chirp = .3*Amax * np.sin(2*np.pi*f_note*t + 2*np.pi*f_note*t**2)
        sig = chirp
    if sim == 'A4':
        f_note=440
        A4 = .5*Amax * np.sin(2*np.pi*f_note*t)
        sig = A4
    # default simulations used in class homeworks
    if sim == 'car_60db':
        Sc = _generate_motion_signal(t, cs=100, x0=1, v0=28.6, a0=0.5)
        sig =_scale_int16(_generate_noise(Sc, 60))
    if sim == 'bike_and_car':
        Sb = _generate_motion_signal(t, cs=2, x0=1, v0=4.5, a0=0.1)
        Sc = _generate_motion_signal(t, cs=100, x0=1, v0=28.6, a0=0.5)
        s=Sb+Sc
        sig =_scale_int16(_generate_noise(s, 60))
    if sim == 'car_ircs':
        Sc = _generate_motion_signal(t, cs=100, x0=1, v0=28.6, a0=0.5, ircs=True)
        sig = _scale_int16(Sc)
    if fname:
        wavfile.write(fname, fs, sig.astype(np.int16))
    return sig
    
# TODO - Add main routine for standalone program
