#!/usr/local/bin/python3

import sys, re, os, time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from scipy import signal
from scipy.io import wavfile

from daq import get_data_path




def _get_doppler_vel(fD, ftx=10.5e9):
  return fD*3e8/(2*ftx)

def _get_peak(S, f, threshold=0.01):
  idx=-1; peak=0
  for i in range(S.size):
      if S[i] > peak and S[i] > threshold:
          idx = i; peak=S[i]
  return (f[idx], peak) if idx != -1 else (idx, peak)

def _get_threshold(S, idf):
  t=[]
  t.append(np.float32(S[idf,:].sum()/ S[idf,:].size))
  t.append(np.float32(S[idf+2,:].sum()/ S[idf,:].size))
  t.append(np.float32(S[idf+7,:].sum()/ S[idf,:].size))
  t.append(np.float32(S[idf+11,:].sum()/ S[idf,:].size))
  t.append(np.float32(S[idf+19,:].sum()/ S[idf,:].size))
  return np.max(t)

def _get_threshold_old(S):
  size = S[:,1].shape[0]
  idx = int(size/2)
  t=S[idx,:].sum()/size
  return t


def process(filename):
  filename = get_data_path(filename)
  if not os.path.exists(filename):
    raise Exception("File not found")
  print("Loading {}".format(filename))
  fs, s = wavfile.read(filename)
  process_signal(s, fs)



# Returns velocity, doppler, target strength
# Used by radar.py
#
def get_doppler(s, fs=8000, nperseg=1024):
  #print("Getting doppler fs: {}, nperseg: {}".format(fs, nperseg))
  f, t, S = signal.spectrogram(s, fs, nperseg=nperseg) #, nfft=256*2)
  target=0
  mfreq=0
  mvel=0
  # go through each time step (column), only look at first 150Hz? which is about 55 mph

  df = fs/(2*f.shape[0])
  max_fidx= int(2000/df)
  # take threshold at max_index
  threshold=max(1, _get_threshold(S, max_fidx)*10)
  #print("Setting threshold: {}, Signal max: {}, Max frequency checked: {}".format(threshold, S[:max_fidx,:].max(), f[max_fidx]))
  #print("Max Signal: {}".format(S.max()))

  for i in range(S.shape[1]):
    s=S[:max_fidx,i]
    ti = 1+i
    idx = s.argmax()
    peak = s[idx]
    freq = f[idx]
    if peak > threshold:
        v=_get_doppler_vel(freq)
        if v>mvel:
          mvel=v
          mfreq=freq
          target=peak

  return target,mfreq,mvel*2.23694,threshold

def process_signal(s, fs=8000):
    f, t, S = signal.spectrogram(s, fs, nperseg=1024*8) #, nfft=256*2)
    threshold=_get_threshold_old(S)*10
    print("Average noise power: {:.3f}".format(threshold))
    print("Setting target threshold to: {:.2f}".format(threshold*10))
    for i in range(S.shape[1]):
        s=S[:,i]
        ti = 1+i
        idx = s.argmax()
        peak = s[idx]
        freq = f[idx]
        if peak > threshold:
            time.sleep(0.1)
            v=_get_doppler_vel(freq)
            print("t = {} s, v = {:.3f} [m/s], v = {:.3f} mph, target: {:.3f}, fD: {:.3f} [Hz]"
              .format(ti, v, v*2.23694, peak, freq))
        else:
          print("t = {}, no target".format(ti))
    return (f, t, S)


if __name__=='__main__':

  if len(sys.argv) < 2:
    print("Doppler.py usage:  ./doppler.py <wav_filename>")
  else:
    try:
      # could consider other parameters here..
      filename = sys.argv[1] if re.match(".*wav", sys.argv[1]) else sys.argv[1]+".wav"
      process(filename)
    except Exception as e:
      print("Error running Doppler.py: {}".format(e))