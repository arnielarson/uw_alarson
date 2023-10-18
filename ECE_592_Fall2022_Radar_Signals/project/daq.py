#!/usr/local/bin/python3
import sys, os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from scipy import signal
from scipy.io import wavfile

import pyaudio
import time

## System wide params
DIR="data"



def get_data_path(filename):
  return os.path.join(DIR,filename)

def collect_and_save(T, filename, chunk=1024, channels=1, rate=8000):
  print("collect and save {} [s] to {}".format(T, filename))
  data = collect(T, chunk, channels, rate)
  wavfile.write(filename, rate, data)

def collect(T, chunk=1024, channels=1, rate=8000):
  print("Collecting for {}   [s]".format(time))

  p = pyaudio.PyAudio()
  s = p.open(format=pyaudio.paInt16, channels=channels, rate=rate, input=True, frames_per_buffer=chunk, input_device_index=2)
  N = rate*T # 2 bytes per data point
  print("* recording, collecting ~ {} bytes".format(N))

  data = np.zeros(N, dtype=np.int16)
  idx=0
  nFrames=0
  for i in range(0, int(rate / chunk * T)):
      buf = s.read(chunk) # 2048 byte chunks, 1024 ints
      d = np.frombuffer(buf, dtype=np.int16)
      #print("data stream: {} inputs, numpy data: {} inputs".format(len(buf), len(d)))
      end = int(idx + chunk)
      data[idx:end] = d
      idx+=int(chunk)
      nFrames+=1
  print("* done recording ({} frames)".format(nFrames))
  s.stop_stream()
  s.close()
  p.terminate()
  return data


def load(filename):
  filename = get_data_path(filename)
  if not os.path.exists(filename):
    raise Exception("File not found")
  print("Loading {}".format(filename))

  rate, data = wavfile.read(filename)
  return rate, data

def load_and_display(filename):
  fs, s = load(filename)
  print("Sample rate is {}".format(fs))
  # if fs = 8000, nps = 256 is sufficient..
  # if fs =48000, nps = 2048 is probably better
  if fs == 48000:
    plot_spectrogram(s, fs=fs, nps=2048)
  else:
    plot_spectrogram(s, fs=fs)


# this is from alarso_doppler - straightforward spectrogram plotter
def plot_spectrogram(s, fs=8000, nF=200, nps=256, title="Spectrogram", shorten=True):
  print("Preparing spectrogram for plotting, (rate={})".format(fs))
  f, t, S = signal.spectrogram(s, fs, nperseg=nps) #, nfft=256*2)
  # worry about this later..
  #print("[Debug] f shape: {}, t shape: {}, S shape: {}".format(f.shape, t.shape, S.shape))
  #dF = f[1]-f[0]
  #print("[Debug] dF approximately {}".format(dF))

  # f_max = fs/2,   Nf = nps/2,  df = f_max / Nf
  # Desired signal (for now) is 1000 Hz or less, 60 Hz / m/s
  print("f.shape: {}".format(f.shape))
  print("f[:10] {}".format(f[:10]))
  N = f.shape[0]
  df = fs/(2*N)
  idx= int(1500/df)
  if shorten:
    print("shortening y axis, N: {}, nps: {}, df: {}, idx: {}".format(N, nps, df, idx))
    plt.pcolormesh(t, f[:idx], S[:idx], shading="auto", norm=colors.LogNorm(vmin=S.min() + 1e-10, vmax=S.max()))
    plt.xlabel("Time [s]"), plt.ylabel("Frequency [Hz]")
    plt.title("Spectrogram of Juniper running")
    plt.savefig('juniper_running.png')
    plt.show()
  else:
    plt.pcolormesh(t, f, S, shading="auto", norm=colors.LogNorm(vmin=S.min() + 1e-10, vmax=S.max()))
    plt.xlabel("Time [s]"), plt.ylabel("Frequency [Hz]")
    plt.title(title)

    plt.show()


def spec(s, fs=8000):
  f, t, S = signal.spectrogram(s, fs, nperseg=1024*8) #, nfft=256*2)


if __name__ == "__main__":

  time = 2 if len(sys.argv)<2 else np.int(sys.argv[1])
  continuous(time, rate=8000)
