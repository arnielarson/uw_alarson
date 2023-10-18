#!/usr/local/bin/python3
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from scipy import signal
import pyaudio

import time

## System wide params
class ContProc():

  def __init__(self, t, chunk=1024, channels=1, rate=8000):
    self.chunk=chunk
    self.channels=channels
    self.rate=rate
    self.data= np.zeros( rate * t * chunk, dtype=np.int16) # size of buffer to maintain
    self.idx=0  # starts here, increments by chunk
    self.rotate=False

  def process(indata, chunk, ti, status):
    buf = indata.read(chunk)
    frame = np.frombuffer(buf, dtype=np.int16)
    if self.rotate:
      pass
    else:
      self.data[idx:(idx+chunk)]=frame
      self.idx+=chunk




  def collect(t, chunk=1024, channels=1, rate=8000):
    print("Collecting for {}   [s]".format(time))
    # Setup pyaudio
    p = pyaudio.PyAudio()
    s = p.open(format=pyaudio.paInt16, channels=channels, rate=rate, input=True, frames_per_buffer=chunk)
    N = 2*rate*t # 2 bytes per data point
    print("* recording, collecting ~ {} bytes".format(N))

    data = np.zeros(N, dtype=np.int16)
    idx=0
    nFrames=0
    for i in range(0, int(rate / chunk * t)):
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
    plot_spectrogram(data, fs=rate, nF=800, nps=int(rate/4))

  # this is from alarso_doppler - straightforward spectrogram plotter
  def plot_spectrogram(s, fs=8000, nF=200, nps=1024, title="Spectrogram"):
    print("Preparing spectrogram for plotting, (rate={})".format(fs))
    f, t, S = signal.spectrogram(s, fs, nperseg=nps) #, nfft=256*2)
    print("[Debug] f shape: {}, t shape: {}, S shape: {}".format(f.shape, t.shape, S.shape))
    dF = f[1]-f[0]
    print("[Debug] dF approximately {}".format(dF))
    plt.pcolormesh(t, f[0:nF], S[0:nF], shading="auto", norm=colors.LogNorm(vmin=S.min() + 1e-10, vmax=S.max()))
    plt.xlabel("Time [s]"), plt.ylabel("Frequency [Hz]")
    plt.title(title)
    plt.show()

  def spec(s, fs=8000):
    f, t, S = signal.spectrogram(s, fs, nperseg=1024*8) #, nfft=256*2)


if __name__ == "__main__":

  channels=1
  rate=8000
  chunk=1024
  cp = ContProc()
  p = pyaudio.PyAudio()
  s = p.open(format=pyaudio.paInt16, channels=channels, rate=rate, input=True, frames_per_buffer=chunk, stream_callback=cp.process)


  N = 2*rate*t # 2 bytes per data point
  print("* recording, collecting ~ {} bytes".format(N))

  data = np.zeros(N, dtype=np.int16)
  input("Press any key to end")