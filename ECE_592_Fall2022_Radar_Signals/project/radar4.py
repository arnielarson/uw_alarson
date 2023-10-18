#!/usr/local/bin/python3
#
#       Author:   Arnie Larson
#       Date:     12/10/2020
#
# Note  I discovered that the better way to utilize multiple cores is with
#       multiprocessing Process and Queue.  There is a threadsafe Queue
#       implemented that is used to share data via IPC.
#       Realtime program uses a multithreading.Process for collecting data
#       from an audio device.
#       The Application has two callbacks (launched in threads) to process
#       recent data for doppler and to update a spectrogram
#
#import queue
import threading
from multiprocessing import Process, Queue
import numpy as np
import doppler as dop
import time
import pyaudio
import tkinter as tk
from tkinter import ttk
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from scipy import signal

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

matplotlib.use("TkAgg")





# device_index = 0 => system microphone
# device_index = 2 => USB microphone (radar IF)
#
def listen(queue, channels=1, rate=8000, chunk=1024, device_index=0):

  print("Listening to device_index {} at fs = {}".format(device_index, rate))
  p = pyaudio.PyAudio()
  s = p.open(format=pyaudio.paInt16, channels=channels, rate=rate, input=True, frames_per_buffer=chunk, input_device_index=device_index)
  valid = p.is_format_supported(input_format=pyaudio.paInt16, input_channels=1, rate=rate, input_device=device_index)
  print("Listening to device_index {} at fs = {}, supported: {}".format(device_index, rate, valid))

  while True:
    try:
      buf = s.read(chunk) # 2048 byte chunks, 1024 ints
      d = np.frombuffer(buf, dtype=np.int16)
      queue.put(d)
      #print("[Listen] read data, putting {} into queue".format(type(d)))
    except Exception as e:
      print("Exception in Listen: {}".format(e))
  print("* exiting listener")
  if not s.is_stopped():
    s.stop_stream()
    s.close()
  p.terminate()






class App(tk.Frame):

  def __init__(self, root, queue, rate, T=5):
    tk.Frame.__init__(self, root)
    self.queue = queue
    self.rate = rate

    self.nperseg = 2048
    self.vidx = 4*self.nperseg
    if rate == 8000:
      N=T*8*1024
    elif rate == 48000:
      N=T*48*1024
    else:
      N=100*1024
    self.rotate=False; update=True
    self.data = np.zeros(N, dtype=np.int16)
    self.datalock = threading.Lock()
    self.idx=0


    #self.container = tk.Frame()
    #self.container.pack(side="top", fill="both", expand=True)
    self.canvas = tk.Canvas(self, bg='white', height=384, width=576)
    self.canvas.pack()
    self.figure = plt.gcf()
    self.figure.set_size_inches(6,4)
    self.figure_canvas_agg = FigureCanvasTkAgg(self.figure, self.canvas)


    self.frame=ttk.Frame(self, padding=100, height=384, width=576)
    self.frame.grid()

    self.speed_var = tk.StringVar(self.frame, "NaN")
    self.target_var = tk.StringVar(self.frame, "Nan")
    self.doppler_var = tk.StringVar(self.frame, "Nan")

    ttk.Label(self.frame, text="Speed  ", font=("Helvetica", 14)).grid(column=0, row=0)
    ttk.Label(self.frame, textvariable=self.speed_var).grid(column=1, row=0)
    ttk.Label(self.frame, text="Frequency").grid(column=0, row=1)
    ttk.Label(self.frame, textvariable=self.doppler_var).grid(column=1, row=1)
    ttk.Label(self.frame, text="Target Strength").grid(column=0, row=2)
    ttk.Label(self.frame, textvariable=self.target_var).grid(column=1, row=2)

    ttk.Button(self.frame, text="Exit", command=self.shutdown).grid(column=1, row=4)
    self.frame.pack()
    self.after(50, self.update_data)
    self.after(100, self.draw_figure)
    # creating a frame and assigning it to container
    # container = tk.Frame(self, height=400, width=600)
    # specifying the region where the frame is packed in root
    # container.pack(side="top", fill="both", expand=True)
    self.pack()

  def shutdown(self):
    self.after_cancel(self)
    time.sleep(.1)
    self.quit()

  # Create updated spectrogram from data
  # uses datalock to access last T seconds of data
  #
  def draw_figure(self):
    print("Drawing figure")
    self.datalock.acquire()
    # If data is all zeros that's a problem
    if self.data.max() == self.data.min():

      self.datalock.release()
      self.after(200, self.draw_figure)
      return
    else:
      print("Processing spectrogram")
      f, t, S = signal.spectrogram(self.data, self.rate, nperseg=self.nperseg) #, nfft=256*2)
    self.datalock.release()



    #canvas.delete('all')
    df = 48000/(2*f.shape[0])
    max_fidx= int(1000/df)

    print("[Spectrogram] S ave: {:.3f}, max: {:.3f} min: {:.3f}".format(np.average(S), S.max(), S.min()))
    #plt.gca.clear() # clear ases?
    plt.clf()
    plt.pcolormesh(t, f[:max_fidx], S[:max_fidx], shading="auto", norm=colors.LogNorm(vmin=S.min() + 1e-6, vmax=S.max()))
    plt.colorbar()
    plt.xlabel("Time [s]"), plt.ylabel("Frequency [Hz]")
    plt.title("Spectrogram")


    self.figure_canvas_agg.draw()
    self.figure_canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=1)
    self.figure_canvas_agg.draw()

    self.canvas.update()
    self.after(200, self.draw_figure)


  # Process recent (100ms?) of data looking for target
  # Uses datalock to access data variable
  #
  def update_data(self):
    #print("Processing data at rate: {}".format(self.rate))
    i=0
    while not self.queue.empty():
      d = self.queue.get()
      #d = np.frombuffer(buf, dtype=np.int16)
      #print("[Process] got data {}".format(d.shape))
      size = d.shape[0]
      try:
        if not self.rotate:
          #idx_end=min(idx+buf.shape[0],data.shape[0])
          eidx=self.idx+size
          self.datalock.acquire()
          self.data[self.idx:eidx] = d
          self.idx=eidx
          if self.idx==self.data.shape[0]:
            self.rotate=True
          self.datalock.release()
        else:
          self.datalock.acquire()
          end_idx=self.data.shape[0]-size
          self.data[0:end_idx]=self.data[size:]
          self.data[end_idx:]=d
          self.datalock.release()

      except Exception as e:
        print("Exception e: {}".format(e))
      i+=1
    print("[Processor] processed {} data chunks".format(i))
    self.datalock.acquire()
    if self.data.max() == self.data.min():
      self.datalock.release()
      self.after(100, self.update_data)
      return
    else:
      mtarget,mfreq,mvel,thresh = dop.get_doppler(self.data[self.vidx:], self.rate, self.nperseg)
    self.datalock.release()
    print("Doppler vel: {:.3f} [mph], freq: {:.3f} [Hz], target_strength: {:.3f}, threshold: {:.3f}".format(mvel, mfreq,mtarget,thresh))
    if not mtarget:
      self.speed_var.set(" NaN")
      self.doppler_var.set(" NaN")
      self.target_var.set("No Target")
    else:
      self.speed_var.set(" {:.2f} [mph]".format(mvel))
      self.doppler_var.set(" {:.2f} [Hz]".format(mfreq))
      self.target_var.set(" {:.2f} [ ]".format(mtarget))
    self.update()
    self.after(100, self.update_data)



#
# Consider adding some basic arguments for configuration
if __name__=='__main__':

  ## Data Acquisition process
  rate=44000
  queue = Queue(maxsize=200)
  listener = Process(target=listen, kwargs={"queue":queue, "rate":rate, "device_index":0})
  listener.start()

  ## Main Application thread
  root = tk.Tk()
  app = App(root, queue, rate)
  app.mainloop()

  ## Tear down after exiting mainloop
  listener.terminate()
  time.sleep(.1)
  root.destroy()
