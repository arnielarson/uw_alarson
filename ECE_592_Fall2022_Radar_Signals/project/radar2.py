#!/usr/local/bin/python3

import queue
import threading
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

# contains buffers
q = queue.Queue(maxsize=10000)

# should almost certainly add a lock here
datalock = threading.Lock()

KILL_PROC=False
KILL_LIST=False
data=None
app=None
canvas=None
figure_canvas_agg=None

def collect():
  print("* Starting collection")
  global GO
  go.acquire()
  GO = True
  go.release()

def kill():
  global KILL_LIST
  global KILL_PROC
  KILL_LIST = True
  KILL_PROC = True
  time.sleep(0.5)
  canvas.destroy()
  time.sleep(1.0)
  app.destroy()

def draw_figure():
  # whatever data is in the buffer..

  spec = True
  datalock.acquire()
  if data.max() == data.min():
    spec = False
  else:
    f, t, S = signal.spectrogram(data, 48000, nperseg=1024) #, nfft=256*2)
  datalock.release()
  print("[Draw Figure] {}".format(spec))

  if spec:
    #canvas.delete('all')
    df = 48000/(2*f.shape[0])
    max_fidx= int(1500/df)

    print("[Spectrogram] S ave: {:.3f}, max: {:.3f} min: {:.3f}".format(np.average(S), S.max(), S.min()))
    #plt.gca.clear() # clear ases?
    plt.clf()
    plt.pcolormesh(t, f[:max_fidx], S[:max_fidx], shading="auto", norm=colors.LogNorm(vmin=S.min() + 1e-6, vmax=S.max()))
    plt.xlabel("Time [s]"), plt.ylabel("Frequency [Hz]")
    plt.title("Spectrogram")


    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=1)
    figure_canvas_agg.draw()

    app.update()
  canvas.after(200, draw_figure)

#
#
def listen(channels=1, rate=8000, chunk=1024, device_index=2):

  print("Listening to device_index {} at fs = {}".format(device_index, rate))
  p = pyaudio.PyAudio()
  s = p.open(format=pyaudio.paInt16, channels=channels, rate=rate, input=True, frames_per_buffer=chunk, input_device_index=0)
  valid = p.is_format_supported(input_format=pyaudio.paInt16, input_channels=1, rate=rate, input_device=device_index)
  print("Listening to device_index {} at fs = {}, supported: {}".format(device_index, rate, valid))
  while True:
    try:
      buf = s.read(chunk) # 2048 byte chunks, 1024 ints
      d = np.frombuffer(buf, dtype=np.int16)
      q.put(d)

    except Exception as e:
      print("Exception in Listen: {}".format(e))

    if KILL_LIST:
      break
  print("* exiting listener")
  if not s.is_stopped():
    s.stop_stream()
    s.close()
  p.terminate()



# get each data chunk
def process(speed, freq, target, max_speed, max_target, app, T=3, rate=8000, chunk=1024 ):
  print("Processing data at rate: {}".format(rate))
  # N=T*rate  ## should do this programatically
  if rate == 8000:
    N=T*8*1024
  elif rate == 48000:
    N=T*48*1024
  else:
    N=100*1024
  rotate=False; update=True

  off48=20
  # looks at last half second?
  vidx = (data.shape[0]-off48*chunk) if rate==48000 else (data.shape[0]-2*chunk)
  print("data size: {}, doppler processed size will be".format(data.shape, (data.shape[0]-vidx)))
  nperseg = 4096 if rate==48000 else 1024
  idx=0
  print("* starting data processor")
  t_max = 0
  v_max = 0
  i = 0
  now=time.time()
  while True:
    if KILL_PROC:
      break

    buf = q.get() # I think this is blocking
    size = buf.shape[0]
    #print("buf ave: {:.3f}, max: {:.3f} min: {:.3f}".format(np.average(buf), buf.max(), buf.min()))
    #print("got data {} with shape {}".format(type(buf),type(size)))
    try:
      #print("updating data: idx: {}, buf size: {}, data_size: {}".format(idx, size, data.shape[0]))
      if not rotate:
        #idx_end=min(idx+buf.shape[0],data.shape[0])
        eidx=idx+size
        datalock.acquire()
        data[idx:eidx] = buf

        idx=eidx
        if idx==data.shape[0]:
          rotate=True
        datalock.release()
      else:
        #print("Rotating new data")
        datalock.acquire()
        end_idx=data.shape[0]-size
        data[0:end_idx]=data[size:]
        data[end_idx:]=buf
        datalock.release()

    except Exception as e:
      print("Exception e: {}".format(e))

    if rotate and update:
      update = False
      datalock.acquire()
      mtarget,mfreq,mvel,thresh = dop.get_doppler(data[vidx:], rate, nperseg)
      datalock.release()
      print("Doppler vel: {:.3f} [mph], freq: {:.3f} [Hz], target_strength: {:.3f}, threshold: {:.3f}".format(mvel, mfreq,mtarget,thresh))
      if not mtarget:
        speed.set(" NaN")
        freq.set(" NaN")
        target.set("No Target")
      else:
        if mvel > v_max:
          max_speed.set(" {:.2f} [mph]".format(mvel))
          v_max = mvel
        if mtarget > t_max:
          max_target.set(" {:.2f} [ ]".format(mtarget))
          t_max = mtarget
        speed.set(" {:.2f} [mph]".format(mvel))
        freq.set(" {:.2f} [Hz]".format(mfreq))
        target.set(" {:.2f} [ ]".format(mtarget))

      app.update()
    i+=1
    #counter.set(" {}".format(i))
    if rate==48000 and i%off48 == 0:
      delta = now - time.time()
      now=time.time()
      #print("Setting update to true, frames processed {}, (timing {:.3f} [s])".format(i, delta))
      update = True


  print("* exiting processor")




if __name__=="__main__":
  p = pyaudio.PyAudio()
  info = p.get_host_api_info_by_index(0)
  numdevices = info.get('deviceCount')

  for i in range(0, numdevices):
      if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
          print("Input Device id ", i, " - ", p.get_device_info_by_host_api_device_index(0, i).get('name'))

  device = p.get_device_info_by_host_api_device_index(0,2)
  print("device type: {}".format(type(device)))
  for k, e in device.items():
    print("{}: {}".format(k,e))
  p.terminate()
  # get whatever params are needed via sys.argv
  rate=48000
  N=6*50*1024
  data = np.zeros(N, dtype=np.int16)
  app = tk.Tk()
  canvas = tk.Canvas(app, height=400, width=400)
  canvas.pack()
  figure = plt.gcf()
  figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
  draw_figure()

  frame=ttk.Frame(app, padding=100)
  frame.grid()

  speed_var = tk.StringVar(frame, "NaN")
  target_var = tk.StringVar(frame, "Nan")
  max_speed_var = tk.StringVar(frame, "NaN")
  max_target_var = tk.StringVar(frame, "Nan")
  doppler_var = tk.StringVar(frame, "Nan")
  count_var = tk.StringVar(frame, "Nan")

  ttk.Label(frame, text="Speed  ", font=("Helvetica", 14)).grid(column=0, row=0)
  ttk.Label(frame, textvariable=speed_var).grid(column=1, row=0)
  ttk.Label(frame, text="Frequency").grid(column=0, row=1)
  ttk.Label(frame, textvariable=doppler_var).grid(column=1, row=1)
  ttk.Label(frame, text="Target Strength").grid(column=0, row=2)
  ttk.Label(frame, textvariable=target_var).grid(column=1, row=2)
  #ttk.Label(frame, text="Processed").grid(column=0, row=3)
  #ttk.Label(frame, textvariable=count_var).grid(column=1, row=3)

  #ttk.Label(frame, text="Max Speed ", font=("Helvetica", 14)).grid(column=4, row=0)
  #ttk.Label(frame, textvariable=max_speed_var).grid(column=5, row=0)
  ##ttk.Label(frame, text="Max Target Strength").grid(column=4, row=1)
  #ttk.Label(frame, textvariable=max_target_var).grid(column=5, row=1)


  #ttk.Button(frame, text="Collect", command=collect).grid(column=0, row=4)

  ttk.Button(frame, text="Exit", command=kill).grid(column=1, row=4)
  #collect.bind("<button>",collect_click)
  frame.pack()
  app.title("Arnie's Toy Radar Applicaiton")
  app.geometry( "800x1200" )
  listener = threading.Thread(target=listen, kwargs={"rate":rate})
  listener.start()
  # thread collects data and periodically calculates transforms and updates

  processor = threading.Thread(target=process, args=(speed_var, doppler_var, target_var, max_speed_var, max_target_var), kwargs= {'app':app, "rate":rate})
  processor.start()
  app.mainloop()
