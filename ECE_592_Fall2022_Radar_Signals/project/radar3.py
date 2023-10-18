#!/usr/local/bin/python3
#
# Note  I discovered that the better way to utilize multiple cores is with
#       multiprocessing.  There is a threadsafe Queue implemented that is
#       used to share data.  Since the other processes are running sequentially
#       (in a single core) there is likely no need for using locks
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



# should almost certainly add a lock here
datalock = threading.Lock()

KILL_PROC=False
KILL_LIST=False
## Using global variables - upgrade this to a class at some point
data=None
app=None
canvas=None
figure_canvas_agg=None
speed_var = None
target_var = None
doppler_var = None


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
  time.sleep(0.2)

  app.destroy()



# device_index = 0 => system microphone
# device_index = 2 => USB microphone (radar IF)
#
def listen(q, channels=1, rate=8000, chunk=1024, device_index=0):
  HALT=False
  print("Listening to device_index {} at fs = {}".format(device_index, rate))
  p = pyaudio.PyAudio()
  s = p.open(format=pyaudio.paInt16, channels=channels, rate=rate, input=True, frames_per_buffer=chunk, input_device_index=0)
  valid = p.is_format_supported(input_format=pyaudio.paInt16, input_channels=1, rate=rate, input_device=device_index)
  print("Listening to device_index {} at fs = {}, supported: {}".format(device_index, rate, valid))
  i = 0
  while True:
    try:
      buf = s.read(chunk) # 2048 byte chunks, 1024 ints
      #d = np.frombuffer(buf, dtype=np.int16)

      q.put(buf)
      print("[Listen] read data, putting {} into queue".format(type(buf)))
    except Exception as e:
      print("Exception in Listen: {}".format(e))

    if KILL_LIST:
      break
    if i > 10:
      break
    i+=1
  print("* exiting listener")
  if not s.is_stopped():
    s.stop_stream()
    s.close()
  p.terminate()



# get each data chunk
def process(q, T=3, rate=8000, chunk=1024 ):
  print("Processing data at rate: {}".format(rate))
  # N=T*rate  ## should do this programatically
  if rate == 8000:
    N=T*8*1024
  elif rate == 48000:
    N=T*48*1024
  else:
    N=100*1024
  rotate=False; update=True
  data = np.zeros(N, dtype=np.int16)
  off48=20
  # looks at last half second?
  vidx = (data.shape[0]-off48*chunk) if rate==48000 else (data.shape[0]-2*chunk)
  print("data size: {}, doppler processed size will be".format(data.shape, (data.shape[0]-vidx)))
  nperseg = 4096 if rate==48000 else 1024
  idx=0
  print("* starting data processor")
  i=0
  while True:
    if KILL_PROC:
      break
    if q.empty():
      print("[Process] no data, sleep(.1)")
      time.sleep(0.1)
      continue
    buf = q.get()
    d = np.frombuffer(buf, dtype=np.int16)
    print("[Process] got data {}".format(d.shape))

    size = d.shape[0]
    #print("buf ave: {:.3f}, max: {:.3f} min: {:.3f}".format(np.average(buf), buf.max(), buf.min()))
    #print("[Process] got data {} with shape {}".format(type(buf),type(size)))
    try:
      #print("updating data: idx: {}, buf size: {}, data_size: {}".format(idx, size, data.shape[0]))
      if not rotate:
        #idx_end=min(idx+buf.shape[0],data.shape[0])
        eidx=idx+size
        datalock.acquire()
        data[idx:eidx] = d

        idx=eidx
        if idx==data.shape[0]:
          rotate=True
        datalock.release()
      else:
        #print("Rotating new data")
        datalock.acquire()
        end_idx=data.shape[0]-size
        data[0:end_idx]=data[size:]
        data[end_idx:]=d
        datalock.release()

    except Exception as e:
      print("Exception e: {}".format(e))
    if i > 10:
      break
    i+=1

  print("* exiting processor")

# Canvas visualization
# Requires locking data then processign to get spectrogram
def draw_figure():
  # whatever data is in the buffer..

  spec = True
  datalock.acquire()
  # If data is all zeros that's a problem
  if data.max() == data.min():
    return
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

## TKinter callback to process and update the radar information
def update_radar():
  # again - put this stuff into a class
  off48=20; chunk=1024; rate=4800
  # looks at last half second?
  vidx = (data.shape[0]-off48*chunk) if rate==48000 else (data.shape[0]-2*chunk)
  datalock.acquire()
  if data.max() == data.min():
    return
  else:
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
  app.after(100, update_radar)

if __name__=="__main__":

  # get whatever params are needed via sys.argv
  rate=48000
  N=6*50*1024
  data = np.zeros(N, dtype=np.int16)
  """
  app = tk.Tk()
  canvas = tk.Canvas(app, height=200, width=400)
  canvas.pack()
  figure = plt.gcf()
  figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
  draw_figure()

  frame=ttk.Frame(app, padding=100)
  frame.grid()

  speed_var = tk.StringVar(frame, "NaN")
  target_var = tk.StringVar(frame, "Nan")
  doppler_var = tk.StringVar(frame, "Nan")

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
  #app.geometry( "800x1200" )
  """
  # contains buffers
  q = Queue(maxsize=10000)

  # this will start in a different process
  listener = Process(target=listen, kwargs={"q":q, "rate":rate})
  listener.start()
  # thread collects data and periodically calculates transforms and updates
  # cannot pass in the TK objects to the Process
  time.sleep(1)
  #processor = Process(target=process, kwargs= {"q":q, "rate":rate})
  #processor = Process(target=process, kwargs= {"q":q, "rate":rate})
  #processor.start()
  process(q, rate=rate)
  #app.mainloop()
  print("[Main] Main loop done, calling terminate() on all threads")
  listener.terminate()
  #processor.terminate()
