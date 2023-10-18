#!/usr/local/bin/python3
# simple script to look at audio devices

import pyaudio

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
