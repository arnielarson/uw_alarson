#!/usr/local/bin/python3
#
#  Record.py      Collect T seconds of data and write data to file
#
#  Author         Arnie Larson
#
#############################

import sys, re, os, time
import daq

if __name__=='__main__':

  if len(sys.argv) < 3:
    print("Record.py usage:  ./record.py T_seconds wav_filename [sample_rate]")
  else:
    try:

      #time.sleep(10)
      print("Recording data")
      # could consider other parameters here..
      filename = sys.argv[2] if re.match(".*wav", sys.argv[2]) else sys.argv[2]+".wav"
      T=int(sys.argv[1])
      if len(sys.argv) >3:
        fs = int(sys.argv[3])
        data = daq.collect_and_save(T, daq.get_data_path(filename), rate=fs)
      else:
        data = daq.collect_and_save(T, daq.get_data_path(filename))
    except Exception as e:
      print("Error running Record.py: {}".format(e))