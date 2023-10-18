#!/usr/local/bin/python3
#
#  Display.py     Load a wavefile and
#
#  Author         Arnie Larson
#
#############################

import sys, re
import daq


if __name__=='__main__':

  if len(sys.argv) < 2:
    print("Display.py usage:  ./display.py <wav_filename>")
  else:
    try:
      # could consider other parameters here..
      filename = sys.argv[1] if re.match(".*wav", sys.argv[1]) else sys.argv[1]+".wav"
      data = daq.load_and_display(filename)
    except Exception as e:
      print("Error running Display.py: {}".format(e))