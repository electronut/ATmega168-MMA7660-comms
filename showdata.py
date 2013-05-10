################################################################################
# showdata.py
#
# Author: Mahesh Venkitachalam
# Created:
#
# Description:
#
################################################################################

import sys, serial
import numpy as np
from time import sleep
from collections import deque
from matplotlib import pyplot as plt

maxLen = 100
# data arrays
ax = deque([0]*maxLen)
ay = deque()
az = deque()

def updateData(line, xval):
  if len(ax) < maxLen:
    ax.append(xval)
  else:
    ax.pop()
    ax.appendleft(xval)
  line.set_ydata(ax)
  plt.draw()

# main() function
def main():
  # use sys.argv if needed

  # plot parameters
  plt.ion() # set plot to animated

  # make plot
  xline, = plt.plot(ax)
  plt.ylim([-2.5,2.5])

  print 'plotting MMA7660 data...'
  strPort = '/dev/tty.usbserial-A7006Yqh'
  ser = serial.Serial(strPort, 9600)
  while True:
    try:
      line = ser.readline().split()
      #print line[0]
      #print line[1:]
      updateData(xline, line[1])
    except KeyboardInterrupt:
      print 'exiting'
      break
  # close serial
  ser.flush()
  ser.close()

# call main
if __name__ == '__main__':
  main()
