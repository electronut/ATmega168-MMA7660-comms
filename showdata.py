################################################################################
# showdata.py
#
# Display MMA7660 accelerometer data
# 
# electronut.in
#
################################################################################

import sys, serial
import numpy as np
from time import sleep
from collections import deque
from matplotlib import pyplot as plt

# class that holds acceleration data for N samples
class AccelData:
  # constr
  def __init__(self, maxLen):
    self.ax = deque([0.0]*maxLen)
    self.ay = deque([0.0]*maxLen)
    self.az = deque([0.0]*maxLen)
    self.maxLen = maxLen

  # ring buffer
  def addToBuf(self, buf, val):
    if len(buf) < self.maxLen:
      buf.append(val)
    else:
      buf.pop()
      buf.appendleft(val)

  # add data
  def add(self, data):
    assert(len(data) == 5)
    self.addToBuf(self.ax, data[0])
    self.addToBuf(self.ay, data[1])
    self.addToBuf(self.az, data[2])
    
# plot class
class AccelPlot:
  # constr
  def __init__(self, accelData):
    # set plot to animated
    plt.ion() 
    self.axline, = plt.plot(accelData.ax)
    self.ayline, = plt.plot(accelData.ay)
    self.azline, = plt.plot(accelData.az)
    plt.ylim([-3.0,3.0])

  # update plot
  def update(self, accelData):
    self.axline.set_ydata(accelData.ax)
    self.ayline.set_ydata(accelData.ay)
    self.azline.set_ydata(accelData.az)
    plt.draw()

# main() function
def main():
  # use sys.argv if needed

  # plot parameters
  accelData = AccelData(100)
  accelPlot = AccelPlot(accelData)

  print 'plotting MMA7660 data...'
  strPort = '/dev/tty.usbserial-A7006Yqh'
  ser = serial.Serial(strPort, 9600)
  while True:
    try:
      line = ser.readline()
      data = line.split()
      # make sure it has data
      if data[0] == 'DATA':
        accelData.add(data[1:6])
        accelPlot.update(accelData)
      # print message
      else:
        print line        
    except KeyboardInterrupt:
      print 'exiting'
      break
  # close serial
  ser.flush()
  ser.close()

# call main
if __name__ == '__main__':
  main()
