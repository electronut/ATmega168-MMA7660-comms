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

# main() function
def main():
  # use sys.argv if needed
  print 'hello'
  strPort = '/dev/tty.usbserial-A7006Yqh'
  ser = serial.Serial(strPort, 9600)
  while True:
    try:
      line = ser.readline()
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
