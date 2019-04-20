#!/usr/bin/python3

import serial
import curses
import time
from curses import wrapper #- See more at: http://ceilingmemo.blogspot.tw/2014/03/raspberry-pi_14.html#sthash.jlh89k9r.dpuf

#arduino=serial.Serial('/dev/ttyUSB0',9600)
#arduino=serial.Serial('/dev/ttyAMA0',9600)
arduino=serial.Serial('/dev/ttyS0',9600)   #mini UART

stdscr = curses.initscr()
stdscr.clear()


while True:
   
   ch = stdscr.getkey()
 # Quit 
   if ch == 'q':
      curses.endwin() 
      break
 # up
   if ch == 'w':
      arduino.write(b'2')
      print ('2') 
 # left
   if ch == 'a':
      arduino.write(b'4')
      print ('4') 
 # down
   if ch == 's':
      arduino.write(b'8')
      print ('8') 
 # right
   if ch == 'd':
      arduino.write(b'6')
      print ('6') 
 # fire
   if ch == 'f':
      arduino.write(b'5')
      print ('5') 

