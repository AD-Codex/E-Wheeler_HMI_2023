
import time
import serial
import os, time
cmd=''
ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
ser.reset_input_buffer()
while True:
    smd=raw_input("please enter a command: ")
    smd=smd+'\n'
    smds=smd.encode('utf-8')
    ser.write(smds)
    print("smd value is:",smd)
    line = ser.read(10).decode('utf-8').rstrip()
    print(line)

