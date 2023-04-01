#!/usr/bin/env python3
import serial
import re


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()

    while True:
        if ser.in_waiting > 0:
            #line = ser.readline().decode('utf-8').rstrip()
            line = ser.readline().decode('utf-8')
            reading= re.match("(\d+.\d+);(\d+.\d+)",line).group()
            print(reading);
            if reading:
                voltage=float(reading[0:5])
                current=float(reading[7:12])
                print(voltage)
                print(current)
            else:
                print("No Match")
        
