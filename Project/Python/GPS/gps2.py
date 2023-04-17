import serial
import time
from decimal import *
from subprocess import call

port = serial.Serial("/dev/ttyUSB0", baudrate = 115200, timeout =1)

port.write(str.encode('AT'+'\r\n'))
rcv=port.read(100)
rcvToString= rcv.decode()
#print(rcvToString.split())
time.sleep(.1)

port.write(str.encode('AT+CGNSPWR =1'+'\r\n')) #to power GPS
rcv=port.read(100)
rcvToString= rcv.decode()
#print(rcvToString.split())
time.sleep(.1)
          
port.write(str.encode('AT+CGNSIPR =115200'+'\r\n')) #set baudrate of GPS
rcv=port.read(100)
rcvToString= rcv.decode()
#print(rcvToString.split())
time.sleep(.1)

port.write(str.encode('AT+CGNSTST=1'+'\r\n')) #send data received to UART
rcv=port.read(1000)
rcvToString= rcv.decode()
#print(rcvToString.split())
time.sleep(.1)

port.write(str.encode('AT+CGNSINF'+'\r\n')) #print GPS information


for i in range (5):
    rcv=port.read(1000)
    rcvToString= rcv.decode()
    print(rcvToString)
    print('\n')
    #print(rcvToString.split())
    #print('\n')
    print(type(rcvToString))
    print('\n')
    time.sleep(.1)


