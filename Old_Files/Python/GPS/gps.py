import serial
import time
from decimal import *
from subprocess import call

port = serial.Serial("/dev/ttyUSB0", baudrate = 115200, timeout =1)

port.write(str.encode('AT'+'\r\n'))
rcv=port.read(100)
rcvToString= rcv.decode()
print(rcvToString.split())
time.sleep(.1)

port.write(str.encode('AT+CGNSPWR =1'+'\r\n')) #to power GPS
rcvP=port.read(100)
rcvToString= rcvP.decode()
print(rcvToString.split())
time.sleep(.1)
          
port.write(str.encode('AT+CGNSIPR =115200'+'\r\n')) #set baudrate of GPS
rcvB=port.read(100)
rcvToString= rcvB.decode()
print(rcvToString.split())
time.sleep(.1)

port.write(str.encode('AT+CGNSTST=1'+'\r\n')) #send data received to UART
rcvU=port.read(1000)
rcvToString2= rcvU.decode()
#print(rcvToString)
#print(rcvToString2.split())
time.sleep(.1)

port.write(str.encode('AT+CGNSINF'+'\r\n')) #print GPS information
rcvI=port.read(1000)
rcvToString= rcvI.decode()
print(rcvToString)
#print(rcvToString.split())
#print(type(rcvToString))
time.sleep(.1)


