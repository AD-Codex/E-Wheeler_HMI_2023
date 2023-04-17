import serial
import time
from decimal import *
from subprocess import call



port = serial.Serial("/dev/ttyUSB0", baudrate = 115200, timeout =1)

port.write(str.encode('AT'+'\r\n'))
rcv=port.read(100)
rcvToString= rcv.decode()
print(rcvToString.split())
print('\n')
time.sleep(.1)

port.write(str.encode('AT+CGNSPWR =0'+'\r\n')) #to power GPS
rcvP=port.read(100)
rcvToString2= rcvP.decode()
#print(rcvToString2)
#print('\n')
print(rcvToString2.split())
print('\n')
time.sleep(.1)
          
port.write(str.encode('AT+CGNSIPR =115200'+'\r\n')) #set baudrate of GPS
rcvB=port.read(100)
rcvToString3= rcvB.decode()
#print(rcvToString3.split())
time.sleep(.1)

'''

port.write(str.encode('AT+CGPSSTATUS?'+'\r\n'))
rcvStat=port.read(100)
rcvToStringS= rcvStat.decode()
print(rcvToStringS.split())
time.sleep(.1)

'''
