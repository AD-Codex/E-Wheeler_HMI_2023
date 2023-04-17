import serial
import time
from decimal import *
from subprocess import call
import paho.mqtt.publish as publish

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
#print(rcvToString.split('\n'))
time.sleep(.1)

port.write(str.encode('AT+CGNSINF'+'\r\n')) #print GPS information
rcv=port.read(1000)
rcvToString= rcv.decode()
print(rcvToString,'\n')

#lon-lat extraction

for i in range(10):
    
    print('\n')
    
    rcv=port.read(1000)
    rcvToString= rcv.decode()   
    fd = rcvToString
    fdS = fd.split('\n')
    print(fdS)
    for element in fdS:
      if '$GNRMC' in element:
        rmc = element.split(',')
        print(rmc)
        if rmc[2] == 'A': #check validity
          
          latCheck = Decimal(rmc[3])
          lonCheck = Decimal(rmc[5])
          print('lat_row = ',latCheck,'\nlon_row = ',lonCheck)
          
          lat1 = rmc[3][2:len(rmc[3])]
          print('\nlat1 ',rmc[3][2:len(rmc[3])])
          lat1 = Decimal(rmc[3][2:len(rmc[3])])
          lat1 = lat1/60
          lat11= int(rmc[3][0:2])
          print('lat11 ',lat11)
          lat = lat11+lat1
          print('Latitude = ',Decimal(lat)) #decimal
       
          lon1 = rmc[5][3:len(rmc[3])]
          print('\nlon1 ',rmc[5][3:len(rmc[3])])
          lon1 = Decimal(rmc[5][3:len(rmc[3])])
          lon1 = lon1/60
          lon11= int(rmc[5][0:3])
          print('lon11 ',lon11)
          lon = lon11+lon1
          print('Longitude = ',Decimal(lon)) #decimal

          LatLon = str(lat)+'/'+str(lon)

          #publish lon lat

          publish.single("93/gps/v1/LatLon",LatLon,hostname = "test.mosquitto.org")
          print("\nLatitude/Longitude - PUBLISHED")
        

    time.sleep(10)

     
          


        

