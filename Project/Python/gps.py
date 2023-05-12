'''
GPS Interfacing with Raspberry Pi using Pyhton
http://www.electronicwings.com
'''
import serial               #import serial pacakge
from time import sleep
import sys                  #import system package
import paho.mqtt.publish as publish


def GPS_Info():
    global NMEA_buff
    global lat_in_degrees
    global long_in_degrees
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
    nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
    nmea_longitude = NMEA_buff[3]               #extract longitude from GPGGA string
    
    lat = float(nmea_latitude)                  #convert string into float for calculation
    longi = float(nmea_longitude)               #convertr string into float for calculation
    
    lat_in_degrees = convert_to_degrees(lat)    #get latitude in degree decimal format
    long_in_degrees = convert_to_degrees(longi) #get longitude in degree decimal format
    
    return lat_in_degrees, long_in_degrees

#convert raw NMEA string into degree decimal format   
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.4f" %(position)
    return position
    



gpgga_info = "$GPGGA,"
ser = serial.Serial ("/dev/ttyUSB1")              #Open port with baud rate
GPGGA_buffer = 0
NMEA_buff = 0
lat_in_degrees = 0
long_in_degrees = 0


def GPS:
    while True:
        try:
            received_data = (str)(ser.readline())                   #read NMEA string received
            GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string                 
            if (GPGGA_data_available>0):
                GPGGA_buffer = received_data.split("$GPGGA,",1)[1]  #store data coming after "$GPGGA," string 
                NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
                
                lat_in_degrees, long_in_degrees = GPS_Info()                                          #get time, latitude, longitude
         
                print("lat in degrees:", lat_in_degrees," long in degree: ", long_in_degrees, '\n')
                lat_long = str(lat_in_degrees) + '/' + str(long_in_degrees)
                publish.single("93/gps/v1/LatLon",lat_long,hostname = "localhost")
            sleep(1)
                                
        except :
            print("GPS restarting .......................")
            sleep(3)
            GPS()

print("GPS  starting ..................")
GPS()