#!/usr/bin/env python

from threading import Thread
import paho.mqtt.publish as publish
import RPi.GPIO as GPIO
import time

Tcount = 0
zeroTimeout = 1
RPM = 0
lastSpeed = 0
Speed = 0


GPIO.setmode(GPIO.BCM)
interrupt_pin = 16

lastCountTime = time.time()
newCountTime = lastCountTime

def pulse_event(channel):
    global newCountTime
    global count
    global Tcount
    
    newCountTime = time.time()
    Tcount = Tcount +1

GPIO.setup(interrupt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(interrupt_pin, GPIO.FALLING, callback=pulse_event, bouncetime=100)



'''
while True:
    CurrentTime = time.time()
    #publish.single("93/bms/velo",round(Speed, 2),hostname = "test.mosquitto.org") 
    
    if ((newCountTime - lastCountTime) > 0.01):
        #print(newCountTime - lastCountTime)
        RPM = 60.00/(newCountTime - lastCountTime)
        Speed = RPM*2*3.14*0.22*60*0.001
        lastCountTime = newCountTime
    elif ( (CurrentTime - lastCountTime) > zeroTimeout ):
        RPM = 0.0
        Speed = 0.0
        
    if (Tcount <= 1):
        Speed = 0.0
        RPM = 0.0
        
    time.sleep(0.1)
    print("RPM : {x:0.2f}".format(x=RPM),"Speed : {x:0.2f}".format(x=Speed) , "Tcount :", Tcount)
 '''           


def WriteData():
    global Speed
    global Tcount
    
    while True:
        drive = Tcount * 2 * 3.14 * 0.22 * 0.001
        publish.single("93/bms/velo",round(int(Speed)),hostname = "test.mosquitto.org")
        publish.single("93/bms/drive",round(drive, 2),hostname = "test.mosquitto.org")
        

def SpeedRead():
    global newCountTime
    global count
    global Tcount
    global Speed
    global lastSpeed
    global lastCountTime
    
    while True:
        CurrentTime = time.time()
    
        if ((newCountTime - lastCountTime) > 0.01):
            #print(newCountTime - lastCountTime)
            RPM = 60.00/(newCountTime - lastCountTime)
            Speed = RPM*2*3.14*0.22*60*0.001
            lastCountTime = newCountTime
            if ( abs(Speed - lastSpeed) > 20 ):
                print("----------------------------error-----------------------------")
                Speed = lastSpeed
        elif ( (CurrentTime - lastCountTime) > zeroTimeout ):
            RPM = 0.0
            Speed = 0.0
        
        if (Tcount <= 1):
            Speed = 0.0
            RPM = 0.0
        
        time.sleep(0.1)
        print("RPM : {x:0.2f}".format(x=RPM),"Speed : {x:0.2f}".format(x=Speed) , "Tcount :", Tcount)
        lastSpeed = Speed
        
  
if __name__ == "__main__":
    
    t1 = Thread( target = SpeedRead)
    t2 = Thread( target = WriteData)
    
    t1.start()
    t2.start()
    
    t1.join()
    t2.join()
    

