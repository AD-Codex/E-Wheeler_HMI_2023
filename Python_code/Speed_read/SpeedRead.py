#!/usr/bin/env python3


import RPi.GPIO as GPIO
import time

Tcount = 0
zeroTimeout = 3
RPM = 0
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



while True:
    CurrentTime = time.time()
    
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
        
    print("RPM : {x:0.2f}".format(x=RPM),"Speed : {x:0.2f}".format(x=Speed) , "Tcount :", Tcount)


