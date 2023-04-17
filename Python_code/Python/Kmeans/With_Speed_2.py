#!/usr/bin/env python

from threading import Thread
import numpy as np
from numpy.linalg import inv
import csv
import paho.mqtt.publish as publish
import serial
import time as timeDelay
import serial.tools.list_ports
import re
import RPi.GPIO as GPIO
import smbus
import math

#inital port
port = []

global VOLTAGE_READ
global CURRENT_READ
global TEMP_READ
global ANG_READ
global DISTANCE
global VELOCITY_READ
global SOC_CALCULATION

VOLTAGE_READ = 0
CURRENT_READ = 0
VELOCITY_READ = 0
SOC_VALUE = 0


Tcount = 0
zeroTimeout = 1
RPM = 0
lastSpeed = 0
Speed = 0


GPIO.setmode(GPIO.BCM)
interrupt_pin = 16

lastCountTime = timeDelay.time()
newCountTime = lastCountTime

def pulse_event(channel):
    global newCountTime
    global count
    global Tcount
    
    newCountTime = timeDelay.time()
    Tcount = Tcount +1

GPIO.setup(interrupt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(interrupt_pin, GPIO.FALLING, callback=pulse_event, bouncetime=100)


def WriteData():
    global Speed
    global Tcount
    
    while True:
        drive = Tcount * 2 * 3.14 * 0.22 * 0.001
        publish.single("93/bms/velo",round(Speed),hostname = "test.mosquitto.org")
        publish.single("93/bms/drive",round(drive, 2),hostname = "test.mosquitto.org")
                




def find_port():
    global port
    find = 0
    print('Finding port for GSM')
    while( find == 0):
        p=serial.tools.list_ports.comports()
        n=len(p)
        print(p)
        for i in range(0,n):
            port.append( p[i][1])
            print( p[i][1])
            if ( p[i][1] != ''):
                print('Found Port '+str(p[i][1]))
                find = 1
        timeDelay.sleep(0.5)



def SpeedRead():
    global newCountTime
    global count
    global Tcount
    global Speed
    global lastSpeed
    global lastCountTime
    global VELOCITY_READ
    
    power_mgmt_1 = 0x6b
    power_mgmt_2 = 0x6c
    
    while True:
        CurrentTime = timeDelay.time()
    
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
        
        VELOCITY_READ = Speed
        print("RPM : {x:0.2f}".format(x=RPM),"Speed : {x:0.2f}".format(x=Speed) , "Tcount :", Tcount)
        timeDelay.sleep(0.1)
        lastSpeed = Speed
    




class thread0:
    def __int__( self):
        self.Dcount = 0
        
    def dataRead( self):
        global VOLTAGE_READ
        global CURRENT_READ
        global TEMP_READ
        global ANG_READ
        global DISTANCE
        
        try:
            while True:
                if ser.in_waiting > 0:
                    data = ser.readline()
                    timeDelay.sleep(0.1)
                    if ( data != b'' and data != b'\r\n'):
                        #print("Recived :", str(data))
                        value = str(data).split(";")
                        print("")
                        #print( "Recived data V:", value[0], "; I:", value[1], "; T:", value[2], "; Ang:", value[3])
    
                        VOLTAGE_READ = float(value[0])
                        CURRENT_READ = float(value[1])
                        TEMP_READ = float(value[2])
                        ANG_READ = float(value[3])

                        publish.single("93/bms/temp",TEMP_READ,hostname = "test.mosquitto.org")
        except:
            print("------------------------------ reading error ----------------------------")

            



class thread1:    
    
    def __init__( self):
        self.V0 = 0
        self.I0 = 0
        self.Speed = 0
        self.SOC = 0
        self.Capacity = 24
        self.Power = 0
        self.P_avg = 0
        self.S_avg = 0
        self.P_list = []
        self.S_list = []
        self.count = -1
        self.sTime = 5
        self.text = "hello from thred1 "
        

    def run( self):
        global VOLTAGE_READ
        global CURRENT_READ
        global VELOCITY_READ
        global DISTANCE
        global SOC_VALUE
        
        S_data_file = open('S_file.txt', 'w')
        P_data_file = open('P_file.txt', 'w')
        
        while True:
            self.V0 = VOLTAGE_READ
            self.I0 = CURRENT_READ
            self.Speed = VELOCITY_READ / 3.6
            self.SOC = SOC_VALUE
            self.count = self.count + 1
            
            self.Power = self.V0 * self.I0
            #print(self.Power)
            
            self.P_list.append( self.Power)
            self.S_list.append( self.Speed)
            
            S_data_file = open('S_file.txt', 'a')
            P_data_file = open('P_file.txt', 'a')
            line = "%f\n" % (self.Speed)
            S_data_file.write(line)
            line = "%f\n" % (self.Power)
            P_data_file.write(line)
            
    
            #print( "voltage:", self.V0 ," current:", self.I0, "speed:", self.Speed, "SOC:", self.SOC)
            
            if (self.count == self.sTime -1):
                print( self.P_list)
                print( self.S_list)
                P_sum = 0
                S_sum = 0
                
                for i in range(self.sTime):
                    P_sum = P_sum + self.P_list[i]
                    S_sum = S_sum + self.S_list[i]

                    
                self.P_avg = P_sum/self.sTime
                self.S_avg = S_sum/self.sTime
                print("P_avg:", self.P_avg)
                print("S_avg", self.S_avg)
                
                if ( self.P_avg <= 40 and self.S_avg <= 40):
                    map_value = 0.234
                elif ( self.P_avg <= 40 and self.S_avg > 40):
                    map_value = 0.333
                elif ( self.P_avg > 40 and self.S_avg <= 40):
                    map_value = 0.433
                elif ( self.P_avg > 40 and self.S_avg > 40):
                    map_value = 0.533
                

                distance = self.SOC*self.Capacity / (map_value*100)
                if ( self.S_avg == 0.0) :
                    distance = 0.0
                DISTANCE = distance
                publish.single("93/bms/dis",round(distance, 2),hostname = "test.mosquitto.org")
                print("distance can go:", DISTANCE)
                
                del self.P_list[:]
                del self.S_list[:]
                self.count = -1
                
            timeDelay.sleep(1)
            
            


class thread2:
    def __init__( self):
        
        #initial values assigned
        self.I0 = 0 #Input current
        self.soh = 0.8 
        self.soc = 0.78 #Assumed full charged

        self.time = []
        self.SOH = []
        self.SOC = []
        self.Vm = []
        self.Vp = []
        self.Im = []
  
        self.sampleTime =0.2

        # 3 states declaration
        self.soc1 = self.soc
        self.soh1 = self.soh
        self.Vth1 = 0
        
        #Process Error Co-variance Matrix (Q) E[XX^T]
        self.Qk = np.array([[0.05 , 0 , 0],[0, 0.04, 0],[ 0, 0, 1]])
        
        #print('Qk',Qk,Qk.shape)
        
        #Initial Process Error Co-variance Matrix (Pkint = Qkint)
        self.Pkint = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
        #print('\n Pkint',Pkint,Pkint.shape)
        self.Pkin = self.Pkint
        
        #Measurement Error Co-variance Matrix (R)
        self.R = np.array([0.5]) #(1,)
         
         

    def statediff ( self, x,u):
        #separate terms from the state vector and input vector
        Ssoh = x[0]
        Ssoc = x[1]
        vc = x[2]
        current = float(u)
        #print('\n current Sttediff ', current, type(current))
        #Get state-dependent parameters. Rth & Cth
        r2= 2.5968*(Ssoc**4)-3.5211*(Ssoc**3)+ 0*(Ssoc**2)+1.4757*(Ssoc)-0.3317 #as per the matlab code
        #r2 = 10.6048 * soc **4 - 15.1475 * soc ** 3 + 0 * soc ** 2 + 7.1826 * soc - 2.0363 # Rth float
        c2=200000
        #c2 = 90000 #constant
        # Capacity changes with soh
        
        #~~~calculating capacity~~~
        CAPACITY_NEW =20;
        CAPACITY_OLD_PERCENT =80
        
        alpha =  CAPACITY_OLD_PERCENT /100
        beta = 1 - alpha
        capacity = CAPACITY_NEW * (alpha + beta*Ssoh)
        
        #calculate the rate of change of capacity
        #dcapacity = derivStruct();
        #dcapacity.dsoh = CAPACITY_NEW * beta ;
        lifetime = 2000; #const depend on battery
        
        #Implement state update equation
        dsoh = -abs(current) / (2*lifetime*capacity*3600)
        dsoc = - current / (capacity*3600)
        dvc = (-vc) /(c2*r2) + (current/c2)
        #dtemp = 0
        
        arrayOut = np.array([dsoh,dsoc,dvc])
        #print('\n arrayOut Sttediff ', arrayOut, arrayOut.shape, type(arrayOut) )
        return arrayOut
    
    

    def run( self):
        global VOLTAGE_READ
        global CURRENT_READ
        global SOC_VALUE
        
        while True:            
            V0 = VOLTAGE_READ
            I0 = CURRENT_READ                 
            #V0 = 25.000
            #I0 = 3.000
            
            #print( "Voltage:", V0, "; Current:", I0)   
                
            r2 = 2.5968*(self.soc**4)-3.5211*(self.soc**3)+ 0*(self.soc**2)+1.4757*(self.soc)-0.3317  # Rth float
            Vth = float(I0) * r2
            self.Vth1 = Vth
                    
            # ~~~~ Neglecting errors ~~~~ 
            r2y = 2.5968*(self.soc**4)-3.5211*(self.soc**3)+ 0*(self.soc**2)+1.4757*(self.soc)-0.3317 # Rth float
                    
            '''
            print('\n soc1',self.soc1)
            print('\n soh1',self.soh1)
            print('\n Vth1',self.Vth1)
            '''
            #Vth calculation
            self.Vth1 = float(I0) * r2y

            y = np.array([[self.soh1],[self.soc1],[self.Vth1]]) # (3,1)
            #print('\n y ', y, y.shape)

            yPredict = y + (self.statediff(y,float(I0))*0.2) #considering no errors writing coulomb counting (3,1)
            #print('\n yPredict ',yPredict, yPredict.shape,type(yPredict))

            self.soh1 = yPredict[0,0]
            self.soc1 = yPredict[1,0]
            self.Vth1 = yPredict[2,0]
            #print('\n yPredict[0] yPredict[1] yPredict[2] typeSOH ',yPredict[0],yPredict[1],yPredict[2], type(yPredict[0]), type(self.soh1))

            Voc1=9.8958*(self.soc1)**4-19.3750*(self.soc1)**3+15.1042*(self.soc1)**2-3.9250*(self.soc1)+24.06

            r01 = 0.0002
            voltage=float(V0)

            #~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    
            #current state of the battery
            x = np.array([[self.soh],[self.soc],[Vth]]) #(3,1)
            #print('\n x ', x, x.shape)

            #find the prediction of x
            xPredict = x + (self.statediff(x,float(I0))*self.sampleTime) #(3,1)
            #print('\n xPredict ',xPredict, xPredict.shape,type(xPredict))

            self.soh = xPredict[0,0]
            self.soc = xPredict[1,0]
            self.Vth = xPredict[2,0]
            #print('\n xPredict[0] xPredict[1] xPredict[2] typeSOH ',xPredict[0],xPredict[1],xPredict[2], type(xPredict[0]), type(soh))


            #calculation of thevenin model parameters
            r0 = 0.0002
            c2 = 200000
            Voc=9.8958*(self.soc**4)-19.3750*(self.soc**3)+15.1042*(self.soc**2)-3.9250*(self.soc)+24.06

            #~~~~~~~~~~~~~~~~
            #predicted output
            HKPredict = Voc-Vth-float(I0)*r0 # Vout predict
            #print('\n HKPredict',HKPredict, type(HKPredict))

            #jacobian of state functions
            Fk = np.array([[1,0,0],[ 0,1,0], [0,0,(1/(r2*c2))]]) #constant here (3,3)
            #print('\n Fk ', Fk, Fk.shape, type(Fk))

            #jacobian of output function
            HK = np.array([[0 ,0, -1]]) #need 2 brackets to (1,3)
            #print('\n HK ', HK, HK.shape, type(HK))
            HKT = HK.T
            #print('\n HKT ', HKT, HKT.shape, type(HKT))

            FkT = Fk.T #FkT=Fk.transpose()
            Pk = np.dot(Fk,np.dot(self.Pkin,FkT))+self.Qk
                    
            '''
            print('\n Pk ', Pk, Pk.shape, type(Pk))
            print('\n check ' ,np.dot(Fk,np.dot(Pkin,FkT)))
            print('\n kui ' ,Qk)  
            print('\n check2 ' ,np.dot(Fk,np.dot(Pkin,FkT))+Qk)
            '''
            #kalman gain
            Gk = np.dot(Pk,np.dot(HKT,inv(np.dot(HK,np.dot(Pk,HKT))+self.R)))
            #print('\n Gk ', Gk, Gk.shape, type(Gk)) #(3,1)

            xest = xPredict+ (Gk*(voltage-HKPredict)); #equation Dimension Issue corrected
            #print('\n xest ', xest, xest.shape, type(xest)) #(3,1)
            #print('\n np.dot(Gk,(voltage-HKPredict ', np.dot(Gk,(voltage-HKPredict))) #(3,1)


            PK = np.dot((np.identity(3)-(np.dot(Gk,HK))),Pk); #equation

            #next loop update
            self.soh = xest[0,0];
            self.soc = xest[1,0];
            Vth = xest[2,0];

            vest=Voc-Vth-float(I0)*r0; #not necessary seems
            self.Qk = self.Pkin
            self.Pkin = PK
                
            #time.append(float(line[1]))
            self.Vm.append(float(V0)) 
            self.Im.append(float(I0))
            self.SOH.append(self.soh)
            self.SOC.append(self.soc)
            self.Vp.append(HKPredict)
                    
            SOC_VALUE = self.soc * 100
                
            publish.single("93/bms/v1/soc",round(self.soc*100, 2),hostname = "test.mosquitto.org")
            #print("SOC - PUBLISHED")
            publish.single("93/bms/v1/soh",round(self.soh*100, 2),hostname = "test.mosquitto.org")
            #print("SOH - PUBLISHED")
            print("Voltage:", V0, "Current:", I0,"SOC:", round(self.soc*100, 2), "; SOH:", round(self.soh*100, 2))
            timeDelay.sleep(1)
                
                


if __name__ == '__main__':
    find_port()

    print(port)
    try:
        port_name = "/dev/" + port[0]
        #print(port_name)
        port_name = "/dev/ttyUSB0"
        ser = serial.Serial( port_name, 9600, timeout=1)
        ser.reset_input_buffer()
    except:
        print("---------------------------- USB port error ---------------------------------")
    
    t1 = Thread( target = SpeedRead)
    t2 = Thread( target = WriteData)
    
    
    Zero = thread0()
    ZeroTread = Thread( target = Zero.dataRead)
    ZeroTread.start()
    
    
    Second = thread2()
    SecondTread = Thread( target = Second.run)
    SecondTread.start()
    

    First = thread1()
    FirstTread = Thread( target = First.run)
    FirstTread.start()
    
    t1.start()
    t2.start()
    
    
    

    
