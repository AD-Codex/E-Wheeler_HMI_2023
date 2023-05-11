#!/usr/bin/env python3

import sys
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
from sklearn.cluster import KMeans




# inital port
port = []

VOLTAGE_READ = 0
CURRENT_READ = 0
VELOCITY_READ = 0
SOC_VALUE = 0
ANG_READ = 0


Tcount = 0
zeroTimeout = 1
RPM = 0
lastSpeed = 0
Speed = 0




# GPIO interrupt setup ----------------------------------------------------------------------------------------------
read_pin = 26
GPIO.setmode(GPIO.BCM)
GPIO.setup(read_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
interrupt_pin = 16

lastCountTime = timeDelay.time()
newCountTime = lastCountTime

def pulse_event(channel):
    global newCountTime
    global Speed
    global count
    global Tcount
    
    newCountTime = timeDelay.time()
    Tcount = Tcount +1

GPIO.setup(interrupt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(interrupt_pin, GPIO.FALLING, callback=pulse_event, bouncetime=100)
# ---------------------------------------------------------------------------------------------- GPIO interrupt setup


# ----------------------------------------------------------------------
def GPIO_read():
    global newCountTime
    global count
    global Tcount


    GPIO.setmode( GPIO.BCM)
    GPIO.setup( 16, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    while(True):
        if GPIO.input(16):
            Tcount = Tcount + 1
            newCountTime = timeDelay.time()
        else:
            pass
# ---------------------------------------------------------------------





# Find Arduino port ------------------------------------------------------------------------------------------------
def find_port():
    global port
    find = 0
    print('Finding port for Serial')
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
# ------------------------------------------------------------------------------------------------ Find Arduino port




# GyroScope setup functions -----------------------------------------------------------------------------------------
# power_mgmt_1 = 0x6b
# power_mgmt_2 = 0x6c
 
# def read_byte(reg):
#     return bus.read_byte_data(address, reg)
 
# def read_word(reg):
#     h = bus.read_byte_data(address, reg)
#     l = bus.read_byte_data(address, reg+1)
#     value = (h << 8) + l
#     return value
 
# def read_word_2c(reg):
#     val = read_word(reg)
#     if (val >= 0x8000):
#         return -((65535 - val) + 1)
#     else:
#         return val
 
# def dist(a,b):
#     return math.sqrt((a*a)+(b*b))
 
# def get_y_rotation(x,y,z):
#     radians = math.atan2(x, dist(y,z))
#     return -math.degrees(radians)
 
# def get_x_rotation(x,y,z):
#     radians = math.atan2(y, dist(x,z))
#     return math.degrees(radians)
 
# bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
# address = 0x68       # via i2cdetect
# # sudo i2cdetect -y 1
# # sudo i2cget -y 1 0x68 0x75
 
# # Aktivieren, um das Modul ansprechen zu koennen
# bus.write_byte_data(address, power_mgmt_1, 0)

# def GyroRead():
#     global ANG_READ
    
#     print("Gyro setting up")
    
#     try:
#         print("..................Gyro Connect................")
#         while True:
#             gyroskop_xout = read_word_2c(0x43)
#             gyroskop_yout = read_word_2c(0x45)
#             gyroskop_zout = read_word_2c(0x47)
             
#             beschleunigung_xout = read_word_2c(0x3b)
#             beschleunigung_yout = read_word_2c(0x3d)
#             beschleunigung_zout = read_word_2c(0x3f)
             
#             beschleunigung_xout_skaliert = beschleunigung_xout / 16384.0
#             beschleunigung_yout_skaliert = beschleunigung_yout / 16384.0
#             beschleunigung_zout_skaliert = beschleunigung_zout / 16384.0
            
#             #ANG_READ = get_x_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, beschleunigung_zout_skaliert)
#             ANG_READ = get_y_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, beschleunigung_zout_skaliert)
#             #print "X Rotation: " , get_x_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, beschleunigung_zout_skaliert)
#             #print "Y Rotation: " , get_y_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, beschleunigung_zout_skaliert)
#             #print("---------------------------------------------------------------------")
#             #publish.single( "93/bms/ang", round(ANG_READ), hostname = "test.mosquitto.org")
#             timeDelay.sleep(0.5)
#     except:
#         print("..................Gyro Disconnect................")
#         GyroRead()
# ----------------------------------------------------------------------------------------- GyroScope setup functions




# Speed Sensor read --------------------------------------------------------------------------------------------------
def SpeedRead():
    global newCountTime
    global count
    global Tcount
    global Speed
    global lastSpeed
    global lastCountTime
    global VELOCITY_READ
    
    while True:
        CurrentTime = timeDelay.time()
    
        if ( (newCountTime - lastCountTime) > 0.1):
            #print(newCountTime - lastCountTime)
            RPM = 60.00/(newCountTime - lastCountTime)
            Speed = RPM*1.45*60*0.001
            lastCountTime = newCountTime
            if ( (Speed - lastSpeed) > 30 ):
                print("........................Noise read.....................")
                Speed = lastSpeed
        elif ( (CurrentTime - lastCountTime) > zeroTimeout ):
            RPM = 0.0
            Speed = 0.0
        
        if (Tcount <= 1):
            Speed = 0.0
            RPM = 0.0
        
        VELOCITY_READ = Speed
        #print("RPM : {x:0.2f}".format(x=RPM),"Speed : {x:0.2f}".format(x=Speed) , "Tcount :", Tcount)
        timeDelay.sleep(0.1)
        lastSpeed = Speed
# ------------------------------------------------------------------------------------------------- Speed Sensor read
    



# Speed, Distace, Gyro --------------------------------------------------------------------------------------------- 
def WriteData():
    global Speed
    global Tcount
    
    while True:
        drive = Tcount * 1.45 * 0.001
        publish.single("93/read/velo",round(Speed),hostname = "localhost")
        publish.single("93/read/distance",round(drive, 3),hostname = "localhost")
        # publish.single("93/bms/ang", round(ANG_READ), hostname = "localhost")
        timeDelay.sleep(1)
# --------------------------------------------------------------------------------------------- Speed, Distace, Gyro 
              



# Arduino data Read ------------------------------------------------------------------------------------------------
class ArduinoRead:
    def __int__( self):
        self.Dcount = 0
        
    def dataRead( self):
        global VOLTAGE_READ
        global CURRENT_READ
        global TEMP_READ
        global ANG_READ
        global DISTANCE
        
        print("Strating read data")
        try:
            while True:
                if ser.in_waiting > 0:
                    data = ser.readline()
                    timeDelay.sleep(0.1)
                    if ( data != b'' and data != b'\r\n'):
                        #print("Recived :", str(data))
                        value = str(data).split(";")
                        #print("")
                        print( "Recived data V:", value[1], "; I:", value[2])
        
                        VOLTAGE_READ = float(value[1])
                        CURRENT_READ = float(value[2])

                        publish.single("93/bms/current",CURRENT_READ,hostname = "localhost")
                        publish.single("93/bms/vol",VOLTAGE_READ,hostname = "localhost")
        except:
            print("------------------------ Arduino read error ---------------------")
            timeDelay.sleep(1)
            self.dataRead()
# ------------------------------------------------------------------------------------------------- Arduino data Read
            
            


# Kmeans Read -----------------------------------------------------------------------------------------------------
class D_Kmean:    
    def __init__( self):
        self.V0 = 0
        self.I0 = 0
        self.Speed = 0
        self.angle = 0
        self.SOC = 0
        self.Capacity = 5306400 # 22x67x3600
        self.Power = 0
        self.P_avg = 0
        self.S_avg = 0
        self.A_avg = 0
        self.P_list = []
        self.S_list = []
        self.A_list = []
        self.count = -1
        self.samples = 1                        # refresh samples start
        self.sTime = 5                          # mean samples
        self.text = "hello from thred1 "
        self.cluster_numbers = 3
        

    def run( self):
        global VOLTAGE_READ
        global CURRENT_READ
        global VELOCITY_READ
        global ANG_READ
        global DISTANCE
        global SOC_VALUE
        
        velocity_input = 'S_file.txt'
        power_input = 'P_file.txt'
        angle_input = 'A_file.txt'
        
        with open(velocity_input) as f1:
            Vvalues = f1.readlines()
            print("vel read")
        f1.close()
        velocity = [ eval(x) for x in Vvalues]
        
        with open(power_input) as f2:
            Pvalues = f2.readlines()
            print("pow read")
        f2.close()
        power = [ eval(x) for x in Pvalues]

        # with open(angle_input) as f3:
        #     Avalues = f3.readlines()
        #     print("ang read")
        # f3.close()
        # angle = [ eval(x) for x in Avalues]

        data = list(zip(velocity, power))
        kmeans = KMeans( n_clusters= self.cluster_numbers, random_state=0, n_init="auto").fit(data)

        print( kmeans.cluster_centers_)
        clusters = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for C_i in range( self.cluster_numbers):
            if ( float(kmeans.cluster_centers_[C_i][0]) == 0.0):
                clusters[C_i] = 0
            else:
                clusters[C_i] = float(kmeans.cluster_centers_[C_i][1]) / float(kmeans.cluster_centers_[C_i][0])
           
        
        while True:
            #print("counting", self.samples)
            self.V0 = VOLTAGE_READ
            self.I0 = CURRENT_READ
            self.angle = ANG_READ
            self.Speed = VELOCITY_READ / 3.6
            self.SOC = SOC_VALUE
            self.count = self.count + 1
            
            self.Power = self.V0 * self.I0
            #print(self.Power)
            
            self.P_list.append( self.Power)
            self.S_list.append( self.Speed)
            
    
            #print( "voltage:", self.V0 ," current:", self.I0, "speed:", self.Speed, "SOC:", self.SOC)
            
            if (self.count == self.sTime -1):
                P_sum = 0
                S_sum = 0
                # A_sum = 0
                
                for i in range(self.sTime):
                    P_sum = P_sum + self.P_list[i]
                    S_sum = S_sum + self.S_list[i]
                    # A_sum = A_sum + self.A_list[i]
    
                self.P_avg = P_sum/self.sTime
                self.S_avg = S_sum/self.sTime
                # self.A_avg = A_sum/self.sTime

                S_data_file = open('S_file.txt', 'a')
                P_data_file = open('P_file.txt', 'a')
                # A_data_file = open('A_file.txt', 'a')
                
                #if ( self.S_avg > 0 and self.P_avg > 0):
                line = "%f\n" % (self.S_avg)
                S_data_file.write(line)
                line = "%f\n" % (self.P_avg)
                P_data_file.write(line)
                    # line = "%f\n" % (self.A_avg)
                    # A_data_file.write(line)

                S_data_file.close()
                P_data_file.close()
                
                a = kmeans.predict( [[self.S_avg, self.P_avg]])

                for C_i in range( self.cluster_numbers):
                    if ( a[0] == C_i ):
                        map_value = clusters[C_i]
                    
                # print("map_value", map_value)

                if ( map_value == 0.0) :
                    distance = 0.0
                else :
                    distance = self.SOC*self.Capacity / (map_value*100)

                if ( self.S_avg == 0.0) :
                    distance = 0.0
                DISTANCE = distance * 0.001
                publish.single("93/read/drive",round( distance*0.001, 2),hostname = "localhost")
                #print("distance can go:", DISTANCE)

                D_data_file = open('D_file.txt', 'a')
                line = "%f\n" % (DISTANCE)
                D_data_file.write(line)
                D_data_file.close()
                
                del self.P_list[:]
                del self.S_list[:]
                self.count = -1

            if ( self.samples >= 100):
                self.samples = 1
                self.run()
            self.samples = self.samples + 1
                
            timeDelay.sleep(1)
# ---------------------------------------------------------------------------------------------------- Kmeans Read
            
            


# SOC, SOH Read --------------------------------------------------------------------------------------------------
class BatteryRead:
    def __init__( self):
        #initial values assigned
        self.I0 = 0 #Input current
        self.soh = 0.8 
        self.soc = 1 #Assumed full charged

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
                
            #publish.single("93/bms/soc",round(self.soc*100, 2),hostname = "localhost")
            #print("SOC - PUBLISHED")
            publish.single("93/bms/soh",round(self.soh*100, 2),hostname = "localhost")
            #print("SOH - PUBLISHED")
            #print("Voltage:", V0, "Current:", I0,"SOC:", round(self.soc*100, 2), "; SOH:", round(self.soh*100, 2))
            
            Data_file = open('BMS_file.txt', 'w')
            line = "%f\n" % (SOC_VALUE)
            Data_file.write(line)
            line = "%f\n" % (self.soh*100)
            Data_file.write(line)
            Data_file.close()
 
            
            timeDelay.sleep(1)
# -------------------------------------------------------------------------------------------------- SOC, SOH Read
                
                


if __name__ == '__main__':
    find_port()

    print(port)
    try:
        port_name = "/dev/" + port[0]
        # print(port_name)
        port_name = "/dev/ttyUSB0"
        ser = serial.Serial( port_name, 9600, timeout=1)
        ser.reset_input_buffer()
    except:
        print("---------------------------- USB port error ---------------------------------")
    
    t1 = Thread( target = SpeedRead)
    t2 = Thread( target = WriteData)
    # t3 = Thread( target = GyroRead)
    
    
    V_I_read = ArduinoRead()
    V_I_readTread = Thread( target = V_I_read.dataRead)
    V_I_readTread.start()
    
    
    SOC_SOH_write = BatteryRead()
    SOC_SOH_writeTread = Thread( target = SOC_SOH_write.run)
    SOC_SOH_writeTread.start()
    

    D_Write = D_Kmean()
    D_WriteTread = Thread( target = D_Write.run)
    D_WriteTread.start()
    
    t1.start()
    t2.start()
    #t3.start()
    
    

    
