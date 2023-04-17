import numpy as np
from numpy.linalg import inv
import csv
import paho.mqtt.publish as publish
import serial
import time as timeDelay
import serial.tools.list_ports
import re

#inital port
port = []

#initial values assigned
I0 = 0 #Input current
soh = 0.8 
soc = 1 #Assumed full charged

time = []
SOH = []
SOC = []
Vm = []
Vp = []
Im = []
  
sampleTime =0.2

# 3 states declaration
soc1 = soc
soh1 = soh
Vth1 = 0

#Process Error Co-variance Matrix (Q) E[XX^T]
Qk = np.array([[0.05 , 0 , 0],[0, 0.04, 0],[ 0, 0, 1]])

#print('Qk',Qk,Qk.shape)

#Initial Process Error Co-variance Matrix (Pkint = Qkint)
Pkint = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
#print('\n Pkint',Pkint,Pkint.shape)
Pkin = Pkint

#Measurement Error Co-variance Matrix (R)
R = np.array([0.5]) #(1,)



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



def statediff (x,u):
  #separate terms from the state vector and input vector
  soh = x[0]
  soc = x[1]
  vc = x[2]
  current = float(u)
  #print('\n current Sttediff ', current, type(current))
  #Get state-dependent parameters. Rth & Cth
  r2= 2.5968*(soc**4)-3.5211*(soc**3)+ 0*(soc**2)+1.4757*(soc)-0.3317 #as per the matlab code
  #r2 = 10.6048 * soc **4 - 15.1475 * soc ** 3 + 0 * soc ** 2 + 7.1826 * soc - 2.0363 # Rth float
  c2=200000
  #c2 = 90000 #constant
  # Capacity changes with soh
  
  #~~~calculating capacity~~~
  CAPACITY_NEW =20;
  CAPACITY_OLD_PERCENT =80
  
  alpha =  CAPACITY_OLD_PERCENT /100
  beta = 1 - alpha
  capacity = CAPACITY_NEW * (alpha + beta*soh)
  
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




if __name__ == '__main__':
    find_port()

    print(port)

    port_name = "/dev/" + port[0]
    print(port_name)
    ser = serial.Serial( port_name, 9600, timeout=1)
    ser.reset_input_buffer()

    while True:
        if ser.in_waiting > 0:
            data = ser.readline()
            timeDelay.sleep(0.1)
            if ( data != b'' and data != b'\r\n'):
                print("Recived :", str(data))
                
                value = str(data).split(";")
                print( "Voltage:", value[0], "; Current:", value[1], "; Temp:", value[2], "; Ang:", value[3], "; Round:", value[4])
    
                V0 = float(value[0])
                I0 = float(value[1])
                
                #V0 = 25.000
                #I0 = 3.000
                
                
                r2 = 2.5968*(soc**4)-3.5211*(soc**3)+ 0*(soc**2)+1.4757*(soc)-0.3317  # Rth float
                Vth = float(I0) * r2
                Vth1 = Vth

                 # ~~~~ Neglecting errors ~~~~ 
                r2y = 2.5968*(soc**4)-3.5211*(soc**3)+ 0*(soc**2)+1.4757*(soc)-0.3317 # Rth float

                '''
                print('\n soc1',soc1)
                print('\n soh1',soh1)
                print('\n Vth1',Vth1)
                '''
                #Vth calculation
                Vth1 = float(I0) * r2y

                y = np.array([[soh1],[soc1],[Vth1]]) # (3,1)
                #print('\n y ', y, y.shape)

                yPredict = y + (statediff(y,float(I0))*0.2) #considering no errors writing coulomb counting (3,1)
                #print('\n yPredict ',yPredict, yPredict.shape,type(yPredict))

                soh1 = yPredict[0,0]
                soc1 = yPredict[1,0]
                Vth1 = yPredict[2,0]
                #print('\n yPredict[0] yPredict[1] yPredict[2] typeSOH ',yPredict[0],yPredict[1],yPredict[2], type(yPredict[0]), type(soh1))

                Voc1=9.8958*(soc1)**4-19.3750*(soc1)**3+15.1042*(soc1)**2-3.9250*(soc1)+24.06

                r01 = 0.0002
                voltage=float(V0)

                #~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                #current state of the battery
                x = np.array([[soh],[soc],[Vth]]) #(3,1)
                #print('\n x ', x, x.shape)

                #find the prediction of x
                xPredict = x + (statediff(x,float(I0))*sampleTime) #(3,1)
                #print('\n xPredict ',xPredict, xPredict.shape,type(xPredict))

                soh = xPredict[0,0]
                soc = xPredict[1,0]
                Vth = xPredict[2,0]
                #print("SOC 1:", soc , "; SOH :", soh)
                #print('\n xPredict[0] xPredict[1] xPredict[2] typeSOH ',xPredict[0],xPredict[1],xPredict[2], type(xPredict[0]), type(soh))


                #calculation of thevenin model parameters
                r0 = 0.0002
                c2 = 200000
                Voc=9.8958*(soc**4)-19.3750*(soc**3)+15.1042*(soc**2)-3.9250*(soc)+24.06

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
                Pk = np.dot(Fk,np.dot(Pkin,FkT))+Qk
                '''
                print('\n Pk ', Pk, Pk.shape, type(Pk))
                print('\n check ' ,np.dot(Fk,np.dot(Pkin,FkT)))
                print('\n kui ' ,Qk)  
                print('\n check2 ' ,np.dot(Fk,np.dot(Pkin,FkT))+Qk)
                '''
                #kalman gain
                Gk = np.dot(Pk,np.dot(HKT,inv(np.dot(HK,np.dot(Pk,HKT))+R)));
                #print('\n Gk ', Gk, Gk.shape, type(Gk)) #(3,1)

                xest = xPredict+ (Gk*(voltage-HKPredict)); #equation Dimension Issue corrected
                #print('\n xest ', xest, xest.shape, type(xest)) #(3,1)
                #print('\n np.dot(Gk,(voltage-HKPredict ', np.dot(Gk,(voltage-HKPredict))) #(3,1)


                PK = np.dot((np.identity(3)-(np.dot(Gk,HK))),Pk); #equation

                #next loop update
                soh = xest[0,0];
                soc = xest[1,0];
                Vth = xest[2,0];

                vest=Voc-Vth-float(I0)*r0; #not necessary seems
                Qk = Pkin
                Pkin = PK
                
                #time.append(float(line[1]))
                Vm.append(float(V0)) 
                Im.append(float(I0))
                SOH.append(soh)
                SOC.append(soc)
                Vp.append(HKPredict)
                
                publish.single("93/bms/v1/soc",round(soc*100, 2),hostname = "test.mosquitto.org")
                #print("SOC - PUBLISHED")
                publish.single("93/bms/v1/soh",round(soh*100, 2),hostname = "test.mosquitto.org")
                #print("SOH - PUBLISHED")
                print("SOC :", soc , "; SOH :", soh)
                print("\n")
                
                
            else:
                print("data not match")
                


