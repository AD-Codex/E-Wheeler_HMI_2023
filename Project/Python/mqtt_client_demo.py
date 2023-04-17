# MQTT Client demo
# Continuously monitor two different MQTT topics for data,
# check if the received data matches two predefined 'commands'

import paho.mqtt.client as mqtt
#to import mqtt client structure from the package paho-mqtt installed and called as mqtt

#the callback when the client receives a CONNACK response from the server
def on_connect(client, userdata, flag, rc):
    print("connected with result code" + str(rc))

    #subscribing in on_connect() - if we lose the connection and
    #reconnect then subscription will be renewed
    client.subscribe("sach/test")
    #TOPIC1
    client.subscribe("sach/topic")
    #TOPIC2
    #everytime reconnect to these topics if it loose theconnection

#the callback for when a PUBLISH message is received from the server
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

    if msg.payload -- "Hello":
        print("Received message #1. do something")
        #Do something CAN be turning on lights, fan etc. anything

    if msg.payload -- "World":
        print("Received message #2. do something else")



#create an MQTT client and attach our routines to it.
client = mqtt.Client()
#creating the object client
client.on_connect = on_connect
client.on_message = on_message

client.connect("test.mosquitto.org",1883,60)
#????????????

#process network traffic and dispatch callbacks. This will also handle
#reconnecting. there are other loop*() functions made available. check
#the documentation at http://github.com/eclipse/paho.mqtt.python

client.loop_forever()
#another builtin function ??????????????
#once it is connected it will leave forever
