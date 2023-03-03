/*******************************************************************************
 * Copyright (c) 2012, 2022 IBM Corp., Ian Craggs
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * and Eclipse Distribution License v1.0 which accompany this distribution. 
 *
 * The Eclipse Public License is available at 
 *   https://www.eclipse.org/legal/epl-2.0/
 * and the Eclipse Distribution License is available at 
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial contribution
 *******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "MQTTClient.h"

#define ADDRESS     "test.mosquitto.org:1883"
#define CLIENTID    "ExampleClient"
#define TOPIC1      "93/bms/vel"
#define TOPIC2      "93/bms/spd"
#define QOS         1
#define TIMEOUT     10000L

int Sub();
int Pub();

int state = 0;
int count = 100;

volatile MQTTClient_deliveryToken deliveredtoken;

void delivered(void *context, MQTTClient_deliveryToken dt)
{
    printf("Message with token value %d delivery confirmed\n", dt);
    deliveredtoken = dt;
}

int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
    printf("Message arrived from: %s, message: %.*s\n", topicName, message->payloadlen, (char*)message->payload);
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    state = state +  1;
    return 1;
}

void connlost(void *context, char *cause)
{
    printf("\nConnection lost\n");
    
    state = state - 1;
    
}

int Sub() {
	
    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    
    int rc;
    
    rc = MQTTClient_create(&client, ADDRESS, CLIENTID,MQTTCLIENT_PERSISTENCE_NONE, NULL);
    
    rc = MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, delivered);
    
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    
    rc = MQTTClient_connect(client, &conn_opts);
    
    printf("\n\nNew connection topic %s for client %s using QoS %d \n\n", TOPIC1, CLIENTID, QOS);
           

    rc = MQTTClient_subscribe(client, TOPIC1, QOS);
    while (1) {
		if (state == 4) {
			break;
		}
	}
	
    rc = MQTTClient_unsubscribe(client, TOPIC1);
    state = 0;
	
		
}


int Pub() {
    
    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
    int rc2;

    MQTTClient_create(&client, ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    
    char PAYLOAD[10] = "50";
    
    
	
    sprintf( PAYLOAD , "%d" , count);
        
        
    if ((rc2 = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
            printf("Failed to connect, return code %d\n", rc2);
        }
        pubmsg.payload = PAYLOAD;
        pubmsg.payloadlen = strlen(PAYLOAD);
        pubmsg.qos = QOS;
        pubmsg.retained = 0;
        MQTTClient_publishMessage(client, TOPIC2, &pubmsg, &token);
        
        printf("\nPublication of: %s on topic: %s for client with ClientID: %s\n", PAYLOAD, TOPIC2, CLIENTID);
            
        rc2 = MQTTClient_waitForCompletion(client, token, TIMEOUT);
        printf("Message with delivery token %d delivered\n", token);
        
        count = count -1;
        if (count == 0) {
            count = 100;
        }
            
        MQTTClient_disconnect(client, 10000);
        //MQTTClient_destroy(&client);
        printf("\n");
	
	MQTTClient_destroy(&client);
    
}


int main(int argc, char* argv[])
{
	
	while (1) {
	    Pub();
	    Sub();
	}
	
	
}
