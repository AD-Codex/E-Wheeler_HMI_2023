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
#define CLIENTID    "ExampleClientSub"
#define TOPIC1       "93/bms/vel"
#define PAYLOAD     "Hello World!"
#define QOS         1
#define TIMEOUT     10000L

int Sub();

int state = 1;

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
    return 1;
}

void connlost(void *context, char *cause)
{
    printf("\nConnection lost\n");
    
    state = 0;
    
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
		if ( state == 0) {
			break;
		}
	}
	
	rc = MQTTClient_unsubscribe(client, TOPIC1);
	state = 1;
	
		
}

int main(int argc, char* argv[])
{
	
	while (1) {
		Sub();
	}
	
	
/**		
	MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    
    int rc;

    if ((rc = MQTTClient_create(&client, ADDRESS, CLIENTID,
        MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to create client, return code %d\n", rc);
        rc = EXIT_FAILURE;
        goto exit;
    }
    
    if ((rc = MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, delivered)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to set callbacks, return code %d\n", rc);
        rc = EXIT_FAILURE;
        goto destroy_exit;
    }

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    
    rc = MQTTClient_connect(client, &conn_opts);
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        rc = EXIT_FAILURE;
        goto destroy_exit;
    }

    printf("Subscribing to topic %s\nfor client %s using QoS%d\n\n"
           "Press Q<Enter> to quit\n\n", TOPIC, CLIENTID, QOS);
           

    if ((rc = MQTTClient_subscribe(client, TOPIC, QOS)) != MQTTCLIENT_SUCCESS)
    {
    	printf("Failed to subscribe, return code %d\n", rc);
    	rc = EXIT_FAILURE;
    }
    else
    {
    	int ch;
    	do
    	{
        	ch = getchar();
    	} while (ch!='Q' && ch != 'q');
		
        if ((rc = MQTTClient_unsubscribe(client, TOPIC)) != MQTTCLIENT_SUCCESS)
        {
        	printf("Failed to unsubscribe, return code %d\n", rc);
        	rc = EXIT_FAILURE;
        }
    }

    if ((rc = MQTTClient_disconnect(client, 10000)) != MQTTCLIENT_SUCCESS)
    {
    	printf("Failed to disconnect, return code %d\n", rc);
    	rc = EXIT_FAILURE;
    }
    
    
destroy_exit:
    MQTTClient_destroy(&client);
exit:
    return rc;
**/
	
}
