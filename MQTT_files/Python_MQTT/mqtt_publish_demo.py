#MQTT publish demo
#publish two messages to 2 different topics

import paho.mqtt.publish as publish

publish.single("eeuop/SS/status/H","Test1",hostname = "test.mosquitto.org")
publish.single("eeuop/SS/status/W","Test2",hostname = "test.mosquitto.org")
print("Done")
