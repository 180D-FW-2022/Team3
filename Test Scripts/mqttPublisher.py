import paho.mqtt.client as mqtt
import numpy as np
import sys
# 0. define callbacks - functions that run when events happen.
# The callback for when the client receives a CONNACK response from the server.
# TN:1;Items:Ham-1,Fries-2,CM-3;Tot:9;Cost:63.76;SR:I am lactose intolerant

menu = {
    "Ham": 10,
    "Fries": 4,
    "CM": 3
}

tableNumber = (sys.argv[1])
specialRequest = sys.argv[3]
itemString = ""
itemQuantityArray = sys.argv[2].split(',')

for i in itemQuantityArray:
    itemString += (i+",")
itemString = itemString[:-1]

total = 0
cost = 0
justItem = ""
for i in itemQuantityArray:
    total += int(i[-1])
    justItem = i.partition("-")[0]
    cost += int(int(i[-1])*menu[justItem])

cost = str(cost)
total = str(total)

orderString = "TN:"+tableNumber+";Items:"+itemString+";Tot:"+total+";Cost:"+cost+";SR:"+specialRequest
print(orderString)

def on_connect(client, userdata, flags, rc):
    print("Connection returned result: "+str(rc))
# Subscribing in on_connect() means that if we lose the connection and
# reconnect then subscriptions will be renewed.
# client.subscribe("ece180d/test")
# The callback of the client when it disconnects.
def on_disconnect(client, userdata, rc):
    if rc != 0:
        print('Unexpected Disconnect')
    else:
        print('Expected Disconnect')
# The default message callback.
# (won’t be used if only publishing, but can still exist)
def on_message(client, userdata, message):
    #print('Received message: "' + str(message.payload) + '" on topic "' + message.topic + '" with QoS ' + str(message.qos))
    print(orderString)
# 1. create a client instance.
client = mqtt.Client()
# add additional client options (security, certifications, etc.)
# many default options should be good to start off.
# add callbacks to client.
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message
# 2. connect to a broker using one of the connect*() functions.
client.connect_async('test.mosquitto.org')
# 3. call one of the loop*() functions to maintain network traffic flow with the broker.
client.loop_start()
# 4. use subscribe() to subscribe to a topic and receive messages.
# 5. use publish() to publish messages to the broker.
# payload must be a string, bytearray, int, float or None.
#for i in range(10):
#while(1):
client.publish('ece180d/orderTest', orderString, qos=1)
#client.publish('ece180d/orderTest')
# 6. use disconnect() to disconnect from the broker.
client.loop_stop()
# client.disconnect()
