import orTak
from time import sleep
import random
import paho.mqtt.client as mqtt
import mqttTopics

class testNode:
    client = mqtt.Client()

    def __on_connect(self, client, userdata, flags, rc):
        print("Connection initialized. Returned result: " + str(rc))
        client.subscribe(mqttTopics.WOKerTableNumberTopic, qos=1) 
        client.subscribe(mqttTopics.WOKerGoTopic, qos=1)
        pass
        
    def __on_disconnect(self, client, userdata, rc): 
        if rc != 0: 
            print('Unexpected Disconnect')
        else:
            print('Expected Disconnect')

    def tableNumberReceive(self, client, userdata, message): 
        print("table number received:", message.payload)
        pass

    def WOKerGo(self, client, userdata, message):
        print("Ready? ", message.payload)

    def run(self):
        self.client.on_connect = self.__on_connect
        self.client.on_disconnect = self.__on_disconnect
        self.client.message_callback_add(mqttTopics.WOKerTableNumberTopic, self.tableNumberReceive)
        self.client.message_callback_add(mqttTopics.WOKerGoTopic, self.WOKerGo)
        
        self.client.connect_async(mqttTopics.broker)
        print("run")
        self.client.loop_start()
        while True:
            pass

testing = testNode()
testing.run()