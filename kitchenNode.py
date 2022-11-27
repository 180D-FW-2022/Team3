import paho.mqtt.client as mqtt
from collections import deque
import keyboard
import mqttTopics

class KitchenNode:
  orderList = deque()
  MQTTbroker = mqttTopics.broker
  orTakTopic = mqttTopics.orTakTopic
  WOKerTopic = mqttTopics.WOKerTopic
  client = mqtt.Client()
  pending = False
  WOKerReady = False

  def __init__(self):
    pass

  def __on_connect(self, client, userdata, flags, rc):
      print("Connection initialized. Returned result: " + str(rc))
      client.subscribe(self.orTakTopic, qos=1) 
      pass
      
    # The callback of the client when it disconnects. 
  def __on_disconnect(self, client, userdata, rc): 
    if rc != 0: 
      print('Unexpected Disconnect')
    else:
      print('Expected Disconnect')

  # The default message callback. 
  # (you can create separate callbacks per subscribed topic)
  def __orderReceive(self, client, userdata, message): 
    rawString = message.payload
    # format: TN:1;Items:Ham-1,Fries-2,CM-3;Tot:9;Cost:63.76;SR:I am lactose intolerant
    splitString = rawString.decode() #convert bytes into string
    splitString = splitString.split(";") #split string by commas
    tableNumber = splitString[0][3:]
    itemsArray = []
    for i in splitString[1].split(":")[1].split(","):
      itemName = i.split("-")[0]
      qnty = i.split("-")[1]
      itemsArray.append((itemName, qnty))
    total = splitString[2][4:] 
    cost = splitString[3][5:]
    specialRequests = splitString[4][3:]
    completeOrder = {
      "tableNumber": tableNumber,
      "itemsArray": itemsArray,
      "totalItems": total,
      "cost": cost,
      "specialRequests":specialRequests
    }
    self.orderList.append(completeOrder)
    print("---")
    print(self.orderList)
    pass

  def __orderComplete(self, event=None):
    print("orderComplete")
    self.pending = True
      
  def __WOKerReadyState(self, client, userdata, message):
      self.WOKerready = message

  def __orderSend(self):
    
    if self.pending == True and self.WOKerReady == True and len(self.orderList) != 0:
      self.client.publish(mqttTopics.WOKerTableNumberTopic, self.orderList[0]["tableNumber"], qos=1)
      self.orderList.popleft()
      self.client.publish(mqttTopics.WOKerGoTopic, True, qos=1)
      self.pending = False
      self.WOKerReady = False
      print("order sent")
    pass

  def __WOKerTest(self, event=None):
    print("WOKer Ready")
    self.WOKerReady = True
  

  def main(self):
    # add additional client options (security, certifications, etc.)
    self.client.on_connect = self.__on_connect
    self.client.on_disconnect = self.__on_disconnect
    self.client.message_callback_add(mqttTopics.orTakTopic, self.__orderReceive)
    self.client.message_callback_add(mqttTopics.WOKerReadyTopic, self.__WOKerReadyState)
    
    # 2. connect to a broker using one of the connect*() functions. 
    self.client.connect_async(self.MQTTbroker)

    # 3. call one of the loop*() functions to maintain network traffic flow with the broker. 
    self.client.loop_start()
    # client.loop_forever()

    keyboard.on_release_key('p', self.__orderComplete)
    keyboard.on_release_key('t', self.__WOKerTest)

    while True:
      self.__orderSend()
      pass
    # use subscribe() to subscribe to a topic and receive messages. 

    # use publish() to publish messages to the broker. 

    # use disconnect() to disconnect from the broker. 
    self.client.loop_stop()
    self.client.disconnect()

    
mainNode = KitchenNode()
mainNode.main()