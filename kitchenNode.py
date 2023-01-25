import paho.mqtt.client as mqtt
from collections import deque
import keyboard
import mqttTopics
import firebase


class KitchenNode:
  orderList = deque()
  latestOrder = []
  client = mqtt.Client()
  orderPending = False
  WOKerReady = False
  readyTables = deque()

  def __init__(self):
    pass

  def __on_connect(self, client, userdata, flags, rc):
      print("Connection initialized. Returned result: " + str(rc))
      client.subscribe(mqttTopics.orTakTopic, qos=1) 
      client.subscribe(mqttTopics.WOKerReadyTopic, qos=1)
      pass
      
  def __on_disconnect(self, client, userdata, rc): 
    if rc != 0: 
      print('Unexpected Disconnect')
    else:
      print('Expected Disconnect')

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
      "specialRequests": specialRequests
    }
    self.orderList.append(completeOrder)
    # print("---")
    # for order in self.orderList:
        # print(order)
    # print("Order queue length: ", len(self.orderList))

  def orderComplete(self):
    self.orderPending = True
    print("Table", self.readyTables[0], "pending")

      
  def __WOKerReadyState(self, client, userdata, message):
      self.WOKerready = message

  def orderSend(self, event=None):
    if self.orderPending == True and self.WOKerReady == True and len(self.orderList) != 0:
      self.client.publish(mqttTopics.WOKerTableNumberTopic, self.readyTables[0], qos=1)
      self.client.publish(mqttTopics.WOKerGoTopic, True, qos=1)
      print("WOKer going to table", self.readyTables[0])
      self.readyTables.popleft()
      self.orderList.popleft()
      if len(self.readyTables) == 0:
        self.orderPending = False
      self.WOKerReady = False
      self.client.publish(mqttTopics.WOKerGoTopic, False, qos=1)
    pass

  def __WOKerTest(self, event=None):
    print("WOKer set to ready!")
    self.WOKerReady = True
  

  def run(self):
    self.client.on_connect = self.__on_connect
    self.client.on_disconnect = self.__on_disconnect
    self.client.message_callback_add(mqttTopics.orTakTopic, self.__orderReceive)
    self.client.message_callback_add(mqttTopics.WOKerReadyTopic, self.__WOKerReadyState)
    
    self.client.connect_async(mqttTopics.broker)

    self.client.loop_start()

    # keyboard.on_release_key('p', self.__orderComplete) # Mark order as complete
    keyboard.on_release_key('t', self.__WOKerTest) # set WOKer as ready; only intended for testing

    # while True:
    #   self.__orderSend() #constantly check
    #   pass

  
# mainNode = KitchenNode()
# mainNode.run()