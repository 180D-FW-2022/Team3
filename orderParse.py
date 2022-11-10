import paho.mqtt.client as mqtt

orderList = []
MQTTserver = "ece180d/orderTest"

# 0. define callbacks - functions that run when events happen. 
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
  print("Connection initialized. Returned result: "+str(rc))

  # Subscribing in on_connect() means that if we lose the connection and
  # reconnect then subscriptions will be renewed.
  client.subscribe(MQTTserver, qos=1) 
  pass
  
# The callback of the client when it disconnects. 
def on_disconnect(client, userdata, rc): 
  if rc != 0: 
    print('Unexpected Disconnect')
  else:
    print('Expected Disconnect')

# The default message callback. 
# (you can create separate callbacks per subscribed topic)
def on_message(client, userdata, message): 
  # print('Received message: "' + str(message.payload) + '" on topic "' + 
  #       message.topic + '" with QoS ' + str(message.qos))
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
  orderList.append(completeOrder)
  print(orderList)
  pass
  

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
# client.connect("mqtt.eclipse.org")

# 3. call one of the loop*() functions to maintain network traffic flow with the broker. 
client.loop_start()
# client.loop_forever()

while True: 
  pass
# use subscribe() to subscribe to a topic and receive messages. 

# use publish() to publish messages to the broker. 

# use disconnect() to disconnect from the broker. 
client.loop_stop()
client.disconnect()
