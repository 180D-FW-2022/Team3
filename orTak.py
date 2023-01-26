import speech_recognition as sr
import paho.mqtt.client as mqtt
import mqttTopics
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import firebase

class OrTak:
    #menu dictionary with items and prices
    menu = {
        "chicken sandwich": 10,
        "fry's": 5,
        "milkshake": 2.5,
        "fruit": 3,
        "burger": 8,
        "cheeseburger": 8.50
    }
    tableNumber = -99
    itemArray = []
    itemCount = 0
    specialRequests = 'N/A'
    cost = 0

    def __init__(self, tableNumberArg):
        self.tableNumber = tableNumberArg 
        pass

    def __resetTable(self):
        self.itemArray = []
        self.itemCount = 0
        self.specialRequests = 'N/A'
        self.cost = 0

    def __speechToText(self):
        while True:
            r = sr.Recognizer()
            with sr.Microphone() as source:
                audio = r.listen(source)

            try:
                text = r.recognize_google(audio).lower()
                break
            except sr.UnknownValueError:
                print("Try again; Google Speech Recognition could not understand audio")
            except sr.RequestError as e:
                print("Try again; Could not request results from Google Speech Recognition service; {0}".format(e))
        return text
    
    def __sendOrder(self):
        # Example string; TN:1;Items:Ham-1,Fries-2,CM-3;Tot:9;Cost:63.76;SR:I am lactose intolerant
        itemString = ''
        for item in self.itemArray:
            itemString += item[0].lower() + '-' + str(item[1]) + ','
        itemString = itemString[:-1]

        nextOrder = str(int(firebase.ref.child("currentOrder").get()) + 1)

        itemDict = {}

        for item in self.itemArray:
            itemDict.update({item[0]: item[1]})
            pass

        orderDict = {
            "tableNumber": self.tableNumber,
            "cost": self.cost,
            "specialRequests": self.specialRequests,
            "itemCount": self.itemCount,
            "items": itemDict
        }

        # for item in self.itemArray:
        #     # print(type(item))
        #     currentOrderRef.child("items/"+item[0]).set(item[1])
        #     print(item[0])
        #     print(item[1])
        #     pass

        firebase.ref.child("orders/order" + str(nextOrder)).set(orderDict)
        # currentOrderRef.set(orderDict)
        firebase.ref.child("currentOrder").set(nextOrder)

        orderString = "TN:"+str(self.tableNumber)+";Items:"+itemString+";Tot:"+str(self.itemCount)+";Cost:"+str(self.cost)+";SR:"+self.specialRequests
        print(orderString)

        def on_connect(client, userdata, flags, rc):
            print("Connection returned result: "+str(rc))
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.

        # The callback of the client when it disconnects.
        def on_disconnect(client, userdata, rc):
            if rc != 0:
                print('Unexpected Disconnect')
            else:
                print('Expected Disconnect')
        # The default message callback.
        # (won’t be used if only publishing, but can still exist)
        def on_message(client, userdata, message):
            print("Received: " , orderString)
        # 1. create a client instance.
        client = mqtt.Client()
        # add additional client options (security, certifications, etc.)
        # many default options should be good to start off.
        # add callbacks to client.
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        client.on_message = on_message
        # 2. connect to a broker using one of the connect*() functions.
        client.connect_async(mqttTopics.broker)
        # 3. call one of the loop*() functions to maintain network traffic flow with the broker.
        client.loop_start()
        # 4. use subscribe() to subscribe to a topic and receive messages.
        # 5. use publish() to publish messages to the broker.
        # payload must be a string, bytearray, int, float or None.
        client.publish(mqttTopics.orTakTopic, orderString, qos=1)
        client.loop_stop()
        print("Order sent!")

    def takeOrder(self): 
        self.__resetTable()
        print("What would you like to order?")
        while True:
            item = self.__speechToText()
            print(item)
            if item == "no":
                print("Any special requests?")
                specialRequestRaw = self.__speechToText()
                print(specialRequestRaw)
                if specialRequestRaw != 'no':
                    self.specialRequests = specialRequestRaw
                break
            
            if item in list(self.menu.keys()):
                print("How many?")
                while True:
                    qty = self.__speechToText()
                    if qty.isnumeric():
                        break
                    print("Please repeat yourself!")
                print(qty)
                self.itemArray.append((item, int(qty)))
                self.itemCount += int(qty)
                self.cost += int(qty) * self.menu[item]
                pass
            else:  
                print("Item not found in menu; try again")
                continue

            print("Would you like to order anything else?")
        
        self.__sendOrder()

        
    def testOrder(self, itemArray):
        self.itemArray = itemArray
        for i in itemArray:
            self.itemCount += i[1]
        self.cost = 9999
        self.__sendOrder()
