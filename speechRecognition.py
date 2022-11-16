import speech_recognition as sr
import paho.mqtt.client as mqtt

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
    MQTTbroker = 'test.mosquitto.org'
    MQTTtopic = 'ece180d/orderTest'

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
        client.connect_async(self.MQTTbroker)
        # 3. call one of the loop*() functions to maintain network traffic flow with the broker.
        client.loop_start()
        # 4. use subscribe() to subscribe to a topic and receive messages.
        # 5. use publish() to publish messages to the broker.
        # payload must be a string, bytearray, int, float or None.
        client.publish(self.MQTTtopic, orderString, qos=1)
        client.loop_stop()
        print("Order sent!")

    def takeOrder(self): 
        self.__resetTable()
        print("What would you like to order?")
        while True:
            item = self.__speechToText()

            if item == "no":
                print("Any special requests?")
                specialRequestRaw = self.__speechToText()
                if specialRequestRaw != 'no':
                    self.specialRequests = specialRequestRaw
                break
            
            if item in list(self.menu.keys()):
                print("How many?")
                qty = self.__speechToText()
                self.itemArray.append((item, int(qty)))
                self.itemCount = self.itemCount + int(qty)
                self.cost = self.cost + int(qty) * self.menu[item]
                pass
            else:  
                print("Item not found in menu; try again")
                continue

            print("Would you like to order anything else?")
        
        self.__sendOrder()

        
    def testOrder(self, itemArray):
        self.itemArray = itemArray
        self.itemCount = len(itemArray)
        self.cost = 9999
        self.__sendOrder()
