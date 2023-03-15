import speech_recognition as sr
import firebase
import LCDdisplay as lcd
screen = lcd.LCDdisplay()

class OrTak:
    #menu dictionary with items and prices
    menu = {
        "chicken sandwich": 10,
        "fries": 5,
        "milkshake": 2.5,
        "fruit": 3,
        "burger": 8,
        "cheeseburger": 8.50,
        "potatoes": 999,
        "marcell's potatoes": 1500
    }
    tableNumber = -99
    itemArray = []
    itemCount = 0
    specialRequests = 'N/A'
    cost = 0
    negativeResponses = ["no", "nah", "nope", "i'm good", "no thanks", "absolutely not"]
    positiveResponses = ["yes", "sure", "yeah", "yep", "yuppers", "yipee", "yes please", "absolutely", "you bet", "roger that", "certainly"]

    def __init__(self, tableNumberArg):
        self.tableNumber = tableNumberArg
        pass

    def __resetTable(self):
        self.itemArray = []
        self.itemCount = 0
        self.specialRequests = 'N/A'
        self.cost = 0

    def __speechToText(self, type=None):
        print(type)
        r = sr.Recognizer()
        text=""
        with sr.Microphone() as source:
            while True:
                try:
                    # r.adjust_for_ambient_noise(source)
                    r.energy_threshold = 800
                    r.adjust_for_ambient_noise(source, 2) 
                    print("Energy:",r.energy_threshold)
                    print("Listening...")
                    screen.listening()
                    audio = r.listen(source, timeout=5, phrase_time_limit=5)
                    screen.processing()
                    if type == None:
                        text = r.recognize_google(audio, show_all=False, key=None, language="en-US")
                    elif type == "cloud":
                        text = r.recognize_google_cloud(audio, show_all=False, credentials_json="credentials_google_speech.json", language="en-US", preferred_phrases=["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "marcell's potatoes"])
                        text = text.strip()
                        print("cloud")
                    numbers = ["one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten"]
                    if text in numbers:
                        text.replace(numbers[numbers.index(text)], str(numbers.index(text) + 1))
                    break
                except Exception as e:
                    print(e)
                    print("exception")
                    return ""
                    screen.tryAgain()
                # except sr.UnknownValueError:
                #     print(sr.UnknownValueError)
                #     print("Try again; Google Speech Recognition could not understand audio")
                # except sr.RequestError as e:
                #     print("Try again; Could not request results from Google Speech Recognition service; {0}".format(e))
                # except sr.WaitTimeoutError:
                    # print("Timeout")
                    pass
        print("transcribed:", (text.lower()).strip())
        return (text.lower()).strip()
    
    def __sendOrder(self):
        # Example string; TN:1;Items:Ham-1,Fries-2,CM-3;Tot:9;Cost:63.76;SR:I am lactose intolerant
        itemString = ''
        for item in self.itemArray:
            itemString += item[0].lower() + '-' + str(item[1]) + ','
        itemString = itemString[:-1]

        orderNumber = str(int(firebase.ref.child("currentOrder").get()) + 1)

        itemDict = {}

        for item in self.itemArray:
            itemDict.update({item[0]: item[1]})
            pass

        orderDict = {
            "tableNumber": self.tableNumber,
            "cost": self.cost,
            "specialRequests": self.specialRequests,
            "itemCount": self.itemCount,
            "items": itemDict,
            "orderNumber": int(orderNumber)
        }

        print(orderDict)


        firebase.ref.child("sentOrders/order" + str(orderNumber)).set(orderDict)
        firebase.ref.child("currentOrder").set(orderNumber)


    # def takeOrder(self):
    #     while True:
    #         screen.clear()
    #         self.__resetTable()
    #         while True:
    #             print("say ready")
    #             screen.displayString("Say \"Ready\"!")
    #             wakeWord = self.__speechToText(type=None)
    #             # print("wake:", wakeWord)
    #             # print("len", len(wakeWord), len("ready"))
    #             # print(wakeWord != "ready")
    #             if wakeWord != "ready":
    #                 screen.tryAgain()
    #                 continue
    #             while True:
    #                 screen.displayString("What is", "your order?")
    #                 order = self.__speechToText()
    #                 order.replace("to", "2")
    #                 order.replace("for", "4")
    #                 numbers = ["one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten"]
    #                 print("order:", order)
    #                 for i in (numbers):
    #                     order.replace(i, str(numbers.index(i) + 1))
    #                 for i in self.menu.keys():
    #                     print("item:", i)
    #                     numberStr = ""
    #                     if i in order:
    #                         print("item:", i)
    #                         item = i
    #                         nums = order.split(item)[0]
    #                         order = order.split(item)[1]
    #                         for i in nums:
    #                             if i.isnumeric():
    #                                 numberStr +=i
    #                         qty = int(numberStr)
    #                         print("qty:",qty)
    #                         break
    #                 screen.tryAgain()
                    

    # def takeOrder(self):
    #     while True:
    #         screen.clear()
    #         # screen.displayString("Say \"Ready\"!")
    #         self.__resetTable()
    #         while True:
    #             screen.displayString("Say \"Ready\"!")
    #             wakeWord = self.__speechToText(type=None)
    #             # print("wake:", wakeWord)
    #             # print("len", len(wakeWord), len("ready"))
    #             # print(wakeWord != "ready")
    #             if wakeWord != "ready":
    #                 screen.tryAgain()
    #                 continue
    #             while True:
    #                 print("What would you like to order?")
    #                 screen.displayString("What item would", "you like?")
    #                 order = self.__speechToText(type="cloud")

    #                 found = False
    #                 for i in self.menu.keys():
    #                     print("item:", i)
    #                     numberStr = ""
    #                     if i in order:
    #                         print("item:", i)
    #                         item = i
    #                         nums = order.split(item)[0]
    #                         for i in nums:
    #                             if i.isnumeric():
    #                                 numberStr +=i
    #                         qty = int(numberStr)
    #                         print("qty:",qty)
    #                         found = True 
    #                         self.itemArray.append((item, qty))
    #                         self.itemCount += qty
    #                         self.cost += qty * self.menu[item]
    #                         break
    #                 if found == False:
    #                     screen.tryAgain()
    #                     continue
    #                 orderMoreItems = False
    #                 while True:
    #                     print("Would you like to order anything else?")
    #                     screen.displayString("Any more items", "to order?")
    #                     print("display")
    #                     orderMore = self.__speechToText()
    #                     if any(word in orderMore for word in self.positiveResponses):
    #                         screen.displayString(orderMore.capitalize())
    #                         orderMoreItems = True
    #                         break
    #                     elif any(word in orderMore for word in self.negativeResponses):
    #                         screen.displayString(orderMore.capitalize())
    #                         while True:
    #                             print("Any special requests?")
    #                             screen.displayString("Any special", "requests?")
    #                             specialRequestRaw = self.__speechToText()
    #                             print(specialRequestRaw)
    #                             if specialRequestRaw == "":
    #                                 screen.tryAgain()
    #                                 continue
    #                             if specialRequestRaw not in self.negativeResponses:
    #                                 self.specialRequests = specialRequestRaw
    #                             screen.displayString(self.specialRequests)
    #                             self.__sendOrder()
    #                             screen.displayString("Order sent!")
    #                             break
    #                         break
    #                     else:
    #                         screen.tryAgain()
    #                         print("try again")
    #                         pass
    #                 if orderMoreItems == True:
    #                     continue
    #                 else:
    #                     break
    #             break

    def takeOrder(self):
        while True:
            screen.clear()
            # screen.displayString("Say \"Ready\"!")
            self.__resetTable()
            while True:
                screen.displayString("Say \"Ready\"!")
                wakeWord = self.__speechToText()
                # print("wake:", wakeWord)
                # print("len", len(wakeWord), len("ready"))
                # print(wakeWord != "ready")
                if wakeWord != "ready":
                    screen.tryAgain()
                    continue
                while True:
                    print("What would you like to order?")
                    screen.displayString("What item would", "you like?")
                    item = self.__speechToText(type="cloud")
                    print(item)
                    if item in list(self.menu.keys()):
                        screen.displayString(item.capitalize())
                        while True:
                            print("How many?")
                            screen.displayString("How many?")
                            qty = self.__speechToText(type="cloud")
                            print("qty:", qty)
                            if qty.isnumeric():
                                break
                            print("Please repeat yourself!")
                            screen.tryAgain()
                        self.itemArray.append((item, int(qty)))
                        self.itemCount += int(qty)
                        self.cost += int(qty) * self.menu[item]
                        screen.displayString(qty + " item(s)")
                        pass
                    else:
                        print("Item not found in menu; try again")
                        screen.displayString("Item not found","Try again!")
                        continue

                    print("Would you like to order anything else?")
                    orderMoreItems = False
                    while True:
                        screen.displayString("Any more items", "to order?")
                        print("display")
                        orderMore = self.__speechToText(type="cloud")
                        if any(word in orderMore for word in self.positiveResponses):
                            screen.displayString(orderMore.capitalize())
                            orderMoreItems = True
                            break
                        elif any(word in orderMore for word in self.negativeResponses):
                            screen.displayString(orderMore.capitalize())
                            while True:
                                print("Any special requests?")
                                screen.displayString("Any special", "requests?")
                                specialRequestRaw = self.__speechToText()
                                print(specialRequestRaw)
                                if specialRequestRaw == "":
                                    screen.tryAgain()
                                    continue
                                if specialRequestRaw not in self.negativeResponses:
                                    self.specialRequests = specialRequestRaw
                                screen.displayString(self.specialRequests)
                                self.__sendOrder()
                                screen.displayString("Order sent!")
                                break
                            break
                        else:
                            screen.tryAgain()
                            print("try again")
                            pass
                    if orderMoreItems == True:
                        continue
                    else:
                        break
                break

        
    def testOrder(self, itemArray):
        self.itemArray = itemArray
        self.cost = 9999
        self.__sendOrder()
