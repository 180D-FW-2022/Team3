import speech_recognition as sr
import firebase
import LCDdisplay as lcd
screen = lcd.LCDdisplay()

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
    negativeResponses = ["no", "nah", "nope", "i'm good", "no thanks", "absolutely not"]

    def __init__(self, tableNumberArg):
        self.tableNumber = tableNumberArg
        pass

    def __resetTable(self):
        self.itemArray = []
        self.itemCount = 0
        self.specialRequests = 'N/A'
        self.cost = 0

    def __speechToText(self):
        r = sr.Recognizer()
        text=""
        with sr.Microphone() as source:
            while True:
                try:
                    # r.adjust_for_ambient_noise(source)
                    r.energy_threshold = 400
                    print("Energy:",r.energy_threshold)
                    print("Listening...")
                    audio = r.listen(source, timeout=5)
                    text = r.recognize_google(audio, show_all=False, key=None, language="en-US")
                    print("text:", text)
                    numbers = ["one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten"]
                    if text in numbers:
                        text = str(numbers.index(text) + 1)
                    break
                except sr.UnknownValueError:
                    print(sr.UnknownValueError)
                    print("Try again; Google Speech Recognition could not understand audio")
                except sr.RequestError as e:
                    print("Try again; Could not request results from Google Speech Recognition service; {0}".format(e))
                except Exception:
                    print("Timeout")
        return text.lower()
    
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


    def takeOrder(self):
        print("Say \"Ready\" to begin your order!")
        screen.displayString("Say \"Ready\" to begin your order!")
        while True:
            self.__resetTable()
            wakeWord = self.__speechToText()
            if wakeWord != "ready":
                continue
            while True:
                print("What would you like to order?")
                screen.displayString("What would you like to order?")
                item = self.__speechToText()
                print(item)
                if item in list(self.menu.keys()):
                    print("How many?")
                    screen.displayString("How many?")
                    while True:
                        qty = self.__speechToText()
                        print("qty:", qty)
                        if qty.isnumeric():
                            break
                        print("Please repeat yourself!")
                        screen.displayString("Please repeat yourself!")
                    self.itemArray.append((item, int(qty)))
                    self.itemCount += int(qty)
                    self.cost += int(qty) * self.menu[item]
                    pass
                else:
                    print("Item not found in menu; try again")
                    screen.displayString("Item not found in menu; try again")
                    continue

                print("Would you like to order anything else?")
                screen.displayString("Would you like to order anything else?")
                orderMore = self.__speechToText()
                if any(word in orderMore for word in ["yes", "sure", "yeah", "yep", "yuppers", "yipee", "yes please", "absolutely", "you bet", "roger that", "certainly"]):
                    continue
                if any(word in orderMore for word in self.negativeResponses):
                    print("Any special requests?")
                    screen.displayString("Any special requests?")
                    specialRequestRaw = self.__speechToText()
                    print(specialRequestRaw)
                    if specialRequestRaw not in self.negativeResponses:
                        self.specialRequests = specialRequestRaw
                    self.__sendOrder()
                    print("Say \"Ready\" to begin your order!")
                    screen.displayString("Say \"Ready\" to begin your order!")
                    break

        
    def testOrder(self, itemArray):
        self.itemArray = itemArray
        for i in itemArray:
            self.itemCount += i[1]
        self.cost = 9999
        self.__sendOrder()
