import speech_recognition as sr

def speechToText():
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

def takeOrder(): 
    #menu dictionary with items and prices
    menu = {
        "chicken sandwich": 10,
        "fry's": 5,
        "milkshake": 2.5,
        "fruit": 3,
        "burger": 8,
        "cheeseburger": 8.50
    }

    print("What would you like to order?")
    tableNumber = 1
    itemArray = []
    itemCount = 0
    specialRequests = 'N/A'
    cost = 0
    while True:
        item = ''
        item = speechToText()

        if item == "no":
            print("Any special requests?")
            specialRequestRaw = speechToText()
            if specialRequestRaw != 'no':
                specialRequests = specialRequestRaw
            break
        
        print(item)
        if item in list(menu.keys()):
            print("How many?")
            qty = speechToText()
            print(qty)
            itemArray.append((item, int(qty)))
            itemCount = itemCount + 1
            cost = cost + menu[item]
            pass
        else:  
            print("Item not found; try again")
            continue

        print("Would you like to order anything else?")
    
    return tableNumber, itemArray, itemCount, cost, specialRequests

print(takeOrder())