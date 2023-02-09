# Import the Firebase service
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

cred = credentials.Certificate("firebase_key.json")
default_app = firebase_admin.initialize_app(cred, {'databaseURL':"https://d-database-c824d-default-rtdb.firebaseio.com"})
ref = db.reference()

def __wipeFirebase():
    nodes = ref.get()
    ref.delete()    

def resetFirebase():
    __wipeFirebase()
    ref.update({
        "Ready":False,
        "tableNumber": -99,
        "kitchenReady" : False,
        "WOKerReady": False,
        "currentOrder": 0
    })

def getCurrentTable():
    return (ref.child("tableNumber").get())

def isKitchenReady():
    return (ref.child("newTableReady").get())

#set WOKer to ready (True)
def WOKerReadyTrue():
    ref.child("WOKerReady").set(True)

#set robotReceived to recieved (True)
def robotReceivedTrue():
    ref.child("robotReceived").set(True)

def sendMap(map):
    ref.child("map").set(map)

def getMap():
    return ref.child("map").get()
