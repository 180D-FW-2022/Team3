# Import the Firebase service
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import sys
import os


def resource_path(relative_path):
    # """ Get absolute path to resource, works for dev and for PyInstaller """
    # try:
    #     # PyInstaller creates a temp folder and stores path in _MEIPASS
    #     base_path = sys._MEIPASS
    # except Exception:
    #     base_path = os.path.abspath(".")

    # print("base:", base_path)
    # print("relative:", relative_path)

    realPath = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(realPath, relative_path)

keyPath = resource_path("firebase_key.json")
print(keyPath)

cred = credentials.Certificate(keyPath)
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
    return (ref.child("newTableNumber").get())

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
