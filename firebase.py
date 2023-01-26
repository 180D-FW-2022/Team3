# Import the Firebase service
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import random


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
        "tableNumber": random.randint(-10,-1),
        "kitchenReady" : False,
        "WOKerReady": False,
        "currentOrder": 0
    })

def getCurrentTable():
    return (ref.child("tableNumber").get())

def isKitchenReady():
    return (ref.child("kitchenReady").get())

def setWOKerReady(state):
    ref.child("WOKerReady").set(state)

# print(ref.child("tableNumber").get())
# Retrieve services via the auth package...
# auth.create_custom_token(...)