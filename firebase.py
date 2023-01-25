# Import the Firebase service
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import random


cred = credentials.Certificate("firebase_key.json")
default_app = firebase_admin.initialize_app(cred, {'databaseURL':"https://d-database-c824d-default-rtdb.firebaseio.com"})
ref = db.reference()

def __wipeFirebase():
    ref = db.reference("")
    nodes = ref.get()
    ref.delete()    

def resetFirebase():
    __wipeFirebase()
    ref = db.reference("")
    ref.update({
        "Ready":False,
        "tableNumber": random.randint(-10,-1),
        "kitchenReady" : False,
        "WOKerReady": False,
        "currentOrder": 0
    })


# print(ref.child("tableNumber").get())
# Retrieve services via the auth package...
# auth.create_custom_token(...)