# Import the Firebase service
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import random

class Firebase():
    ref = []

    def __init__(self, num):
        print("in")
        cred = credentials.Certificate("firebase_key.json")
        # Initialize the default app
        self.ref = db.reference("")

    def __wipeFirebase(self):
        ref = db.reference("")
        nodes = ref.get()
        ref.delete()    

    def resetFirebase(self):
        self.__wipeFirebase()
        ref = db.reference("")
        ref.update({"Ready":False})
        ref.update({"tableNumber":random.randint(-10,-1)})
        ref.update({"kitchenReady":False})
        ref.update({"WOKerReady": False})


# print(ref.child("tableNumber").get())
# Retrieve services via the auth package...
# auth.create_custom_token(...)