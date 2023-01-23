# Import the Firebase service
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import random

class firebase():
    def connectToFirebase():
        cred = credentials.Certificate("firebase_key.json")
        # Initialize the default app
        default_app = firebase_admin.initialize_app(cred, {'databaseURL':"https://d-database-c824d-default-rtdb.firebaseio.com"})

    def __wipeFirebase(self):
        ref = db.reference("")
        nodes = ref.get()
        for i in nodes:
            # print(i,": ",nodes[i])
            ref.delete()

    def resetFirebase(self):
        self.connectToFirebase()
        self.wipeFirebase()
        ref = db.reference("")
        ref.update({"Ready":False})
        ref.update({"tableNumber":random.randint(-10,-1)})
        ref.update({"kitchenReady":False})
        ref.update({"WOKerReady": False})

    ref = db.reference("")


# print(ref.child("tableNumber").get())
# Retrieve services via the auth package...
# auth.create_custom_token(...)