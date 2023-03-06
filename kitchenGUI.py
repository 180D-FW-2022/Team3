import os, sys
from kivy.resources import resource_add_path, resource_find

from kivy.metrics import dp
from kivymd.app import MDApp
from kivymd.uix.datatables import MDDataTable
from kivymd.uix.screen import Screen
from kivy.core.window import Window

from collections import deque
import firebase
import asyncio
import traceback

class customEvent:
    data: True

class KitchenGUI(MDApp):
    # mainKitchenNode = kitchenNode.KitchenNode()
    orderList = []
    readyTable = -99
    # tableNoExists = True
    # latestOrder = []
    # prevNotReadyOrderTables = []
    orderNumbers = deque()


    def build(self):
        # Window.bind(on_request_close=self.on_request_close)

        self.theme_cls.theme_style = "Dark"
        self.theme_cls.primary_palette = "Green"
        screen = Screen()
        self.row_data = []
        columnWidth = 50
        self.data_tables = MDDataTable(
            size_hint = (0.9, 0.9),
            pos_hint={"center_x": 0.5, "center_y": 0.5},
            use_pagination=False,
            check=False,
            column_data=[
                ("Order#", dp(columnWidth)),
                ("Table No.", dp(columnWidth)),
                ("Items", dp(columnWidth)),
                ("Qty", dp(columnWidth) ),
                ("Special Requests", dp(columnWidth))
            ],
            rows_num = 100,
            row_data = self.row_data,    
            sorted_on = "Table No."        
        )
        
        # button_box = MDBoxLayout(
        #     pos_hint={"center_x": 0.5},
        #     adaptive_size=True,
        #     padding="24dp",
        #     spacing="24dp",
        # )

        # for button_text in ["Add row", "Remove row"]:
        #     button_box.add_widget(
        #         MDLabel(
        #             text=button_text
        #         )
        #     )

        # self.data_tables.bind(on_check_press=self.on_check_press)
        self.data_tables.bind(on_row_press=self.on_row_press)
        self.title = "Kitchen Node"
        # screen.add_widget(
        #     MDLabel(
        #         text="test",
        #         # halign="center",
        #     )
        # )
        screen.add_widget(self.data_tables)
        # screen.add_widget(button_box)

        return screen
    
    def on_start(self):
        self.initial = [True, True, True]
        self.listener = firebase.ref.child("sentOrders").listen(self.addRows)
        self.readyOrdersListener = firebase.ref.child("readyOrders").listen(self.setNewTable)
        self.WOKerReadyListener = firebase.ref.child("WOKerReady").listen(self.WOKerReady)
        self.robotReceievedListener = firebase.ref.child("robotReceived").listen(self.robotReceived)
        # self.listener.close()

        pass
    
    def addRows(self, event):
        if event.data == None:
            return
        if "cost" in event.data.keys():
            print("if")
            latestItem = event.data
            # print(latestItem)
            newTableNumber = latestItem["tableNumber"]
            newOrderNumber = latestItem["orderNumber"]
            specialRequest = latestItem["specialRequests"]
            # itemString = "\n"
            # qtyString = "\n"
            for item in latestItem["items"].items():
                itemString = item[0]
                # itemString += '\n'
                qtyString = str(item[1])
                # qtyString += '\n'
                self.data_tables.add_row(list((newOrderNumber, newTableNumber, itemString, qtyString, specialRequest)))
                self.orderNumbers.append(newOrderNumber)
            self.orderList.append( (
                "order" + str(newOrderNumber), latestItem
             ) )
        else:
            print("else")
            for i in event.data.items():
                latestItem = i[1]
                newTableNumber = latestItem["tableNumber"]
                newOrderNumber = latestItem["orderNumber"]
                specialRequest = latestItem["specialRequests"]
                # itemString = "\n\n\n\n"
                # qtyString = "\n"
                for item in latestItem["items"].items():
                    itemString = item[0]
                    # itemString += '\n'
                    qtyString = str(item[1])
                    # qtyString += '\n'
                    # itemString += '\n\n\n'
                    self.data_tables.add_row(list((newOrderNumber, newTableNumber, itemString, qtyString, specialRequest)))
                    self.orderNumbers.append(newOrderNumber)
                self.orderList.append(i)
                # print(self.orderNumbers)
                pass

    def setNewTable(self, event):
        if firebase.ref.child("readyOrders").get() == None:
                return
        # if self.initial[0] == True:
        #     print("initial1")
        #     self.initial[0] = False
        #     return
        firstOrder = list(firebase.ref.child("readyOrders").get().keys())[0]
        tableNumber = firebase.ref.child("readyOrders").get()[firstOrder]["tableNumber"]
        firebase.ref.child("newTableNumber").set(tableNumber)

    def WOKerReady(self, event):
        # if self.initial[1] == True:
        #     print("initial2")
        #     self.initial[1] = False
        print(event.data)
        if event.data == True:
            if firebase.ref.child("readyOrders").get() == None:
                print("return")
                return
            # print("WOKER")
            firebase.ref.child("tableNumber").set(firebase.ref.child("newTableNumber").get())
            orders = []
            for i in firebase.ref.child("readyOrders").get().keys():
                # print(int(i[5:]))
                orders.append(int(i[5:]))
            orders.sort()
            print(orders)
            firstOrder = "order" + str(orders[0])
            # firstOrder = list(firebase.ref.child("readyOrders").get().keys())[0]
            print(firstOrder)
            firebase.ref.child("pastOrders/" + firstOrder).set(firebase.ref.child("readyOrders/" + firstOrder).get())
            firebase.ref.child("readyOrders/" + firstOrder).delete()
            firebase.ref.child("newTableReady").set(True)
            pass
        elif event.data == False:
            pass
            
    def robotReceived(self, event):
        if self.initial[2] == True:
            print("initial3")
            self.initial[2] = False
            return
        if event.data == True:
            # if firebase.ref.child("readyOrders").get() == None:
            #     return
            firebase.ref.child("robotReceived").set(False)
            firebase.ref.child("newTableReady").set(False)
            firebase.ref.child("WOKerReady").set(False)        


    def on_stop(self):
        print("exiting...")
        self.listener.close()
        self.readyOrdersListener.close()
        self.WOKerReadyListener.close()
        self.robotReceievedListener.close()

    async def check(self, i):
        firebase.ref.child("readyOrders/order"+ str(i[0][5:])).set(i[1])
        firebase.ref.child("sentOrders/order" + str(i[0][5:])).delete()
        event2 = customEvent()
        event2.data = firebase.ref.child("WOKerReady").get()
        print(event2.data)
        self.WOKerReady(event2)
        

    def on_row_press(self, instance_table, instance_row):
        rowCount = 0
        index = int(instance_row.index/5)
        # print(index)
        if instance_row.ids.check.state == 'normal':
            instance_row.ids.check.state = 'down'
        else:            
            instance_row.ids.check.state = 'normal'

        orderNumber = instance_table.row_data[index][0]
        # print(orderNumber)

        instance_table.remove_row(instance_table.row_data[index])
        
        print(self.orderList)
        self.orderNumbers.remove(orderNumber)
        if orderNumber not in self.orderNumbers:
            count = 0
            for i in self.orderList:
                if str(orderNumber) == i[0][5:]:
                    # print("found:", count)
                    self.orderList.pop(count)
                    # print(self.orderList)
                    # firebase.ref.child("readyOrders/order"+ str(i[0][5:])).set(i[1])
                    asyncio.run(self.check(i))
                    break
                count +=1

if __name__ == "__main__":
    try:
        if hasattr(sys, '_MEIPASS'):
            resource_add_path(os.path.join(sys._MEIPASS))
        GUI = KitchenGUI()
        GUI.run()
    except Exception as e:
        print(traceback.format_exc())
        input("Press enter.")
