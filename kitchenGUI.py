from selectors import EVENT_WRITE
from kivy.metrics import dp
from kivy.uix.button import Button
# from kivy.clock import Clock
from kivymd.app import MDApp
from kivymd.uix.datatables import MDDataTable
from kivymd.uix.label import MDLabel
from kivymd.uix.screen import MDScreen
from kivymd.uix.boxlayout import MDBoxLayout
from kivymd.uix.floatlayout import MDFloatLayout
from kivymd.uix.button import MDRaisedButton
from kivymd.uix.screen import Screen
from kivy.core.window import Window

from collections import deque
import firebase
import asyncio


# import kitchenNode

class KitchenGUI(MDApp):
    # mainKitchenNode = kitchenNode.KitchenNode()
    orderList = []
    readyTable = -99
    # tableNoExists = True
    # latestOrder = []
    # prevNotReadyOrderTables = []
    orderNumbers = deque()

    def build(self):
        Window.bind(on_request_close=self.on_request_close)

        self.theme_cls.theme_style = "Dark"
        self.theme_cls.primary_palette = "Green"
        screen = Screen()
        self.row_data = []

        self.data_tables = MDDataTable(
            size_hint = (0.9, 0.9),
            pos_hint={"center_x": 0.5, "center_y": 0.5},
            use_pagination=False,
            check=False,
            column_data=[
                ("Order#", dp(25)),
                ("Table No.", dp(25)),
                ("Items", dp(50)),
                ("Qty", dp(25) ),
            ],
            rows_num = 15,
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

        self.listener = firebase.ref.child("sentOrders").listen(self.addRows)
        print("start")
        # self.listener.close()
        print("start2")

        pass
    
    def addRows(self, event):
        for i in event.data.items():
            latestItem = i[1]
            # print(latestItem)
            newTableNumber = latestItem["tableNumber"]
            newOrderNumber = latestItem["orderNumber"]
            itemString = "\n"
            qtyString = "\n"
            for item in latestItem["items"].items():
                itemString += item[0]
                itemString += '\n'
                qtyString += str(item[1])
                qtyString += '\n'
            self.data_tables.add_row(list((newOrderNumber, newTableNumber, itemString, qtyString)))
            self.orderNumbers.append(newOrderNumber)
            self.orderList.append(i)
            pass
    
    async def check(self, i):
        firebase.ref.child("readyOrders/order"+ str(i[0][5:])).set(i[1])

    def on_request_close(self, *args):
        print("exiting1")
        self.listener.close()
        print("exiting")


    def on_row_press(self, instance_table, instance_row):
        rowCount = 0
        index = int(instance_row.index/4)
        print(index)
        if instance_row.ids.check.state == 'normal':
            instance_row.ids.check.state = 'down'
        else:            
            instance_row.ids.check.state = 'normal'

        orderNumber = instance_table.row_data[index][0]
        print(orderNumber)

        instance_table.remove_row(instance_table.row_data[index])

        count = 0
        for i in self.orderList:
            if str(orderNumber) == i[0][5:]:
                print("found:", count)
                self.orderList.pop(count)
                print(self.orderList)
                # firebase.ref.child("readyOrders/order"+ str(i[0][5:])).set(i[1])
                asyncio.run(self.check(i))
                break
            count +=1

if __name__ == "__main__":
    GUI = KitchenGUI()
    GUI.run()
