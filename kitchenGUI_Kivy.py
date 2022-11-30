from selectors import EVENT_WRITE
from kivy.metrics import dp
from kivy.uix.button import Button
from kivy.clock import Clock
from kivymd.app import MDApp
from kivymd.uix.datatables import MDDataTable
from kivymd.uix.screen import MDScreen
from kivymd.uix.boxlayout import MDBoxLayout
from kivymd.uix.floatlayout import MDFloatLayout
from kivymd.uix.button import MDRaisedButton
from kivymd.uix.screen import Screen
import random
import kitchenNode
from collections import deque


# tableNumber = 2
# chickenSandwichQty = 2
# friesQty = 3
# milkshakeQty = 1
# fruitQty = 0
# burgerQty = 6
# cheeseburgerQty = 2
# specialRequest = "lactose intolerant"
# totalItems = 15

class KitchenGUI(MDApp):
    mainKitchenNode = kitchenNode.KitchenNode()
    orderList = []
    servingTable = -99
    tableNoExists = True
    # def generateNewOrder():
    #     tableNumber = random.randint(0, 5)
    #     chickenSandwichQty = random.randint(0, 5)
    #     friesQty = random.randint(0, 5)
    #     milkshakeQty = random.randint(0, 5)
    #     fruitQty = random.randint(0, 5)
    #     burgerQty = random.randint(0, 5)
    #     cheeseburgerQty = random.randint(0, 5)
    #     totalItems = chickenSandwichQty+friesQty+milkshakeQty+fruitQty+burgerQty+cheeseburgerQty
    #     specialRequest = "lactose intolerant"

    def test(self):
        print("test")

    def build(self):
        self.theme_cls.theme_style = "Dark"
        self.theme_cls.primary_palette = "Green"
        screen = Screen()
        self.row_data = []
        # self.row_data =  [("1", "Chicken Sandwich", str(chickenSandwichQty)),
        #     ("2", "Fries", str(friesQty)),
        #     ("2", "Milkshake", str(milkshakeQty)),
        #     ("1", "Fruit", str(fruitQty)),
        #     ("2", "Burger", str(burgerQty)),            
        #     ("2", "Cheeseburger", str(cheeseburgerQty)),
        #     ]

        self.data_tables = MDDataTable(
            size_hint = (0.9, 0.9),
            pos_hint={"center_x": 0.5, "center_y": 0.5},
            use_pagination=False,
            check=True,
            column_data=[
                ("Table No.", dp(100)),
                ("Item", dp(100)),
                ("Qty", dp(100) ),
            ],
            rows_num = 15,
            row_data = self.row_data,    
            sorted_on = "Table No."        
        )
        
        self.data_tables.bind(on_check_press=self.on_check_press)
        
        screen.add_widget(self.data_tables)
        return screen
    
    def on_start(self):
        self.mainKitchenNode.run()
        Clock.schedule_interval(self.checkForOrder, 1/4.)
        self.orderList = self.mainKitchenNode.orderList.copy()

    def checkForOrder(self, dt, event=None):
        if self.orderList != self.mainKitchenNode.orderList:
            # print(self.orderList == self.mainKitchenNode.orderList)
            self.orderList = self.mainKitchenNode.orderList.copy()
            # print("orderList: " , self.orderList[len(self.orderList)-1])

            self.addRows(self.orderList[len(self.orderList)-1]) #add rows
    
    def addRows(self, latestItem):
        print("in add rows")
        newTableNumber = latestItem["tableNumber"]
        for item in latestItem["itemsArray"]:
            self.data_tables.add_row(list((newTableNumber, item[0], item[1])))
        pass

    def on_check_press(self, instance_table, instance_row):
        print(self.row_data)
        rowCount = 0
        index = instance_table.row_data.index(instance_row)*3
        cols_num = len(instance_table. column_data)
        row_num = int(index/cols_num)
        cell_row =instance_table.table_data.view_adapter.get_visible_view(row_num*cols_num)
        cell_row.change_check_state_no_notify("normal")
        instance_table.remove_row(instance_row) 
        print("serving: ", self.servingTable)
        for row in self.data_tables.row_data:
            rowCount = rowCount+1
            row_list = list(row)
            if row_list[0] == str(self.servingTable):
                self.tableNoExists = True
                break
            else:
                self.tableNoExists = False
        print("tableNoExists: ", self.tableNoExists)

        if(rowCount == 0):
            print("Table", self.servingTable, "Ready ")
            # print("send command because empty")
        
        if (self.tableNoExists == False):
            print("Table", self.servingTable, "Ready")
            self.servingTable = self.servingTable+1
            # print("servingTable = " + str(servingTable))

if __name__ == "__main__":
    GUI = KitchenGUI()
    GUI.run()
