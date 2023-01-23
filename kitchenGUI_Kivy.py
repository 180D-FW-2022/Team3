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
from collections import deque

import kitchenNode

class KitchenGUI(MDApp):
    mainKitchenNode = kitchenNode.KitchenNode()
    orderList = []
    readyTable = -99
    tableNoExists = True
    latestOrder = []
    prevNotReadyOrderTables = []

    def build(self):
        self.theme_cls.theme_style = "Dark"
        self.theme_cls.primary_palette = "Green"
        screen = Screen()
        self.row_data = []

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
        Clock.schedule_interval(self.mainKitchenNode.orderSend, 1/4)
        self.orderList = self.mainKitchenNode.orderList.copy()

    def checkForOrder(self, dt, event=None):
        # if self.orderList != self.mainKitchenNode.orderList:
        #     self.orderList = self.mainKitchenNode.orderList.copy()
        #     if len(self.orderList) != 0:
        #         self.addRows(self.orderList[len(self.orderList)-1]) #add rows
        if len(self.mainKitchenNode.orderList) > 0:
            if self.latestOrder != self.mainKitchenNode.orderList[len(self.mainKitchenNode.orderList)-1]:
                self.latestOrder = self.mainKitchenNode.orderList[len(self.mainKitchenNode.orderList)-1]
                self.addRows(self.latestOrder) #add rows
    
    def addRows(self, latestItem):
        newTableNumber = latestItem["tableNumber"]
        for item in latestItem["itemsArray"]:
            self.data_tables.add_row(list((newTableNumber, item[0], item[1])))
        pass
    def on_check_press(self, instance_table, instance_row):
        # self.servingTable = int(self.data_tables.row_data[0][0])
        # print(self.servingTable)
        prevNotReadyOrderTables = []
        for i in self.data_tables.row_data:
            if i[0] not in prevNotReadyOrderTables:
                prevNotReadyOrderTables.append(i[0])
        
        rowCount = 0
        index = instance_table.row_data.index(instance_row)*3
        cols_num = len(instance_table. column_data)
        row_num = int(index/cols_num)
        cell_row =instance_table.table_data.view_adapter.get_visible_view(row_num*cols_num)
        cell_row.change_check_state_no_notify("normal")
        instance_table.remove_row(instance_row) 
        
        newNotReadyOrderTables = []
        for i in instance_table.row_data:
            if i[0] not in newNotReadyOrderTables:
                newNotReadyOrderTables.append(i[0])                

        if (len(set(prevNotReadyOrderTables) - set(newNotReadyOrderTables)) != 0):
            
            print(set(prevNotReadyOrderTables) - set(newNotReadyOrderTables))
            print("length: " , len(set(prevNotReadyOrderTables) - set(newNotReadyOrderTables)))
            # for i in (set(prevNotReadyOrderTables) - set(newNotReadyOrderTables)):
            #     print(i)
            self.readyTable = next(iter(set(prevNotReadyOrderTables) - set(newNotReadyOrderTables)))
            print("Table", self.readyTable, "Ready")
            self.mainKitchenNode.readyTables.append(self.readyTable)
            print(self.mainKitchenNode.readyTables)
            self.mainKitchenNode.orderComplete()
        # self.prevNotReadyOrderTables = notReadyOrderTables.copy()

        # # print("Serving: ", self.servingTable)
        # for row in self.data_tables.row_data:
        #     rowCount = rowCount+1
        #     row_list = list(row)
        #     if row_list[0] == str(self.servingTable):
        #         self.tableNoExists = True
        #         break
        #     else:
        #         self.tableNoExists = False
        # # print("tableNoExists: ", self.tableNoExists)

        # if (self.tableNoExists == False or rowCount == 0):
        #     print("Table", self.servingTable, "Ready")
        #     self.mainKitchenNode.readyTable.append(self.servingTable)
        #     print(self.mainKitchenNode.readyTable)
        #     self.mainKitchenNode.orderComplete()

if __name__ == "__main__":
    GUI = KitchenGUI()
    GUI.run()
