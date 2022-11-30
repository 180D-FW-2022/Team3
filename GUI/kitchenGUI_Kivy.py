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

tableNumber = 2
chickenSandwichQty = 2
friesQty = 3
milkshakeQty = 1
fruitQty = 0
burgerQty = 6
cheeseburgerQty = 2
specialRequest = "lactose intolerant"
totalItems = 15
servingTable = 1
tableNoExists = True

class KitchenGUI(MDApp):
    def generateNewOrder():
        tableNumber = random.randint(0, 5)
        chickenSandwichQty = random.randint(0, 5)
        friesQty = random.randint(0, 5)
        milkshakeQty = random.randint(0, 5)
        fruitQty = random.randint(0, 5)
        burgerQty = random.randint(0, 5)
        cheeseburgerQty = random.randint(0, 5)
        totalItems = chickenSandwichQty+friesQty+milkshakeQty+fruitQty+burgerQty+cheeseburgerQty
        specialRequest = "lactose intolerant"

    def test(self):
        print("test")

    def build(self):
        self.theme_cls.theme_style = "Dark"
        self.theme_cls.primary_palette = "Green"
        screen = Screen()
        self.row_data =  [("1", "Chicken Sandwich", str(chickenSandwichQty)),
            ("2", "Fries", str(friesQty)),
            ("2", "Milkshake", str(milkshakeQty)),
            ("1", "Fruit", str(fruitQty)),
            ("2", "Burger", str(burgerQty)),            
            ("2", "Cheeseburger", str(cheeseburgerQty)),
            ]

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
        # event = Clock.schedule_interval(self.checkForOrder, 1/4.)
        return screen

    def checkForOrder(dt, event=None):
        print("empty for now")

    def on_check_press(self, instance_table, instance_row):        
        global tableNoExists
        global servingTable
        rowCount = 0

        index = instance_table.row_data.index(tuple(instance_row))*3
        cols_num = len(instance_table. column_data)
        row_num = int(index/cols_num)
        cell_row =instance_table.table_data.view_adapter.get_visible_view(row_num*cols_num)
        cell_row.change_check_state_no_notify("normal")
        instance_table.remove_row(tuple(instance_row)) 
        print("serving: ", servingTable)
        for row in self.data_tables.row_data:
            rowCount = rowCount+1
            row_list = list(row)
            print("Row list: ", row_list[0])
            if row_list[0] == str(servingTable):
                tableNoExists = True
                break
            else:
                tableNoExists = False
        print("tableNoExists: ", tableNoExists)

        if(rowCount == 0):
            print("Table", servingTable, "Ready ")
            # print("send command because empty")
        
        if (tableNoExists == False):
            print("Table", servingTable, "Ready")
            servingTable = servingTable+1
            # print("servingTable = " + str(servingTable))

if __name__ == "__main__":
    GUI = KitchenGUI()
    GUI.run()
