from kivy.metrics import dp
from kivy.uix.button import Button
from kivymd.app import MDApp
from kivymd.uix.datatables import MDDataTable
from kivymd.uix.screen import MDScreen
from kivymd.uix.boxlayout import MDBoxLayout
from kivymd.uix.floatlayout import MDFloatLayout
from kivymd.uix.button import MDRaisedButton

tableNumber = 1
chickenSandwichQty = 2
friesQty = 3
milkshakeQty = 1
fruitQty = 0
burgerQty = 6
cheeseburgerQty = 2
specialRequest = "lactose intolerant"
totalItems = 15

class Example(MDApp):
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

    def build(self):
        self.row_data =  [(str(tableNumber), "Chicken Sandwich", str(chickenSandwichQty)),
            (str(tableNumber), "Fries", str(friesQty)),
            (str(tableNumber), "Milkshake", str(milkshakeQty)),
            (str(tableNumber), "Fruit", str(fruitQty)),
            (str(tableNumber), "Burger", str(burgerQty)),            
            (str(tableNumber), "Cheeseburger", str(cheeseburgerQty)),
            ]

        layout = MDFloatLayout()  # root layout
        # Creating control buttons.
        button_box = MDBoxLayout(
            pos_hint={"center_x": 0.5},
            adaptive_size=True,
            padding="24dp",
            spacing="24dp",
        )

        for button_text in [ "Remove row"]:
            button_box.add_widget(
                MDRaisedButton(
                    text=button_text, on_release=self.on_button_press
                )
            )

        self.data_tables = MDDataTable(
            # use_pagination=True,
            check=True,
            column_data=[
                ("Table No.", dp(30)),
                ("Item", dp(30)),
                ("Qty", dp(30) ),
            ],
            rows_num = 15,
            row_data = self.row_data,            
        )
        
        layout.add_widget(self.data_tables)
        layout.add_widget(button_box)

        return layout

    def on_button_press(self, instance_button: MDRaisedButton) -> None:
        '''Called when a control button is clicked.'''
        if(instance_button.text == "Remove row"):
            for row in self.data_tables.get_row_checks():
                # index = int(row[0])-1
                # del self.data_tables.row_data[index]
                self.data_tables.remove_row(tuple(row))

            for i, row in enumerate(self.data_tables.row_data):
                row_list = list(row)
                #row_list[0] = str(i+1)
                self.data_tables.row_data[i] = tuple(row_list)

    '''def addRow(self, item, status):
        item_num = len(self.data_tables.row_data)+1
        self.data_tables.row_data.append((item_num, item, status))'''

if __name__ == "__main__":
    Example().run()