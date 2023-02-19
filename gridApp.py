import kivy 
from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.gridlayout import GridLayout
from kivy.core.window import Window
import numpy as np
import firebase
from kivy.config import Config

#set window size
# Config.set('graphics', 'width', '1200')
# Config.set('graphics', 'height', '1200')

mat = np.zeros((20,20), dtype=int)
white = [255,255,255,1]
red = [255,0,0,1]
green = [0,255,0,1]
blue = [0,0,255,1]
black = [1,1,1,1]
color = [1,1,1,1]
code = 0
tableNum = 10
finalString = ""
isWokerPlaced = False

#Buttons: Open Space(White), Table(Green), Obstacles(Red), Table Number(Blue), WOKer Station(Black)

class gridLayout(App):
    global mat, white, red, green, blue, black, color, code, tableNum
    def build(self):
        Window.fullscreen = 'auto'
        #Window.size = (1200,1200)
        #set window size
        # Config.set('graphics', 'width', '800')
        # Config.set('graphics', 'height', '400')
        layout = GridLayout(cols=20)
        print(Window.width, Window.height)

        for i in range(400):
            btn = Button(background_color=[255,255,255,1], size_hint_x = None, width = 150, height = 150) #size = ((100,100))
            btn.id=str(i)
            layout.add_widget(btn)
            btn.bind(on_release = self.callback)

        doneBtn = Button(text = "Done", size = (100,100))
        tableButton = Button(text = "Table", background_color=[0,255,0,1]) #green
        obstacleButton = Button(text = "Obstacle", background_color=[255,0,0,1]) #red
        tableNumber = Button(text = "Table No.", background_color=[0,0,255,1]) #blue
        wokerStation = Button(text = "WOKer", background_color=[1,1,1,1]) #black
        deselect = Button(text = "Deselect", background_color = white) #white

        tableButton.id="Table"
        obstacleButton.id="Obstacle"
        tableNumber.id="Table No."
        wokerStation.id="WOKer"
        doneBtn.id = "Done"
        deselect.id = "Deselect"

        layout.add_widget(tableButton)
        layout.add_widget(obstacleButton)
        layout.add_widget(tableNumber)
        layout.add_widget(wokerStation)
        layout.add_widget(deselect)

        doneBtn.bind(on_release = self.buttonPress)
        tableButton.bind(on_release = self.buttonPress)
        obstacleButton.bind(on_release = self.buttonPress)
        wokerStation.bind(on_release = self.buttonPress)
        tableNumber.bind(on_release = self.buttonPress)
        deselect.bind(on_release = self.buttonPress)

        for i in range(5):
            dummyBtn = Button(background_color=[1,1,1,1])
            layout.add_widget(dummyBtn)

        layout.add_widget(doneBtn)
        
        for i in range(9):
            dummyBtn = Button(background_color=[1,1,1,1])
            layout.add_widget(dummyBtn)
        
        return layout

    def buttonPress(self, instance):
        global color, code, finalString, tableNum
        print(instance.id)
        if(instance.id == "Table"):
            color = green
            code = 1
        elif(instance.id == "Obstacle"):
            color = red
            code = 2
        elif(instance.id == "Table No."):
            color = blue
            # code = tableNum
            # tableNum = tableNum+1
        elif(instance.id == "WOKer"):
            color = black
            code = 4
        elif(instance.id == "Deselect"):
            color = white
            code = 0
        elif(instance.id == "Done"):
            #print(mat)
            Window.close()
            for line in mat:
                for element in line:
                    if finalString == "":
                        finalString = str(element)
                    else:
                        finalString = str(finalString) + "," + str(element)
            firebase.sendMap(finalString)
        else:
            color = white
            code = 0

        print(finalString)

    def getTuple(self, x):
        x1 = int(x/20)
        x2 = int(x%20)
        return (x1,x2)

    def callback(self, event):
        global color, tableNum, isWokerPlaced
        (a, b) = self.getTuple(int(event.id))

        if(color == white):
                if(event.background_color == blue):
                    event.background_color = green
                    tableNum = tableNum - 1
                    mat[int(a)][int(b)] = int(1)
                elif(event.background_color == black):
                    event.background_color = color
                    isWokerPlaced = False
                    mat[int(a)][int(b)] = int(1)
                else:
                    event.background_color = color
                    mat[int(a)][int(b)] = int(1)
        elif(color == blue):
            if(event.background_color == green):
                event.background_color = color
                mat[int(a)][int(b)] = tableNum
                event.text = str(tableNum-9)
                tableNum = tableNum + 1
                print(tableNum)
        elif(color == black):
            if(isWokerPlaced == False):
                event.background_color = color
                mat[int(a)][int(b)] = int(code)
                isWokerPlaced = True
        else:
            print(color)
            event.background_color = color
            mat[int(a)][int(b)] = int(code)

gridLayout().run()
