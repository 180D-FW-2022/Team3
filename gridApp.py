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
pink = [245, 40, 145, 0.8]
color = [1,1,1,1]
code = 0
tableNum = 10
finalString = ""
isWokerPlaced = False
layout = GridLayout(cols=20)

#Buttons: Open Space(White), Table(Green), Obstacles(Red), Table Number(Blue), WOKer Station(Black)

class gridLayout(App):
    global mat, white, red, green, blue, black, color, code, tableNum

    def otherButtons(self, layout):
        doneBtn = Button(text = "Done", size = (100,100))
        tableButton = Button(text = "Table", background_color=[0,255,0,1]) #green
        obstacleButton = Button(text = "Obstacle", background_color=[255,0,0,1]) #red
        tableNumber = Button(text = "Table No.", background_color=[0,0,255,1]) #blue
        wokerStation = Button(text = "WOKer", background_color=[1,1,1,1]) #black
        deselect = Button(text = "Deselect", background_color = white) #white
        loadOldMap = Button(text = "Load Map", background_color = pink) 

        tableButton.id="Table"
        obstacleButton.id="Obstacle"
        tableNumber.id="Table No."
        wokerStation.id="WOKer"
        doneBtn.id = "Done"
        deselect.id = "Deselect"
        loadOldMap.id = "Load Map"

        layout.add_widget(tableButton)
        layout.add_widget(obstacleButton)
        layout.add_widget(tableNumber)
        layout.add_widget(wokerStation)
        layout.add_widget(deselect)
        layout.add_widget(loadOldMap)

        doneBtn.bind(on_release = self.buttonPress)
        tableButton.bind(on_release = self.buttonPress)
        obstacleButton.bind(on_release = self.buttonPress)
        wokerStation.bind(on_release = self.buttonPress)
        tableNumber.bind(on_release = self.buttonPress)
        deselect.bind(on_release = self.buttonPress)
        loadOldMap.bind(on_release = self.oldMap)

        for i in range(4):
            dummyBtn = Button(background_color=[1,1,1,1])
            layout.add_widget(dummyBtn)

        layout.add_widget(doneBtn)
        
        for i in range(9):
            dummyBtn = Button(background_color=[1,1,1,1])
            layout.add_widget(dummyBtn)
        
        return layout

    def build(self):
        global layout
        Window.fullscreen = 'auto'
        #layout = GridLayout(cols=20)
        #print(Window.width, Window.height)

        for i in range(400):
            btn = Button(background_color=[255,255,255,1], size_hint_x = None, width = 150, height = 150) #size = ((100,100))
            btn.id=str(i)
            layout.add_widget(btn)
            btn.bind(on_release = self.callback)
        
        return self.otherButtons(layout)

    def oldMap(self, instance):
        global layout, tableNum
        layout.clear_widgets()
        Window.fullscreen = 'auto'
        oldMapString = firebase.getMap()
        oldMapString += ','
        #print(oldMapString)
        element = ""
        countID = 0
        tables = 0
        for i in oldMapString:
            if i < '99' and i >= '0':
                element += i
            elif i == ',':
                if(element < '0' or element > '99'):
                    continue
                elementNum = int(element)
                print(elementNum)
                if(elementNum == 0):
                    btn = Button(background_color=white, size_hint_x = None, width = 150, height = 150)
                    btn.id=countID
                    layout.add_widget(btn)
                    mat[int(countID/20)][int(countID%20)] = elementNum
                    btn.bind(on_release = self.callback)
                elif(elementNum == 1):
                    btn = Button(background_color=green, size_hint_x = None, width = 150, height = 150) #size = ((100,100))
                    btn.id=countID
                    layout.add_widget(btn)
                    mat[int(countID/20)][int(countID%20)] = elementNum
                    btn.bind(on_release = self.callback)
                elif(elementNum == 2):
                    btn = Button(background_color=red, size_hint_x = None, width = 150, height = 150) #size = ((100,100))
                    btn.id=countID
                    layout.add_widget(btn)
                    mat[int(countID/20)][int(countID%20)] = elementNum
                    btn.bind(on_release = self.callback)
                elif(elementNum == 4):
                    btn = Button(background_color=black, size_hint_x = None, width = 150, height = 150) #size = ((100,100))
                    btn.id=countID
                    layout.add_widget(btn)
                    mat[int(countID/20)][int(countID%20)] = elementNum
                    btn.bind(on_release = self.callback)
                else:
                    btn = Button(text = str(elementNum - 9),background_color=blue, size_hint_x = None, width = 150, height = 150) #size = ((100,100))
                    btn.id=countID
                    layout.add_widget(btn)
                    mat[int(countID/20)][int(countID%20)] = elementNum
                    btn.bind(on_release = self.callback)
                    tables += 1
                countID+=1
                element = ""
        tableNum = tableNum+tables
        return self.otherButtons(layout)

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
        elif(instance.id == "WOKer"):
            color = black
            code = 4
        elif(instance.id == "Deselect"):
            color = white
            code = 0
        elif(instance.id == "Done"):
            Window.close()
            for line in mat:
                for element in line:
                    if finalString == "":
                        finalString = str(element)
                    else:
                        finalString = str(finalString) + "," + str(element)
            print(finalString)
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
