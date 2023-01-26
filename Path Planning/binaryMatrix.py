import collections

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

# Import the Firebase service
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import random

#test data
grid=[
    [1,0,0,1,0,0,1,1],
    [1,0,0,1,0,0,0,0],
    [1,0,0,1,0,0,1,1],
    [1,0,0,1,0,1,1,1],
    [1,0,0,0,0,1,1,1]
]
alley_v = np.zeros((9*2,1*2))
alley_h = np.zeros((1*2, 7*2))
table = 1*np.ones((1*2, 1*2))
table_alley = 2*np.ones((1*2, 1*2))
alleyAndTable_f = np.concatenate((table_alley, table, table_alley, table, table_alley, table, table_alley, table, table_alley), axis=0)
alleyAndV = np.concatenate((alley_v, alleyAndTable_f, alley_v, alleyAndTable_f, alley_v, alleyAndTable_f, alley_v), axis=1)
alleyF = np.concatenate((alley_h,alleyAndV,alley_h), axis = 0)
#test datae


cred = credentials.Certificate("firebase_key.json")
default_app = firebase_admin.initialize_app(cred, {'databaseURL':"https://d-database-c824d-default-rtdb.firebaseio.com"})
ref = db.reference()

def __wipeFirebase():
    nodes = ref.get()
    ref.delete()    

def resetFirebase():
    __wipeFirebase()
    ref.update({
        "Ready":False,
        "tableNumber": random.randint(-10,-1),
        "kitchenReady" : False,
        "WOKerReady": False,
        "currentOrder": 0
    })

def getCurrentTable():
    return (ref.child("tableNumber").get())

def isKitchenReady():
    return (ref.child("kitchenReady").get())

def setWOKerReady(state):
    ref.child("WOKerReady").set(state)

def calcPath(grid, start_m, start_n, goal_m, goal_n): 
    m,n=len(grid),len(grid[0])
    deque=collections.deque([[(start_m,start_n)]])
    seen=set()
    while deque:
        arr=deque.popleft()
        i,j=arr[-1]
        if (i,j)==(goal_m,goal_n):
            return arr
        seen.add((i,j))
        possible=[(x,y) for x,y in [(i+1,j),(i-1,j),(i,j+1),(i,j-1)] if 0<=x<m and 0<=y<n and grid[x][y]==0]
        for x,y in possible:
            if (x,y) not in seen:
                deque.append(arr+[(x,y)])

def calcMovesDistance(path_arr):    
    arr = path_arr
    cnt = 0
    arr_direction_y = []
    arr_direction_x = []
    arr_angle = []
    for i in arr:
        if(cnt>0):
            y_dir = i[0]-arr[cnt-1][0]
            x_dir = i[1]-arr[cnt-1][1]
            arr_direction_y.append(y_dir)
            arr_direction_x.append(x_dir)
            angle = 0
            if(y_dir == 1):
                angle = 0
            elif(y_dir == -1):
                angle = 180
            elif(x_dir == 1):
                angle = 90
            elif(x_dir == -1):
                angle = 270
            arr_angle.append(angle)
        cnt+=1

    ang_prev = arr_angle[0]
    distance = 0
    arr_distance = []
    arr_angle_clean = []
    counter = 0
    for j in range(1,len(arr_angle)):
        counter+= 1
        if(arr_angle[j] == ang_prev and counter+1 <= len(arr_angle)):
            ang_prev = arr_angle[j]
            distance+=0.5   
            if(counter+1 == len(arr_angle)):
                distance+=0.5
                arr_distance.append(distance)
                arr_angle_clean.append(ang_prev)
                distance = 0
        else:    
            distance+=0.5
            arr_distance.append(distance)
            arr_angle_clean.append(ang_prev)
            if(counter+1 == len(arr_angle)):
                arr_distance.append(0.5)
                arr_angle_clean.append(arr_angle[j])
            distance = 0
            ang_prev = arr_angle[j]
    return arr_distance

def calcMovesHeading(path_arr):    
    arr = path_arr
    cnt = 0
    arr_direction_y = []
    arr_direction_x = []
    arr_angle = []
    for i in arr:
        if(cnt>0):
            y_dir = i[0]-arr[cnt-1][0]
            x_dir = i[1]-arr[cnt-1][1]
            arr_direction_y.append(y_dir)
            arr_direction_x.append(x_dir)
            angle = 0
            if(y_dir == 1):
                angle = 0
            elif(y_dir == -1):
                angle = 180
            elif(x_dir == 1):
                angle = 90
            elif(x_dir == -1):
                angle = 270
            arr_angle.append(angle)
        cnt+=1

    ang_prev = arr_angle[0]
    distance = 0
    arr_distance = []
    arr_angle_clean = []
    counter = 0
    for j in range(1,len(arr_angle)):
        counter+= 1
        if(arr_angle[j] == ang_prev and counter+1 <= len(arr_angle)):
            ang_prev = arr_angle[j]
            distance+=0.5   
            if(counter+1 == len(arr_angle)):
                distance+=0.5
                arr_distance.append(distance)
                arr_angle_clean.append(ang_prev)
                distance = 0
        else:    
            distance+=0.5
            arr_distance.append(distance)
            arr_angle_clean.append(ang_prev)
            if(counter+1 == len(arr_angle)):
                arr_distance.append(0.5)
                arr_angle_clean.append(arr_angle[j])
            distance = 0
            ang_prev = arr_angle[j]        
    return arr_angle_clean
        

#alleyF = np.zeros((20,20))

#print(alleyF)

plt.ion()
fig, ax = plt.subplots()
x, y = [],[]
sc = ax.scatter(x,y)
colorsList = ['w', 'r', 'g']
cmap = mpl.colors.ListedColormap(colorsList)
plt.imshow(alleyF, cmap=cmap)
plt.xlim(-2,20)
plt.ylim(0,25)
plt.draw()

current_robotDir = 0 #90, 180, 270.
current_robotX = 0
current_robotY = 0

print(getCurrentTable())

headingTableY = 15
headingTableX = 5

arr = calcPath(alleyF, current_robotY, current_robotX, headingTableY, headingTableX) #y-start, x-start, y-end, x-end
distArr = []
headingArr = []

distances = calcMovesDistance(arr)
print("distance")
print(distances)
print()
angles = calcMovesHeading(arr)
print("angles")
print(angles)
print()
print()
print("STARTING TEST SEQUENCE")
print()

for i in range(len(angles)): 
    if(angles[i] == 0):
        print("moving "+str(distances[i])+"m")
        print()
    else:
        print("rotating "+str(angles[i])+"°")
        print()
        toSubtract = angles[i]
        for ind in range(len(angles)):
            angles[ind] = angles[ind]-toSubtract
        print("moving "+str(distances[i])+"m")
        print()
    
#display
x.clear()
y.clear()
for i in arr: 
        y.append(i[0])
for i in arr: 
        x.append(i[1])
sc.set_offsets(np.c_[x,y])
fig.canvas.draw_idle()
#print(arr)

plt.waitforbuttonpress()