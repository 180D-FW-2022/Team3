from pupil_apriltags import Detector
import cv2 as cv

import numpy as np
import time
import math
import struct
import collections

import warnings
import serial
import serial.tools.list_ports
import time

import matplotlib.pyplot as plt
import matplotlib as mpl

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

# Import the Firebase service
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db


test_mode = 1
#global consts
test_tag_id = 0
distanceScale = 0.75

moveTimeoutConst = 15 #seconds

cam_id = 0 #"/dev/video0"
scale = 1.0

font = cv.FONT_HERSHEY_SIMPLEX

movementDone = True
moveStartTimestamp = 0
stepCounter = 0


current_robotDir = 0 #90, 180, 270.
current_robotX = 0
current_robotY = 0


cred = credentials.Certificate("firebase_key.json")
default_app = firebase_admin.initialize_app(cred, {'databaseURL':"https://d-database-c824d-default-rtdb.firebaseio.com"})
ref = db.reference()

def __wipeFirebase():
    nodes = ref.get()
    ref.delete()    

def getCurrentTable():
    return (ref.child("tableNumber").get())

def isKitchenReady():
    return (ref.child("kitchenReady").get())

def setWOKerReady(state):
    ref.child("WOKerReady").set(state)

def getMap():
    return ref.child("map").get()


fetched_map = getMap().split(',')
fetched_map_matrix = np.reshape(fetched_map, (20, 20))
grid_raw = fetched_map_matrix.astype(int)
grid_raw[grid_raw > 0] = -1
grid_raw[grid_raw == 0] = 1
grid_raw[grid_raw == -1] = 0

print(grid_raw)

#arduino connect pySerial
if(test_mode == 0):
    arduino_ports = [
        p.device
        for p in serial.tools.list_ports.comports()
        if ('Arduino' in p.description or 'USB Serial' in p.description)  # may need tweaking to match new arduinos
    ]
    if not arduino_ports:
        raise IOError("[arduino ] Not found")
    if len(arduino_ports) > 1:
        warnings.warn('[arduino ] Multiple found - using the first')
    ser = serial.Serial(arduino_ports[0], baudrate = 115200, timeout = 0.5)
    time.sleep(2)
    print(ser.name)         # check which port was really used

#video init.
cap = cv.VideoCapture(cam_id)
if not cap.isOpened():
    print("[ camera ] Cannot open")
    exit()
    
#APTag detector init.
at_detector = Detector(
   families="tag36h11",
   nthreads=1,
   quad_decimate=2.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0,
)



#display inits

plt.ion()
fig, ax = plt.subplots()
x, y = [],[]
sc = ax.scatter(x,y)
colorsList = ['w', 'r', 'g']
cmap = mpl.colors.ListedColormap(colorsList)
plt.imshow(grid_raw, cmap=cmap)
plt.xlim(-1,21)
plt.ylim(-1,21)
plt.draw()

#end






#functions
def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv.resize(frame, dim, interpolation =cv.INTER_AREA)

def send_distance(distance, direction = 'b'):
    if(test_mode == 0):  
        distance_send = int(abs(distance))
        print("[distance]: move: "+ str(distance_send))
        #bytes_val = distance_send.to_bytes(2, 'big', signed=True)
        bytes_data = struct.pack('>h', distance_send)
        print(bytes_data)
        ser.write(str.encode('b'))
        ser.write(bytes_data)
        if(bytes_data == ser.read(2)):
            print("[distance]: received")
        else:
            ser.write(b'\x00') 
            ser.write(b'\x00') 
            ser.write(b'\x00') 
        # movementDone = True 
            time.sleep(0.1)

def send_angle(angle):
    if(test_mode == 0):  
        angle_send = int(angle)
        print("[ angle  ]: move: "+str(angle_send))
        #bytes_val = angle_send.to_bytes(2, 'big', signed=True)
        bytes_data = struct.pack('>h', angle_send)
        print(bytes_data)
        ser.write(str.encode('d'))
        ser.write(bytes_data)
        if(bytes_data == ser.read(2)):
            print("[ angle  ]: received")
        else:
            ser.write(b'\x00') 
            ser.write(b'\x00') 
            ser.write(b'\x00') 
        # movementDone = True
            time.sleep(0.1)

def process_april_tags(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    gray = rescale_frame(gray, 100/scale)
    result = at_detector.detect(gray, True, (978/scale, 978/scale, 930/scale, 524/scale), 0.175)
    img_size = gray.shape
    
    wid = img_size[1]
    hei = img_size[0]
    angleStore = 0
    distanceStore = 0

    for i in result: #for each detected aprilTag. 
        if(i.tag_id == test_tag_id):#):
            cent = i.center
            Pose_R = i.pose_R
            Pose_T = i.pose_t
            # print(i)
            unofficial_tag_position = Pose_T #P @ Pose_R.T @ (-1 * Pose_T)
            # print("relative pos: ")
            #print("x: "+str(unofficial_tag_position[0]) + ", y: "+ str(unofficial_tag_position[1])+ ", z: "+ str(unofficial_tag_position[2]))
            angle = -1*int(math.atan(unofficial_tag_position[2]/unofficial_tag_position[0])*(180/math.pi))
            angle_temp = angle
            angle = int(90-abs(angle))
            if(angle_temp > 0):
                angle = angle * -1

            distance = abs(int(unofficial_tag_position[2]*100*distanceScale)) 

            angleStore += angle
            distanceStore += distance

            #print("[ angle  ]: "+str(angle))
            #print("[distance]: "+str(distance))
            cv.circle(frame,(int(cent[0]), int(cent[1])), 100, (0,0,255), -1)
            cv.putText(frame, str(i.tag_id), (int(cent[0]), int(cent[1])), font, 3, (0, 0, 0), 10, cv.LINE_4)
    if(len(result) > 0):
        angleStore = int(angleStore/len(result))
        distanceStore = int(distanceStore/len(result))
    return angleStore, distanceStore

def calcPath(grid, start_x, start_y, goal_x, goal_y):
    start = grid.node(start_x, start_y)
    end = grid.node(goal_x, goal_y)
    finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
    path, runs = finder.find_path(start, end, grid)
    print(grid.grid_str(path=path, start=start, end=end))
    return path

def calcMovesDistance(path_arr):    
    arr = path_arr
    cnt = 0
    arr_direction_y = []
    arr_direction_x = []
    arr_angle = []
    for i in arr:
        if(cnt>0):
            y_dir = i[1]-arr[cnt-1][1]
            x_dir = i[0]-arr[cnt-1][0]
            arr_direction_y.append(y_dir)
            arr_direction_x.append(x_dir)
            angle = 0
            if(y_dir == 1):
                angle = 180
            elif(y_dir == -1):
                angle = 0
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
            y_dir = i[1]-arr[cnt-1][1]
            x_dir = i[0]-arr[cnt-1][0]
            arr_direction_y.append(y_dir)
            arr_direction_x.append(x_dir)
            angle = 0
            if(y_dir == 1):
                angle = 180
            elif(y_dir == -1):
                angle = 0
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
      







headingTableX = 15
headingTableY = 10

grid_loaded = Grid(matrix=grid_raw)
arr = calcPath(grid_loaded, current_robotX, current_robotY, headingTableX, headingTableY) #y-start, x-start, y-end, x-end
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
        x.append(i[0])
for i in arr: 
        y.append(i[1])
sc.set_offsets(np.c_[x,y])
sc.axes.invert_yaxis()
fig.canvas.draw_idle()
#print(arr)

plt.waitforbuttonpress()


# start = time.time()
# while True:
#     # Capture frame-by-frame
#     try:
#         ret, frame = cap.read()
#         angle, distance = process_april_tags(frame)

#         if(movementDone == True):
#             #Code begin
#             movementDone = False
#             moveActionTimestamp = time.time()
#             print("[movement] Start")
#             #code end
#             angleStore = 0
#             distanceStore = 0
            
            
        
#         #check if movmement is done - arduino sends 0x61 on move done. 
#         readS = 0x00
#         if(test_mode == 0):
#             readS = ser.read(1)
#         if((b'\x61' == readS and movementDone == False) or time.time()-moveActionTimestamp >= moveTimeoutConst):
#                 print("[movement] Done")
#                 print()
#                 movementDone = True 
#                 time.sleep(0.1)
#         cv.imshow('frame', frame)
#         if cv.waitKey(1) == ord('q'):
#             break
#     except Exception as e: 
#         print(e)
#         break
# # When everything done, release the capture
# cap.release()
# cv.destroyAllWindows()
# if(test_mode == 0):     
#     ser.close()             # close port