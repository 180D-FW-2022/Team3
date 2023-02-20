from pupil_apriltags import Detector
import cv2 as cv

import os

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

from scipy.spatial.transform import Rotation as R

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.heuristic import null

# Import the Firebase service
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

#custom classes improt
from robotClass import Robot
from mapClass import Map
from timerClass import RepeatedTimer
#end


test_mode = 0
#global consts
test_tag_id = 0
distanceScale = 1.1

moveTimeoutConst = 40 #seconds

cam_id = 2 #"/dev/video0"
cam_id_bottom = 1

scale = 1.0

font = cv.FONT_HERSHEY_SIMPLEX

movementDone = True
moveActionTimestamp = time.time()
stepCounter = 0

position_report = 0
battery_report = 0

table_dict = dict()
table_count = 0
home_coords = np.array([0,0]).astype(int)



cred = credentials.Certificate("firebase_key.json")
default_app = firebase_admin.initialize_app(cred, {'databaseURL':"https://d-database-c824d-default-rtdb.firebaseio.com"})
ref = db.reference()

def __wipeFirebase():
    nodes = ref.get()
    ref.delete()    

def getCurrentTable():
    return (ref.child("newTableNumber").get())

def isKitchenReady(): #True when current table ready. Becomes false when woker ready set true 
    return (ref.child("newTableReady").get())

#set WOKer to ready (True)
def WOKerReadyTrue():
    ref.child("WOKerReady").set(True)

#set robotReceived to recieved (True)
def robotReceivedTrue():
    ref.child("robotReceived").set(True)

def getMap():
    return ref.child("map").get()


matrix_size = 20

fetched_map = getMap().split(',')
fetched_map.reverse()
fetched_map_matrix = np.reshape(fetched_map, (matrix_size, matrix_size)).astype(int)
grid_raw = fetched_map_matrix.astype(int).copy()

robot = Robot()
map = Map(grid_raw)
#print(fetched_map_matrix)

# grid_raw = np.array([
#             [4, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 0, 0, 11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],    
#             ]).astype(int)
# fetched_map_matrix = grid_raw.copy()

#print(grid_raw)
print((grid_raw).shape)
#grid_display = fetched_map_matrix.astype(int)
grid_raw[grid_raw > 9] = -1
grid_raw[grid_raw == 4] = -1
grid_raw[grid_raw > 0] = -2
grid_raw[grid_raw == 0] = 1
grid_raw[grid_raw == -1] = 1
grid_raw[grid_raw == -2] = 0

grid_raw_margined = grid_raw.copy()
print(grid_raw_margined)
# grid_loaded = Grid(matrix=grid_raw_margined)


for row in range(matrix_size):
    for column in range(matrix_size):
        if(fetched_map_matrix[row][column]>=10):
            table_num = fetched_map_matrix[row][column] - 10
            if (table_num) not in table_dict:
                table_dict[table_num] = [column*0.5, row*0.5]
                print(f"Table {table_num} found {table_dict[table_num]}")
                table_count+=1
        if(fetched_map_matrix[row][column] == 4):
            home_coords[0] = column*0.5
            home_coords[1] = row*0.5
            robot.setPosition_xy(home_coords[0], home_coords[1])
            robot.matchPrevWithCurrent()
            print(f"Home found: x: {home_coords[0]}, y: {home_coords[1]}")



to_set_const = 4 #weight of margin entries
populate = 1
if populate == 1:
    for row in range(matrix_size): #populate margins of 0.5m or 1 block around items for path planning. 
        for column in range(matrix_size):
            current = grid_raw[row][column]
        # left = grid_raw[row][column-1]
        # right = grid_raw[row][column+1]
        # up = grid_raw[row-1][column]
        # down = grid_raw[row+1][column]
            if(row > 0 and row<(matrix_size-1) and column > 0 and column<(matrix_size-1)):
                if(current == 1):
                    left = grid_raw[row][column-1]
                    right = grid_raw[row][column+1]
                    up = grid_raw[row-1][column]
                    down = grid_raw[row+1][column]
                    d1 = grid_raw[row-1][column-1] #up left diagonal
                    d2 = grid_raw[row-1][column+1] #up right diag
                    d3 = grid_raw[row+1][column-1] #down left diag
                    d4 = grid_raw[row+1][column+1] #down right diag
                    if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                        grid_raw_margined[row][column] = to_set_const
            if(row == 0 and column > 0 and column<(matrix_size-1)): #top 
                if(current == 1):
                    left = grid_raw[row][column-1]
                    right = grid_raw[row][column+1]
                    up = 1
                    down = grid_raw[row+1][column]
                    d1 = 1 #up left diagonal
                    d2 = 1 #up right diag
                    d3 = grid_raw[row+1][column-1] #down left diag
                    d4 = grid_raw[row+1][column+1] #down right diag
                    if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                        grid_raw_margined[row][column] = to_set_const
            if(row == (matrix_size-1) and column > 0 and column<(matrix_size-1)): #bottom
                if(current == 1):
                    left = grid_raw[row][column-1]
                    right = grid_raw[row][column+1]
                    up = grid_raw[row-1][column]
                    down = 1
                    d1 = grid_raw[row-1][column-1] #up left diagonal
                    d2 = grid_raw[row-1][column+1] #up right diag
                    d3 = 1 #down left diag
                    d4 = 1 #down right diag
                    if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                        grid_raw_margined[row][column] = to_set_const
            if(row > 0 and row<(matrix_size-1) and column == 0): #left
                if(current == 1):
                    left = 1
                    right = grid_raw[row][column+1]
                    up = grid_raw[row-1][column]
                    down = grid_raw[row+1][column]
                    d1 = 1 #up left diagonal
                    d2 = grid_raw[row-1][column+1] #up right diag
                    d3 = 1 #down left diag
                    d4 = grid_raw[row+1][column+1] #down right diag
                    if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                        grid_raw_margined[row][column] = to_set_const
            if(row > 0 and row<(matrix_size-1) and column==(matrix_size-1)): #right
                if(current == 1):
                    left = grid_raw[row][column-1]
                    right = 1
                    up = grid_raw[row-1][column]
                    down = grid_raw[row+1][column]
                    d1 = grid_raw[row-1][column-1] #up left diagonal
                    d2 = 1 #up right diag
                    d3 = grid_raw[row+1][column-1] #down left diag
                    d4 = 1 #down right diag
                    if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                        grid_raw_margined[row][column] = to_set_const
            if(row == 0 and column == 0): #top left
                if(current == 1):
                    left = 1
                    right = grid_raw[row][column+1]
                    up = 1
                    down = grid_raw[row+1][column]
                    d1 = 1 #up left diagonal
                    d2 = 1 #up right diag
                    d3 = 1 #down left diag
                    d4 = grid_raw[row+1][column+1] #down right diag
                    if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                        grid_raw_margined[row][column] = to_set_const
            if(row == (matrix_size-1) and column == 0): #bottom left
                if(current == 1):
                    left = 1
                    right = grid_raw[row][column+1]
                    up = grid_raw[row-1][column]
                    down = 1
                    d1 = 1 #up left diagonal
                    d2 = grid_raw[row-1][column+1] #up right diag
                    d3 = 1 #down left diag
                    d4 = 1 #down right diag
                    if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                        grid_raw_margined[row][column] = to_set_const
            if(row == (matrix_size-1) and column == (matrix_size-1)): #bottom right
                if(current == 1):
                    left = grid_raw[row][column-1]
                    right = 1
                    up = grid_raw[row-1][column]
                    down = 1
                    d1 = grid_raw[row-1][column-1] #up left diagonal
                    d2 = 1 #up right diag
                    d3 = 1 #down left diag
                    d4 = 1 #down right diag
                    if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                        grid_raw_margined[row][column] = to_set_const
            if(row == 0 and column == (matrix_size-1)): #top right
                if(current == 1):
                    left = grid_raw[row][column-1]
                    right = 1
                    up = 1
                    down = grid_raw[row+1][column]
                    d1 = 1 #up left diagonal
                    d2 = 1 #up right diag
                    d3 = grid_raw[row+1][column-1] #down left diag
                    d4 = 1 #down right diag
                    if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                        grid_raw_margined[row][column] = to_set_const

serial_ultrasonic = 0
serial_motor = 0
#arduino connect pySerial
if(test_mode == 0):
    arduino_ports = [
        p.device
        for p in serial.tools.list_ports.comports()
        if (('Arduino' in p.description or 'USB Serial' in p.description) and 'wch' not in p.device)  # may need tweaking to match new arduinos
    ]
    if not arduino_ports:
        print("[arduino ] Not found")
        exit(0)
    if len(arduino_ports) != 2:
        print('[arduino ] Incorrect # found')
        exit(0)
    print(arduino_ports[0])
    print(arduino_ports[1])
    ser_1 = serial.Serial(arduino_ports[0], baudrate = 115200, timeout = 0.1)
    ser_2 = serial.Serial(arduino_ports[1], baudrate = 115200, timeout = 0.1)
    time.sleep(2)
    arduino_init_timeout = 0
    ser_1.write(str.encode('y'))
    while(arduino_init_timeout < 15):
        arduino_init_timeout += 1
        read_data = ser_1.read(1).decode()
        if(read_data == 'u'):
            print("[arduino ] Not Swapping Serial Ports")
            serial_ultrasonic = ser_1
            serial_motor = ser_2
            arduino_init_timeout = -1
            break
        elif(read_data == 'm'):
            print("[arduino ] Swapping Serial Ports")
            serial_ultrasonic = ser_2
            serial_motor = ser_1
            arduino_init_timeout = -1
            break
    if(arduino_init_timeout != -1):
        print("[arduino ] Error identifying, please make sure currect program running on Arduinos")
        exit(0)

#video init.
index = 0
arr = []
while True:
    cap = cv.VideoCapture(index)
    if not cap.read()[0]:
        break
    else:
        ret_test, frame_test = cap.read()
        print(f"Frame x:{frame_test.shape[0]}, y: {frame_test.shape[1]}")
        if(int(frame_test.shape[0]) == 1080):
            arr.append(index)
    cap.release()
    index += 1
print(arr)
if(len(arr) != 2):
    print("[ camera ] Connect both cameras")
    exit(0)
cam_id = arr[0]
cam_id_bottom = arr[1]
cap = cv.VideoCapture(cam_id)
if not cap.isOpened():
    print("[ camera ] Cannot open top")
    exit()
cap_bottom = cv.VideoCapture(cam_id_bottom)
if not cap_bottom.isOpened():
    print("[ camera ] Cannot open bottom")
    exit()

ret_start, frame_start = cap.read()
average = frame_start.mean(axis=0).mean(axis=0)
if(average[0] < 5.0):
    print("[ camera ] Flipping right and left")
    cap.release()
    cap_bottom.release()
    temp = cam_id_bottom
    cam_id_bottom = cam_id
    cam_id = temp

print("PLEASE REMOVE LEFT CAP and press enter")
input()

cap = cv.VideoCapture(cam_id)
if not cap.isOpened():
    print("[ camera ] Cannot open top")
    exit()
cap_bottom = cv.VideoCapture(cam_id_bottom)
if not cap_bottom.isOpened():
    print("[ camera ] Cannot open bottom")
    exit()
#video init end

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
cmap = plt.cm.jet  # define the colormap
cmaplist = [cmap(i) for i in range(cmap.N)]
cmaplist[0] = (1, 1, 1, 1.0) #allowed spaces
cmap_new = mpl.colors.LinearSegmentedColormap.from_list(
    'Custom cmap', cmaplist)
bounds = np.linspace(-1, 20, 21)
norm = mpl.colors.BoundaryNorm(bounds, cmap_new.N)

plt.imshow(fetched_map_matrix, cmap=cmap_new, norm=norm)
plt.colorbar(plt.cm.ScalarMappable(cmap=cmap_new, norm=norm))
plt.xlim(-1,21)
plt.ylim(-1,21)
plt.draw()
#end



global stereo, output_canvas, depth_map
#stereo init
print("Settng parameters Single ......")
DIM=(1920, 1080)
K=np.array([[1053.9492767154009, 0.0, 951.950093568802], [0.0, 1052.5528725501529, 465.04595064900246], [0.0, 0.0, 1.0]])
D=np.array([[-0.05334899174471995], [0.004050987506634966], [-0.001763065658573223], [-0.0027162184694266523]])
map1, map2 = cv.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv.CV_16SC2)

print("Reading parameters Stereo ......")
cv_file = cv.FileStorage("./data/params_py.xml", cv.FILE_STORAGE_READ)

Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
cv_file.release()

disparity = None
depth_map = None

# These parameters can vary according to the setup
max_depth = 400 # maximum distance the setup can measure (in cm)
min_depth = 0 # minimum distance the setup can measure (in cm)
depth_thresh = 200.0 # Threshold for SAFE distance (in cm)

# Reading the stored the StereoBM parameters

cv_file = cv.FileStorage("./depthEst.xml", cv.FILE_STORAGE_READ)
numDisparities = int(cv_file.getNode("numDisparities").real())
blockSize = int(cv_file.getNode("blockSize").real())
preFilterType = int(cv_file.getNode("preFilterType").real())
preFilterSize = int(cv_file.getNode("preFilterSize").real())
preFilterCap = int(cv_file.getNode("preFilterCap").real())
textureThreshold = int(cv_file.getNode("textureThreshold").real())
uniquenessRatio = int(cv_file.getNode("uniquenessRatio").real())
speckleRange = int(cv_file.getNode("speckleRange").real())
speckleWindowSize = int(cv_file.getNode("speckleWindowSize").real())
disp12MaxDiff = int(cv_file.getNode("disp12MaxDiff").real())
minDisparity = int(cv_file.getNode("minDisparity").real())
M = cv_file.getNode("M").real()
cv_file.release()


cv.namedWindow('disp',cv.WINDOW_NORMAL)
cv.resizeWindow('disp',600,600)


output_canvas = None

stereo = cv.StereoBM_create()
#functions

#stereo

def obstacle_avoid():

    # Mask to segment regions with depth less than threshold
    mask = cv.inRange(depth_map,10,depth_thresh)

    # Check if a significantly large obstacle is present and filter out smaller noisy regions
    if np.sum(mask)/255.0 > 0.001*mask.shape[0]*mask.shape[1]:

        # Contour detection 
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cnts = sorted(contours, key=cv.contourArea, reverse=True)
        
        # Check if detected contour is significantly large (to avoid multiple tiny regions)
        if True:#cv.contourArea(cnts[0]) > 0.01*mask.shape[0]*mask.shape[1]:

            x,y,w,h = cv.boundingRect(cnts[0])

            # finding average depth of region represented by the largest contour 
            mask2 = np.zeros_like(mask)
            cv.drawContours(mask2, cnts, 0, (255), -1)

            # Calculating the average depth of the object closer than the safe distance
            depth_mean, _ = cv.meanStdDev(depth_map, mask=mask2)
            return float(depth_mean)

    else:
      #  cv.putText(output_canvas, "SAFE!", (100,100),1,3,(0,255,0),2,3)
        return -1

    cv.imshow('output_canvas',output_canvas)

def processStereo(imgR, imgL):
    global stereo, output_canvas, depth_map
    output_canvas = imgL.copy()

    imgR_gray = cv.cvtColor(imgR,cv.COLOR_BGR2GRAY)
    imgL_gray = cv.cvtColor(imgL,cv.COLOR_BGR2GRAY)
    #map1, map2 = cv.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv.CV_16SC2)
    Left_nice = cv.remap(imgL, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)
    Right_nice = cv.remap(imgR, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)

    Left_nice= cv.remap(Left_nice,Left_Stereo_Map_x,Left_Stereo_Map_y, cv.INTER_LANCZOS4, cv.BORDER_CONSTANT, 0)
    Right_nice= cv.remap(Right_nice,Right_Stereo_Map_x,Right_Stereo_Map_y, cv.INTER_LANCZOS4, cv.BORDER_CONSTANT, 0)
    Left_nice = cv.cvtColor(Left_nice,cv.COLOR_BGR2GRAY)
    Right_nice = cv.cvtColor(Right_nice,cv.COLOR_BGR2GRAY)
    Left_nice = cv.resize(Left_nice, (600, 500))
    Right_nice = cv.resize(Right_nice, (600, 500))
    # Setting the updated parameters before computing disparity map
    stereo.setNumDisparities(numDisparities)
    stereo.setBlockSize(blockSize)
    stereo.setPreFilterType(preFilterType)
    stereo.setPreFilterSize(preFilterSize)
    stereo.setPreFilterCap(preFilterCap)
    stereo.setTextureThreshold(textureThreshold)
    stereo.setUniquenessRatio(uniquenessRatio)
    stereo.setSpeckleRange(speckleRange)
    stereo.setSpeckleWindowSize(speckleWindowSize)
    stereo.setDisp12MaxDiff(disp12MaxDiff)
    stereo.setMinDisparity(minDisparity)

    # Calculating disparity using the StereoBM algorithm
    disparity = stereo.compute(Left_nice,Right_nice)
    # NOTE: compute returns a 16bit signed single channel image,
    # CV_16S containing a disparity map scaled by 16. Hence it 
    # is essential to convert it to CV_16S and scale it down 16 times.

    # Converting to float32 
    disparity = disparity.astype(np.float32)

    # Normalizing the disparity map
    disparity = (disparity/16.0 - minDisparity)/numDisparities

    depth_map = M/(disparity) # for depth in (cm)

    mask_temp = cv.inRange(depth_map,min_depth,max_depth)
    depth_map = cv.bitwise_and(depth_map,depth_map,mask=mask_temp)

    depth_value = obstacle_avoid()
    
   # cv.resizeWindow("disp",700,700)
   # cv.imshow("disp",disparity)
    
    return depth_value
#end


def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv.resize(frame, dim, interpolation =cv.INTER_AREA)


def halt_movment():
    serial_motor.write(str.encode('x'))
    if(serial_motor.read(1) == b'\x73'):
        return 1
    return 0

def send_distance(distance, direction = 'r'):
    if(test_mode == 0):  
        distance_send = int(abs(distance))
        print("[distance]: move: "+ str(distance_send))
       # bytes_val = distance_send.to_bytes(2, 'big', signed=False)
        bytes_data = struct.pack('>h', distance_send)
        
        serial_motor.write(str.encode(direction))
        serial_motor.write(bytes_data)
        # if((upper_byte+lower_byte) == ser.read(2)):
        #     print("[distance]: received")
        # else:
        #     # ser.write(b'\x00') 
        #     # ser.write(b'\x00') 
        #     # ser.write(b'\x00') 

        # # movementDone = True 
        #     time.sleep(0.1)
        time.sleep(0.1)

def send_angle(angle):
    if(test_mode == 0):  
        angle_send = int(angle)
        print("[ angle  ]: move: "+str(angle_send))
        #bytes_val = angle_send.to_bytes(2, 'big', signed=True)
        bytes_data = struct.pack('>h', angle_send)
        print(bytes_data)
        serial_motor.write(str.encode('d'))
        serial_motor.write(bytes_data)
        # if(bytes_data == ser.read(2)):
        #     print("[ angle  ]: received")
        # else:
        #     # ser.write(b'\x00') 
        #     # ser.write(b'\x00') 
        #     # ser.write(b'\x00') 
        # # movementDone = True
        #     time.sleep(0.1)
        time.sleep(0.1)

def process_april_tags(frames):
    angleStore = 0
    xStore = 0
    eulerRotx = 0
    distanceStore = 0
    total_results = 0
    for frame in frames: #for each detected aprilTag. 
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        gray = rescale_frame(gray, 100/scale)
        result = at_detector.detect(gray, True, (978/scale, 978/scale, 930/scale, 524/scale), 0.171)
        img_size = gray.shape

        wid = img_size[1]
        hei = img_size[0]
        for i in result:
            if(len(result)>0 and i.tag_id == 0):#i.tag_id == test_tag_id):#):
                total_results += 1
                x_offset = 0
                cent = i.center
                Pose_R = i.pose_R
                Pose_T = i.pose_t
                rot_matrix = R.from_matrix(Pose_R)
                eulers = rot_matrix.as_euler('zxy', degrees=True)
                #print("Euler Angles: ")
                #print(eulers)
                eulerRotx += eulers[1]
                # print(i)
                unofficial_tag_position = Pose_T #P @ Pose_R.T @ (-1 * Pose_T)
                #print(f"relative angle: {Pose_R}")
               # print("x: "+str(unofficial_tag_position[0]) + ", y: "+ str(unofficial_tag_position[1])+ ", z: "+ str(unofficial_tag_position[2]))
                x_offset = unofficial_tag_position[0]
                angle = -1*int(math.atan(unofficial_tag_position[2]/unofficial_tag_position[0])*(180/math.pi))
                angle_temp = angle
                angle = int(90-abs(angle))
                if(angle_temp > 0):
                    angle = angle * -1

                distance = abs(int(unofficial_tag_position[2]*100*distanceScale)) 

                angleStore += angle
                distanceStore += distance
                xStore += x_offset

                #print("[ angle  ]: "+str(angle))
                #print("[distance]: "+str(distance))
                cv.circle(frame,(int(cent[0]), int(cent[1])), 50, (0,0,255), -1)
                cv.putText(frame, str(i.tag_id), (int(cent[0]), int(cent[1])), font, 3, (0, 0, 0), 8, cv.LINE_4)
    if(total_results > 0):
        angleStore = int(angleStore/total_results)
        distanceStore = int(distanceStore/total_results)
        xStore = float(xStore/total_results)
        eulerRotx = int(eulerRotx/total_results)
    return xStore, eulerRotx, angleStore, distanceStore

def calcPath(grid, start_x, start_y, goal_x, goal_y):
    start = grid.node(start_x, start_y)
    end = grid.node(goal_x, goal_y)
    finder = AStarFinder(diagonal_movement=DiagonalMovement.never,  heuristic=null)
    path, runs = finder.find_path(start, end, grid)
   #print(grid.grid_str(path=path, start=start, end=end))
    #print(path)
    return path

def calcPathRWU(grid, start_x, start_y, goal_x, goal_y):
   # print(f"startx: {start_x}, stary: {start_y}, goalx: {goal_x}, goaly: {goal_y}")
    return calcPath(grid, int(2*start_x), int(2*start_y), int(2*goal_x), int(2*goal_y))

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
                angle = 0
            elif(y_dir == -1):
                angle = 180
            elif(x_dir == 1):
                angle = -90
            elif(x_dir == -1):
                angle = 90
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
                angle = 0
            elif(y_dir == -1):
                angle = 180
            elif(x_dir == 1):
                angle = -90
            elif(x_dir == -1):
                angle = 90
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

def calcLeastAngle(angle):
    to_ret = angle
    if(angle > 360):
         to_ret = angle%360
    if(angle < -360):
        to_ret = -1*(abs(angle)%360)
    if(angle == 270):
        to_ret = -90
    if(angle == -270):
        to_ret = 90
    return int(to_ret)

def updateDisplay(arr):     
    #display
    x.clear()
    y.clear()
    for i in arr: 
        x.append(i[0])
    for i in arr: 
        y.append(i[1])
    sc.set_offsets(np.c_[x,y])
    #sc.axes.invert_yaxis()
    sc.axes.invert_xaxis()
    fig.canvas.draw_idle()     
    plt.pause(1) 


def logInfo():
    robot.printLiveData()

rt = RepeatedTimer(1, logInfo)

headingTableX = 2
headingTableY = 2
current_step = 0
distances = []
angles = []
print("STARTING PROGRAM")
print()



halt_movment()
job = 0 #nothing
prev_job = 0 #used for returning to previous job from obstacle detection and april tag detection jobs. 
while True:
   # os.system('clear')
    # Capture frame-by-frame
    try:
        if(job == 0):
            WOKerReadyTrue()
            #print(isKitchenReady())
            if(isKitchenReady() == True):
                print("Kitchen Ready")
                table_to_go_to = int(getCurrentTable())
                robotReceivedTrue()
                print(table_to_go_to)
                #print(table_num)
                if(table_to_go_to <= table_count):
                    headingTableX = table_dict[table_to_go_to][0]
                    headingTableY = table_dict[table_to_go_to][1]
                    job = 10
                    print(f"JOB: {job}")
                else:
                    print("Table doesn't exist")
            else:
                print("No action need to be done")
                time.sleep(2)
            headingTableX = table_dict[0][0] #testing
            headingTableY = table_dict[0][1] #testing
            job = 10
        elif(job == 5):#check april location
            countFrame = 0
            frames_top = []
            while (countFrame < 5):
                ret, frame_top = cap.read()
                if(ret):
                    countFrame+=1
                    frames_top.append(frame_top)
            x_offset, euler_rotation_x, angle, distance = process_april_tags(frames_top)  
            print(f"X-OFFSET: {x_offset}, angle: {angle}, Distance: {distance}, Euler-Rot: {euler_rotation_x}")  
            #cv.imshow('frame', frames_top[0]) 
            #cv.waitKey(1)
        elif(job  == 6):
            distance_to_closest = 0
            procseedStero = 0
            runCount = 0
            while(runCount<2):
                ret, frame_top = cap.read()
                ret_b, frame_bottom = cap_bottom.read()
                if ret and ret_b:
                    runCount+=1
                    imgForStereo = frame_top
                    imgBotForStereo = frame_bottom
                    procseedStero = processStereo(frame_top, frame_bottom)
                    distance_to_closest+=procseedStero
                    print(f"Stereo Returned Object: {procseedStero}")
            distance_to_closest = distance_to_closest/2 #cm
            if(distance_to_closest < 90): #make sure to only call in linear movment. 
                print(f"Obstacle @: {distance_to_closest}")
                rem_dist = halt_movment()
                if(rem_dist == -1):
                    rem_dist = halt_movment()
                if(rem_dist != -1):
                    print("need update position to halted one.")
        elif(job == 9):
            #check for new data.
            print(table_dict[0])
            job = 10
            print(f"JOB: {job}")
        elif(job == 10):
            grid_loaded = Grid(matrix=grid_raw_margined)
            arr = calcPathRWU(grid_loaded, robot.get_x(), robot.get_y(), headingTableX, headingTableY) #y-start, x-start, y-end, x-end
            map.setPosition(robot.get_x(), robot.get_y())
            if(map.setTarget(headingTableX, headingTableY) > 0):
                updateDisplay(arr)
                distances = calcMovesDistance(arr)
                angles = calcMovesHeading(arr)                
                current_step = 0
                job = 11
                print(f"JOB: {job}")
            else:
                print("Error with path - NEED JOB TO FIX")
        elif(job == 11): #move to table
            if(movementDone == True):
                print(map.getNextMove())
                if(current_step < len(angles)):
                    i=current_step
                    if(angles[i]-robot.getRotation() == 0):
                        print("moving "+str(distances[i])+"m")
                        send_distance(distances[i]*100)
                        robot.move(distance=distances[i])
                        #job = 6
                        current_step += 1
                    else:
                        angle_to_rot = angles[i]-robot.getRotation()
                        print("rotating "+str(angle_to_rot)+"°")
                        send_angle(angle_to_rot)
                        robot.rotate(angle_to_rot)
                    movementDone = False
                    moveActionTimestamp = time.time()
                    print("[movement] Start")
                else:
                    job = 12
                    print(f"JOB: {job}")
        elif(job == 12):
            time.sleep(5) #wait at table 5s
            job = 13
            print(f"JOB: {job}")
        elif(job == 13): #return home
            headingTableX = home_coords[0]
            headingTableY = home_coords[1] #setNew data.
            grid_loaded = Grid(matrix=grid_raw_margined)
            arr = calcPathRWU(grid_loaded, robot.get_x(), robot.get_y(), headingTableX, headingTableY) #y-start, x-start, y-end, x-end
            if(len(arr) > 0):
                updateDisplay(arr)
                distances = calcMovesDistance(arr)
                angles = calcMovesHeading(arr)
                current_step = 0
                job = 14
                print(f"JOB: {job}")
            else:
                print("Error with path - NEED JOB TO FIX")
        elif(job == 14): #move home
            if(movementDone == True):
                if(current_step < len(angles)):
                    i=current_step
                    if(angles[i]-robot.getRotation() == 0):
                        print("moving "+str(distances[i])+"m")
                        send_distance(distances[i]*100)
                        robot.move(distances[i])
                        current_step += 1
                    else:
                        angle_to_rot = angles[i]-robot.getRotation()
                        print("rotating "+str(angle_to_rot)+"°")
                        send_angle(angle_to_rot)
                        robot.rotate(angle_to_rot)
                    movementDone = False
                    moveActionTimestamp = time.time()
                    print("[movement] Start")
                else:
                    job = 15
                    print(f"JOB: {job}")
        elif(job == 15):
            if(movementDone == True):
                send_angle(-1*robot.getRotation())
                robot.setRotation(0)
                movementDone = False
                moveActionTimestamp = time.time()
                print("[movement] Start")
                job = 16
        elif(job == 16):
            if(movementDone == True):
                robot.setPosition_xy(home_coords[0], home_coords[1])
                robot.matchPrevWithCurrent()
                job = 0
        else:
            job = 0 #reset
            print(f"JOB: {job}")
    
            
        
        #check if movmement is done - arduino sends 0x61 on move done. 
        readS = 0x00
        if(test_mode == 0 and serial_motor.inWaiting() > 0):
            readS = serial_motor.read(1)
          #  print(readS)
        if((time.time()-moveActionTimestamp >= moveTimeoutConst and movementDone == False)):
            print("[WARNING] TIMEOUT")
        if((b'\x61' == readS and movementDone == False) or (time.time()-moveActionTimestamp >= moveTimeoutConst and movementDone == False)):
            print("[movement] Done")
            print()
            movementDone = True 
            moveActionTimestamp = time.time()
            time.sleep(0.1)
        elif(b'\x70' == readS):#logging of current position, battery voltage. 
            readS = 0x00
            readBat = serial_motor.read(2)
            readPos = serial_motor.read(2)
            position_report = int.from_bytes(readPos, "little")/100 #m
            battery_report = int.from_bytes(readBat, "little")*10 #mV
            print(position_report)
            robot.setLeg(position_report)
            robot.setBatteryVoltage(battery_report)
        if cv.waitKey(1) == ord('q'):
            break
    except Exception as e: 
        print("Error"+str(e))
        break
# When everything done, release the capture
cap.release()
cap_bottom.release()
cv.destroyAllWindows()
plt.waitforbuttonpress()
if(test_mode == 0):     
    serial_motor.close()             # close port
    serial_ultrasonic.close()