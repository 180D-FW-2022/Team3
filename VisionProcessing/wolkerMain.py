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
from robotClass import Robot, RobotMotionType
from mapClass import Map, Move_type
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

moveActionTimestamp = time.time()
stepCounter = 0

position_report = 0
battery_report = 0

table_dict = dict()
table_count = 0



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


robot = Robot()
map = Map(grid_raw)
robot.setPosition_xy(map.getHomePosition()[0], map.getHomePosition()[1])
robot.matchPrevWithCurrent()
#print(fetched_map_matrix)


#fetched_map_matrix = grid_raw.copy() #remove for actual 

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
    ser_2.write(str.encode('y'))
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

# print("PLEASE REMOVE LEFT CAP and press enter")
# input()

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
sc = ax.scatter(x,y, color = 'r')
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
plt.show(block=False)
sc.axes.invert_xaxis()

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


# cv.namedWindow('disp',cv.WINDOW_NORMAL)
# cv.resizeWindow('disp',600,600)


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

    #cv.imshow('output_canvas',output_canvas)

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
        #print("[distance]: move: "+ str(distance_send))
       # bytes_val = distance_send.to_bytes(2, 'big', signed=False)
        bytes_data = struct.pack('>h', distance_send)
        serial_motor.write(str.encode(direction))
        serial_motor.write(bytes_data)
        time.sleep(0.1)

def send_angle(angle):
    if(test_mode == 0):  
        angle_send = int(angle)
        #print("[ angle  ]: move: "+str(angle_send))
        #bytes_val = angle_send.to_bytes(2, 'big', signed=True)
        bytes_data = struct.pack('>h', angle_send)
        #print(bytes_data)
        serial_motor.write(str.encode('d'))
        serial_motor.write(bytes_data)
        time.sleep(0.1)

def process_april_tags(frames, tag_id_search):
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
            if(len(result)>0 and i.tag_id == tag_id_search):#i.tag_id == test_tag_id):#):
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


def process_april_tags_vertical(frames, tag_id_search):
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
            if(len(result)>0 and i.tag_id == tag_id_search):#i.tag_id == test_tag_id):#):
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
                unofficial_tag_position[1] = -1*unofficial_tag_position[1]
                #print(f"relative angle: {Pose_R}")
               # print("x: "+str(unofficial_tag_position[0]) + ", y: "+ str(unofficial_tag_position[1])+ ", z: "+ str(unofficial_tag_position[2]))
                x_offset = unofficial_tag_position[1]
                angle = -1*int(math.atan(unofficial_tag_position[1]/unofficial_tag_position[0])*(180/math.pi))
                angle_temp = angle
                angle = int(abs(angle))
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

def angle (a, b, c):
    return math.degrees(math.acos((c**2 - b**2 - a**2)/(-2.0 * a * b)))

onlyOnce = 0
def updateDisplay(arr, updateValues = True):     
    #display
    global fetched_map_matrix, cmap_new, norm
    try:
        if(updateValues and arr != None):
            x.clear()
            y.clear()
            for i in arr: 
                x.append(i[0])
            for i in arr: 
                y.append(i[1])
        sc.set_offsets(np.c_[x,y])
        fig.canvas.draw_idle()  
        plt.pause(0.00001)   
    except:
        print("plot error")

def logInfo():
    robot.printLiveData()


rt = RepeatedTimer(1, logInfo)

headingTableX = 0
headingTableY = 0
current_step = 0
distances = []
angles = []
calibration_angle = 0
print("STARTING PROGRAM")
print()

halt_movment()
job = 0 #nothing
prev_job = 0 #used for returning to previous job from obstacle detection and april tag detection jobs. 
last_check_timestamp = 0
closest_distance_front = 200
lastPlotUpdate = 0
initialCalibCounter = 0
initialCalibStore = 0
newPosReady = False
obstacleDetectEnabled = False
while True:
   # os.system('clear')
    # Capture frame-by-frame
    try:
        if(job == -5): #calibration sequence
            countFrame = 0
            frames_top = []
            while (countFrame < 5):
                ret, frame_top = cap.read()
                if(ret):
                    countFrame+=1
                    frames_top.append(frame_top)
            x_offset, euler_rotation_x, angle, distance = process_april_tags_vertical(frames_top, tag_id_search=1)  
            print(f"X-OFFSET: {x_offset}, angle: {angle}, Distance: {distance}, Euler-Rot: {euler_rotation_x}")  
            if(x_offset > 0):
                send_distance(abs(x_offset)*100,'w')
            if(x_offset > 0):
                send_distance(abs(x_offset)*100,'b')
            cv.imshow('frame', frames_top[0]) 
            cv.waitKey(1)
            job = -4
            print(f"JOB: {job}")

        if(job == 0):#startup
            if(initialCalibCounter > 50 and newPosReady == True):
                initialCalibStore += calibration_angle
                newPosReady = False
                if(initialCalibCounter >= 60):
                    calibAngle = int(initialCalibStore/10)
                    print(f"Calibration Angle: {calibAngle}")
                    send_angle(calibAngle)
                    robot.setMotionType(RobotMotionType.ANGLE)
                    job = 1
                    print(f"JOB: {job}")
                initialCalibCounter+=1
            elif(initialCalibCounter <= 50 and newPosReady == True):
                initialCalibStore = 0
                newPosReady = False
                initialCalibCounter+=1
        elif(job == 1):
            if(robot.isMoveDone()):
                if(calibration_angle == 0):
                    robot.setRotation(180)
                    robot.setPosition_xy(map.getHomePosition()[0], map.getHomePosition()[1])
                    robot.matchPrevWithCurrent()
                    move_amount = 40-closest_distance_front
                    if(move_amount > 0):
                        send_distance(move_amount, 'g')
                    elif(move_amount < 0):
                        send_distance(move_amount)
                    robot.setMotionType(RobotMotionType.DISTANCE)
                    print("Setting Position")
                    job = 2
                    print(f"JOB: {job}")
                else:
                    initialCalibCounter = 0
                    newPosReady = False
                    initialCalibStore = 0
                    job = 0
                    print(f"JOB: {job}")
        elif(job == 2):
            if(robot.isMoveDone()):
                job = 4
                print(f"JOB: {job}")
        elif(job == 4 and time.time()-last_check_timestamp>2):
            last_check_timestamp = time.time()
            WOKerReadyTrue()
            #print(isKitchenReady())
            if(isKitchenReady() == True):
                print("Kitchen Ready")
                table_to_go_to = int(getCurrentTable())
                robotReceivedTrue()
                print(table_to_go_to)
                #print(table_num)
                if(map.getTablePosition(table_to_go_to)[0] != -1):
                    headingTableX = map.getTablePosition(table_to_go_to)[0]
                    headingTableY = map.getTablePosition(table_to_go_to)[1]
                    job = 10
                    print(f"JOB: {job}")
                else:
                    print("Table doesn't exist")
            else:
                print("No action need to be done")
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
                if(rem_dist == 0):
                    rem_dist = halt_movment()
                if(rem_dist != 0):
                    print("need update position to halted one.")
        elif(job == 9):
            #check for new data.
            job = 10
            print(f"JOB: {job}")
        elif(job == 10):
            map.setPosition(robot.get_x(), robot.get_y())
            robot.matchPrevWithCurrent()
            if(map.setTarget(headingTableX, headingTableY) > 0):
                job = 11
                print(f"JOB: {job}")
            else:
                print("Error with path - NEED JOB TO FIX")
        elif(job == 11): #move to table
            if(robot.isMoveDone()):
                moveT, amount = map.getNextMove(robot.getRotation())
                print(moveT)
                print(amount)
                if(moveT != Move_type.COMPLETE):
                    if(moveT == Move_type.ANGLE and amount != 0): #if rotate, and there is angle to rotate. 
                        send_angle(amount)
                        robot.rotate(amount) #sets robotInMotion
                    elif(moveT == Move_type.ANGLE and amount == 0):
                        robot.setMotionDone()
                    elif(moveT == Move_type.DISTANCE):
                        send_distance(amount*100)
                        robot.move(amount) #sets robotInMotion
                    
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
            map.setPosition(robot.get_x(), robot.get_y())
            robot.matchPrevWithCurrent()
            if(map.setTarget(map.getHomePosition()[0], map.getHomePosition()[1]) > 0):
                job = 14
                print(f"JOB: {job}")
            else:
                print("Error with path - NEED JOB TO FIX")
        elif(job == 14): #move home
            if(robot.isMoveDone()):
                moveT, amount = map.getNextMove(robot.getRotation())
                print(moveT)
                print(amount)
                if(moveT != Move_type.COMPLETE):
                    if(moveT == Move_type.ANGLE and amount != 0): #if rotate, and there is angle to rotate. 
                        send_angle(amount)
                        robot.rotate(amount) #sets robotInMotion
                    elif(moveT == Move_type.ANGLE and amount == 0):
                        robot.setMotionDone()
                    elif(moveT == Move_type.DISTANCE):
                        send_distance(amount*100)
                        robot.move(amount) #sets robotInMotion
                    
                    moveActionTimestamp = time.time()
                    print("[movement] Start")
                else:
                    job = 15
                    print(f"JOB: {job}")
        elif(job == 15):
            if(robot.isMoveDone()):
                if(robot.getRotation() != 0):
                    send_angle(-1*robot.getRotation())
                    robot.setRotation(0)
                    robot.setMotionType(RobotMotionType.ANGLE)
                    moveActionTimestamp = time.time()
                    print("[movement] Start")
                    job = 16
                else:
                    job = 4
                print(f"JOB: {job}")
        elif(job == 16):
            if(robot.isMoveDone()):
                robot.setPosition_xy(map.getHomePosition()[0], map.getHomePosition()[1])
                robot.matchPrevWithCurrent()
                job = 4
                print(f"JOB: {job}")
        elif(job == 17):
            #move back to grid
            dist_to_rev = 0
            rounded_x = robot.get_x()
            rounded_y = robot.get_y()
            if(robot.getRotation() == 0):
                rounded_y = float(math.floor(robot.get_y() * 2)) / 2.0
                y_offset = abs(robot.get_y() - rounded_y)
                print(f"y_off: {y_offset}")
                dist_to_rev = y_offset
            elif(robot.getRotation() == 90):
                rounded_x = float(math.ceil(robot.get_x() * 2)) / 2.0
                x_offset = abs(robot.get_x() - rounded_x)
                print(f"x_off: {x_offset}")
                dist_to_rev = x_offset
            elif(robot.getRotation() == -90):
                rounded_x = float(math.floor(robot.get_x() * 2)) / 2.0
                x_offset = abs(robot.get_x() - rounded_x)
                print(f"x_off: {x_offset}")
                dist_to_rev = x_offset
            elif(robot.getRotation() == 180):
                rounded_y = float(math.ceil(robot.get_y() * 2)) / 2.0
                y_offset = abs(robot.get_y() - rounded_y)
                print(f"y_off: {y_offset}")
                dist_to_rev = y_offset
            if(dist_to_rev > 0.01):
                send_distance(dist_to_rev*100, 'g')
                robot.setRotation(robot.getRotation()+180) #workaround reverse
                robot.move(dist_to_rev)
                robot.setPosition_xy(rounded_x, rounded_y)
                job = 18
            else:
                if(prev_job == 11):
                    job = 10
                elif(prev_job == 14):
                    job = 13
            print(f"JOB: {job}")
        elif(job == 18):
            if(robot.isMoveDone()):
                robot.setRotation(robot.getRotation()+180) #workaround reverse undo.
                if(prev_job == 11):
                    job = 10
                elif(prev_job == 14):
                    job = 13
                print(f"JOB: {job}")
                
        
        if(robot.getMotion() == RobotMotionType.DISTANCE and closest_distance_front < 18 and not robot.getInObstacleStateBool()): ##TODO FIX
            prev_job = job
            job = 1000 #no job
            if(halt_movment() != 1):
                print("Inital halt failed sending again...")
                halt_movment()
            print("EMERGENCY HALT - will wait till obstacle clear")
            robot.setObstacle()
            robot.setMotionDone()
            robot.setCurrentPositionToInMotion()
            robot.setLeg(0)
        elif(robot.getInObstacleStateBool() and closest_distance_front > 22): #obstacle cleared. 
            robot.setObstacleClear()
            job = 17
        
            
            
        
        #check if movmement is done - arduino sends 0x61 on move done. 
        readS = 0x00
        if(test_mode == 0 and serial_motor.in_waiting > 0):
            readS = serial_motor.read(1)
          #  print(readS)
        if((time.time()-moveActionTimestamp >= moveTimeoutConst and not robot.isMoveDone())):
            print("[movement] Timeout")
        if((b'\x61' == readS and not robot.isMoveDone()) or (time.time()-moveActionTimestamp >= moveTimeoutConst and not robot.isMoveDone())):
            print("[movement] Done")
            robot.matchPrevWithCurrent()
            print()
            robot.setMotionDone()
            moveActionTimestamp = time.time()
            time.sleep(0.1)
        elif(b'\x70' == readS):#logging of current position, battery voltage. 
            readS = 0x00
            readBat = serial_motor.read(2)
            readPos = serial_motor.read(2)
            position_report = int.from_bytes(readPos, "little")/100 #m
            battery_report = int.from_bytes(readBat, "little")*10 #mV
            if(position_report != -1 and robot.getMotion() == RobotMotionType.DISTANCE):
                robot.setLeg(position_report)
            elif(not robot.getInObstacleStateBool()):
                robot.setLeg(0)
            robot.setBatteryVoltage(battery_report)

        if(test_mode == 0 and serial_ultrasonic.in_waiting > 0):
            readUltra = serial_ultrasonic.readline().decode()
            readUltra = readUltra.strip().split(';')[0:-1]
            tempArr = []
            for element in readUltra:
                temp_dist = int(element.split(',')[1])
                if(temp_dist == 0):
                    temp_dist = 200
                tempArr.append((temp_dist))
            distances.append(tempArr)
            if(len(distances) > 5):
                distances.pop(0)
            closest_distance_front = min(np.mean(distances, axis=0))
            diff_dist = (np.mean(distances, axis=0)[0]-np.mean(distances, axis=0)[1])
            calibration_angle = int(math.atan(diff_dist/25)*(180/math.pi))
            newPosReady = True
        # if(len(map.pathForPlotting) > 0):
        #     updateDisplay([robot.getCurrentPositionInMotionXY()])
        if(time.time() - lastPlotUpdate > 0.2):
            lastPlotUpdate = time.time()
            updateDisplay([robot.getCurrentPositionInMotionXYIndex()], robot.getMotion() == RobotMotionType.DISTANCE)
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