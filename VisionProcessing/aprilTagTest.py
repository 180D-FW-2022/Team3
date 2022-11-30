from pupil_apriltags import Detector

import numpy as np
import cv2 as cv
import time
import math 

import warnings
import serial
import serial.tools.list_ports
import time

#global consts
test_tag_id = 1
distanceScale = 0.8

cam_id_L = "/dev/video0"
scale = 1.0

font = cv.FONT_HERSHEY_SIMPLEX

#global vars
movementDone = True
hasRotated = False

#functions
def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv.resize(frame, dim, interpolation =cv.INTER_AREA)

def send_distance(distance, direction = 'b'):
    print("[distance]: move - "+ str(distance))
    distance_send = distance
    bytes_val = distance_send.to_bytes(2, 'big', signed=True)
    ser.write(str.encode(direction))
    ser.write(bytes_val)
    if(bytes_val == ser.read(2)):
        print("[distance]: received")
    else:
        ser.write(b'\x00') 
        ser.write(b'\x00') 
        ser.write(b'\x00') 
        movementDone = True 
        time.sleep(0.1)

def send_angle(angle):
    print("[ angle  ]: move - "+str(angle))
    bytes_val = angle.to_bytes(2, 'big', signed=True)
    ser.write(str.encode('d'))
    ser.write(bytes_val)
    if(bytes_val == ser.read(2)):
        print("[ angle  ]: received")
    else:
        ser.write(b'\x00') 
        ser.write(b'\x00') 
        ser.write(b'\x00') 
        movementDone = True
        time.sleep(0.1)

#arduino connect pySerial
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
cap = cv.VideoCapture(cam_id_L)
if not cap.isOpened():
    print("[ camera ] Cannot open")
    exit()
    
#APTag detector init.
at_detector = Detector(
   families="tag36h11",
   nthreads=4,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0,
)


start = time.time()
while True:
    # Capture frame-by-frame
    try:
        ret, frame = cap.read()
    #frame = cv.imread("175mmPrint_3.jpg", cv.IMREAD_COLOR)
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        gray = rescale_frame(gray, 100/scale)
        result = at_detector.detect(gray, True, (978/scale, 978/scale, 930/scale, 524/scale), 0.175)
        img_size = gray.shape
        
        wid = img_size[1]
        hei = img_size[0]
        angleStore = 0
        distanceStore = 0
        
        for i in result: #for each detected aprilTag. 
            if(i.tag_id == test_tag_id):
                cent = i.center
                Pose_R = i.pose_R
                Pose_T = i.pose_tspot
                # print(i)
                unofficial_tag_position = Pose_T #P @ Pose_R.T @ (-1 * Pose_T)
                # print("relative pos: ")
                # print("x: "+str(unofficial_tag_position[0]) + ", y: "+ str(unofficial_tag_position[1])+ ", z: "+ str(unofficial_tag_position[2]))
                angle = -1*int(math.atan(unofficial_tag_position[2]/unofficial_tag_position[0])*(180/math.pi))
                angle = int(90-abs(angle))
                if(angle > 0):
                    angle = angle * -1

                distance = abs(int(unofficial_tag_position[2]*1000*distanceScale)) 

                angleStore += angle
                distanceStore += distance

                print("[ angle  ]: "+str(angle))
                print("[distance]: "+str(distance))
                #cv.circle(frame,(int(cent[0]), int(cent[1])), 100, (0,0,255), -1)
                #cv.putText(frame, str(i.tag_id), (int(cent[0]), int(cent[1])), font, 3, (0, 0, 0), 10, cv.LINE_4)

        angleStore = angleStore/len(result)
        distanceStore = distanceStore/len(result)

        if(movementDone == True and hasRotated == False):
            movementDone = False
            # hasRotated = True            
            send_angle(angleStore)
        if(movementDone == True and hasRotated == True):
            movementDone = False
            hasRotated = False
            send_distance(distanceStore)
        
        angleStore = 0
        distanceStore = 0
        
        #check if movmement is done - arduino sends 0x61 on move done. 
        readS = ser.read(1)
        if(b'\x61' == readS):
                print("[movement] Done")
                print()
                movementDone = True 
        #cv.imshow('frame', frame)
    except i:
        print(i)
        break
# When everything done, release the capture
cap.release()
#cv.destroyAllWindows()
ser.close()             # close port