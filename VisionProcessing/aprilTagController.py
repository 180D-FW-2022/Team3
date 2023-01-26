from pupil_apriltags import Detector

import numpy as np
import cv2 as cv
import time
import math
import struct

import warnings
import serial
import serial.tools.list_ports
import time

test_mode = 1
#global consts
test_tag_id = 0
distanceScale = 0.75

cam_id = 0 #"/dev/video0"
scale = 1.0

font = cv.FONT_HERSHEY_SIMPLEX

#global vars
#global movementDone
movementDone = True
stepCounter = 0

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
            if(True):#i.tag_id == test_tag_id):
                cent = i.center
                Pose_R = i.pose_R
                Pose_T = i.pose_t
                # print(i)
                unofficial_tag_position = Pose_T #P @ Pose_R.T @ (-1 * Pose_T)
                # print("relative pos: ")
                print("x: "+str(unofficial_tag_position[0]) + ", y: "+ str(unofficial_tag_position[1])+ ", z: "+ str(unofficial_tag_position[2]))
                angle = -1*int(math.atan(unofficial_tag_position[2]/unofficial_tag_position[0])*(180/math.pi))
                angle_temp = angle
                angle = int(90-abs(angle))
                if(angle_temp > 0):
                    angle = angle * -1

                distance = abs(int(unofficial_tag_position[2]*100*distanceScale)) 

                angleStore += angle
                distanceStore += distance

                print("[ angle  ]: "+str(angle))
                print("[distance]: "+str(distance))
                cv.circle(frame,(int(cent[0]), int(cent[1])), 100, (0,0,255), -1)
                cv.putText(frame, str(i.tag_id), (int(cent[0]), int(cent[1])), font, 3, (0, 0, 0), 10, cv.LINE_4)
        if(len(result) > 0):
            angleStore = int(angleStore/len(result))
            distanceStore = int(distanceStore/len(result))

            if(movementDone == True):
                if(stepCounter == 0):
                    movementDone = False     
                    send_angle(angleStore)
                    stepCounter = 1       
                elif(stepCounter == 1):
                    movementDone = False
                    stepCounter = 2
                    send_distance(distanceStore)
                elif(stepCounter == 3):
                    movementDone = False
                    send_angle(angleStore)
                    stepCounter = 4           
                elif(stepCounter == 4):
                    movementDone = False
                    stepCounter = 5
                    send_distance(distanceStore)
                elif(stepCounter == 5):
                    movementDone = False
                    stepCounter = 6
                    send_distance(distanceStore)
                angleStore = 0
                distanceStore = 0
        if(movementDone == True and stepCounter == 2):
            movementDone = False
            stepCounter = 3
            send_angle(90)
        
        elif(movementDone == True and stepCounter == 6):
            stepCounter = 0
            entered_value = input('press button to restart.\n')
            
            
        
        #check if movmement is done - arduino sends 0x61 on move done. 
        readS = 0x00
        if(test_mode == 0):
            readS = ser.read(1)
        if(b'\x61' == readS):
                print("[movement] Done")
                print()
                movementDone = True 
                time.sleep(0.1)
        cv.imshow('frame', frame)
        if cv.waitKey(1) == ord('q'):
            break
    except Exception as e: 
        print(e)
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
if(test_mode == 0):     
    ser.close()             # close port