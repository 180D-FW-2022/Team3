from pupil_apriltags import Detector

import numpy as np
import cv2 as cv
import time
import math 

import warnings
import serial
import serial.tools.list_ports
import time

arduino_ports = [
    p.device
    for p in serial.tools.list_ports.comports()
    if ('Arduino' in p.description or 'USB Serial' in p.description)  # may need tweaking to match new arduinos
]
if not arduino_ports:
    raise IOError("No Arduino found")
if len(arduino_ports) > 1:
    warnings.warn('Multiple Arduinos found - using the first')


ser = serial.Serial(arduino_ports[0], timeout = 0.5)
time.sleep(4)
print(ser.name)         # check which port was really used



cap = cv.VideoCapture(2)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

at_detector = Detector(
   families="tag36h11",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0,
)

font = cv.FONT_HERSHEY_SIMPLEX
P = [[1, 0, 0],[0, -1, 0],[0, 0, -1]]

start = time.time()
while True:
    # Capture frame-by-frame
    try:
        ret, frame = cap.read()
    #frame = cv.imread("175mmPrint_3.jpg", cv.IMREAD_COLOR)
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        result = at_detector.detect(gray, True, (978, 978, 930, 524), 0.175)
        img_size = gray.shape
        
        wid = img_size[1]
        hei = img_size[0]
        for i in result:
            cent = i.center
            Pose_R = i.pose_R
            Pose_T = i.pose_t
            print(i)
            unofficial_tag_position = Pose_T #P @ Pose_R.T @ (-1 * Pose_T)
            print("relative pos: ")
            print("x: "+str(unofficial_tag_position[0]) + ", y: "+ str(unofficial_tag_position[1])+ ", z: "+ str(unofficial_tag_position[2]))
            angle = -1*int(math.atan(unofficial_tag_position[2]/unofficial_tag_position[0])*(180/math.pi))
            print("angle: "+str(angle))
            cv.circle(frame,(int(cent[0]), int(cent[1])), 100, (0,0,255), -1)
            cv.putText(frame, 
                        str(i.tag_id), 
                        (int(cent[0]), int(cent[1])), 
                        font, 3, 
                        (0, 0, 0), 
                        10, 
                        cv.LINE_4)
            if(time.time()-start) > 5:
                start = time.time()
                angle_send = int(90-abs(angle))
                if(angle > 0):
                    angle_send = angle_send * -1
                print("angleSend: "+str(angle_send))
                bytes_val = angle_send.to_bytes(2, 'big', signed=True)
                ser.write(str.encode('d'))
                ser.write(bytes_val)

           
                if(bytes_val == ser.read(2)):
                    print("received")
                else:
                    ser.write(b'\xFF') 

                time.sleep(5)
                angle_send = int(unofficial_tag_position[2]*1000)
                print("distanceSend: "+str(angle_send))
                bytes_val = angle_send.to_bytes(2, 'big', signed=True)
                ser.write(str.encode('m'))
                ser.write(bytes_val)

           
                if(bytes_val == ser.read(2)):
                    print("received")
                else:
                    ser.write(b'\xFF')   
                          
    # print(result)
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ... ")
            break
        # Our operations on the frame come here
        #gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # Display the resulting frame
        cv.imshow('frame', frame)
        if cv.waitKey(1) == ord('q'):
            break
    except:
        print("An exception occurred")
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
ser.close()             # close port