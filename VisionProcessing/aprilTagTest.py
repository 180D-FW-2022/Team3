from pupil_apriltags import Detector

import numpy as np
import cv2 as cv
import time
import math 

cap = cv.VideoCapture(0)
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
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame = cv.imread("175mmPrint_3.jpg", cv.IMREAD_COLOR)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    result = at_detector.detect(gray, True, (1570, 1570, 945, 630), 0.175)
    img_size = gray.shape
    
    wid = img_size[1]
    hei = img_size[0]
    for i in result:
        cent = i.center
        Pose_R = i.pose_R
        Pose_T = i.pose_t
        unofficial_tag_position = P @ Pose_R.T @ (-1 * Pose_T)
        print("relative pos: ")
        print("x: "+str(unofficial_tag_position[0]) + ", y: "+ str(unofficial_tag_position[1])+ ", z: "+ str(unofficial_tag_position[2]))
        print("angle: "+str(math.atan(unofficial_tag_position[2]/unofficial_tag_position[1])*(180/math.pi)))
        cv.circle(frame,(int(cent[0]), int(cent[1])), 100, (0,0,255), -1)
        cv.putText(frame, 
                    str(i.tag_id), 
                    (int(cent[0]), int(cent[1])), 
                    font, 3, 
                    (0, 0, 0), 
                    10, 
                    cv.LINE_4)
   # print(result)
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    #gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # Display the resulting frame
    cv.imshow('frame', frame)
    if cv.waitKey(1) == ord('q'):
        break
    break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()