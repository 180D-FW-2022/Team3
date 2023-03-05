
import numpy as np
import cv2
from PIL import Image, ImageEnhance


print("Setting parameters Single ......")
DIM=(1920, 1080)
K=np.array([[1053.9492767154009, 0.0, 951.950093568802], [0.0, 1052.5528725501529, 465.04595064900246], [0.0, 0.0, 1.0]])
D=np.array([[-0.05334899174471995], [0.004050987506634966], [-0.001763065658573223], [-0.0027162184694266523]])
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

print("Reading parameters Stereo ......")
cv_file = cv2.FileStorage("../data/params_py.xml", cv2.FILE_STORAGE_READ)

Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
cv_file.release()

thres = 0.5 # Threshold to detect object
nms_threshold = 0.2 #(0.1 to 1) 1 means no suppress , 0.1 means high suppress 
cap_right = cv2.VideoCapture(2)
cap_left = cv2.VideoCapture(0)
cap_right.set(cv2.CAP_PROP_FRAME_WIDTH,560) #width 
cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT,240) #height 
cap_left.set(cv2.CAP_PROP_FRAME_WIDTH,560) #width 
cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT,240) #height 


classNames = []
with open('coco.names','r') as f:
    classNames = f.read().splitlines()
print(classNames)

font = cv2.FONT_HERSHEY_PLAIN
#font = cv2.FONT_HERSHEY_COMPLEX
Colors = np.random.uniform(0, 255, size=(len(classNames), 3))

weightsPath = "frozen_inference_graph.pb"
configPath = "ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(560,240)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

netV = cv2.dnn_DetectionModel(weightsPath,configPath)
netV.setInputSize(240,560)
netV.setInputScale(1.0/ 127.5)
netV.setInputMean((127.5, 127.5, 127.5))
netV.setInputSwapRB(True)

while True:  #
    success, img = cap_right.read()  #
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    success_L, imgL = cap_left.read() #
    img = cv2.convertScaleAbs(img, alpha = 1 / 1, beta = 0)
    imgL = cv2.convertScaleAbs(imgL, alpha = 1 / 1, beta = 0)
    #img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    #img= cv2.remap(img,Right_Stereo_Map_x,Right_Stereo_Map_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

    classIds, confs, bbox = netV.detect(img, confThreshold=0.5)
    classIdsL, confsL, bboxL = net.detect(imgL, confThreshold=0.5)

    #print(classIds, bbox)

    if len(classIds) != 0:   #
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            if(classId == 1): #person
                cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
                cv2.putText(img, classNames[classId-1].upper(), (box[0]+10, box[1]+30), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)
    if len(classIdsL) != 0:   #
       for classId, confidence, box in zip(classIdsL.flatten(), confsL.flatten(), bboxL):
            if(classId == 1): #person
                cv2.rectangle(imgL, box, color=(0, 255, 0), thickness=2)
                cv2.putText(imgL, classNames[classId-1].upper(), (box[0]+10, box[1]+30), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)

        


    imS = cv2.resize(img, (480*2, 1120*2))                # Resize image
    cv2.imshow('output', imS)
    cv2.waitKey(1)
    imS = cv2.resize(imgL, (1120, 480))                # Resize image
    cv2.imshow('output2', imS)
    cv2.waitKey(1)