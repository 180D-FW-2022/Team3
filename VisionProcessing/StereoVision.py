import numpy as np 
import cv2
from matplotlib import pyplot as plt

CamL_id = 2
CamR_id = 1

CamL= cv2.VideoCapture(CamL_id)
CamR= cv2.VideoCapture(CamR_id)
print("Reading parameters ......")
cv_file = cv2.FileStorage("./data/params_py.xml", cv2.FILE_STORAGE_READ)

Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
print(Left_Stereo_Map_x)
Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
cv_file.release()


while True:
	retR, imgR= CamR.read()
	retL, imgL= CamL.read()
#	imgL = cv2.resize(imgL, (640,480), interpolation = cv2.INTER_AREA)
	#imgR = cv2.resize(imgR, (640,480), interpolation = cv2.INTER_AREA)
	#if retL and retR:
	imgR_gray = cv2.cvtColor(imgR,cv2.COLOR_BGR2GRAY)
	imgL_gray = cv2.cvtColor(imgL,cv2.COLOR_BGR2GRAY)
	Left_nice= cv2.remap(imgL,Left_Stereo_Map_x,Left_Stereo_Map_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
	Right_nice= cv2.remap(imgR,Right_Stereo_Map_x,Right_Stereo_Map_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
	cv2.imshow("left",Left_nice)
	output = Right_nice.copy()
	output[:,:,0] = Right_nice[:,:,0]
	output[:,:,1] = Right_nice[:,:,1]
	output[:,:,2] = Left_nice[:,:,2]

	# output = Left_nice+Right_nice
	#output = cv2.resize(output,(700,700))
	cv2.namedWindow("3D movie",cv2.WINDOW_NORMAL)
	#cv2.resizeWindow("3D movie",700,700)
	cv2.imshow("3D movie",output)
	stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
	disparity = stereo.compute(imgL_gray,imgR_gray)
	plt.imshow(disparity,'gray')
	plt.show()

	cv2.waitKey(1)