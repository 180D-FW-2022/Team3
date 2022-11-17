import numpy as np 
import cv2
from matplotlib import pyplot as plt

CamL_id = 2
CamR_id = 1

CamL= cv2.VideoCapture(CamL_id)
CamR= cv2.VideoCapture(CamR_id)

print("Reading parameters ......")
cv_file = cv2.FileStorage("data/params_py.xml", cv2.FILE_STORAGE_READ)

Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
cv_file.release()


while True:
	retR, imgR= CamR.read()
	retL, imgL= CamL.read()
	
	if retL and retR:
		imgR_gray = cv2.cvtColor(imgR,cv2.COLOR_BGR2GRAY)
		imgL_gray = cv2.cvtColor(imgL,cv2.COLOR_BGR2GRAY)

		Left_nice= cv2.remap(imgL,Left_Stereo_Map_x,Left_Stereo_Map_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
		Right_nice= cv2.remap(imgR,Right_Stereo_Map_x,Right_Stereo_Map_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
		output = Right_nice.copy()
		output[:,:,0] = Right_nice[:,:,0]
		output[:,:,1] = Right_nice[:,:,1]
		output[:,:,2] = Left_nice[:,:,2]
		# scale_percent = 10 # percent of original size
		# width = int(imgL.shape[1] * scale_percent / 100)
		# height = int(imgL.shape[0] * scale_percent / 100)
		# dim = (width, height)	
  
		# resize image
		# resized_L = cv2.resize(imgL, dim, interpolation = cv2.INTER_AREA)
		# resized_R = cv2.resize(imgR, dim, interpolation = cv2.INTER_AREA)

		stereo = cv2.StereoBM_create(numDisparities=32, blockSize=25)
		gray1 = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
		gray2 = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
		disparity = stereo.compute(gray1,gray2)
		plt.imshow(disparity,'gray')
		plt.show()

		#cv2.waitKey(1)
	
	else:
		break