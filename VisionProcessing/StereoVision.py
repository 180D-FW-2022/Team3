import numpy as np 
import cv2
from matplotlib import pyplot as plt

CamL_id = 1
CamR_id = 2

CamL= cv2.VideoCapture(CamL_id)
CamR= cv2.VideoCapture(CamR_id)
print("Reading parameters ......")
DIM=(1920, 1080)
K=np.array([[1053.9492767154009, 0.0, 951.950093568802], [0.0, 1052.5528725501529, 465.04595064900246], [0.0, 0.0, 1.0]])
D=np.array([[-0.05334899174471995], [0.004050987506634966], [-0.001763065658573223], [-0.0027162184694266523]])
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

print("Reading parameters ......")
cv_file = cv2.FileStorage("data/params_py.xml", cv2.FILE_STORAGE_READ)

Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
cv_file.release()

#create two subplots
ax1 = plt.subplot(1,2,1)
ax2 = plt.subplot(1,2,2)

def grab_frame(cap):
    ret,frame = cap.read()
    return cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

#create two image plots
im1 = ax1.imshow(grab_frame(CamL))
im2 = ax2.imshow(grab_frame(CamR))

plt.ion()
stereo = cv2.StereoBM_create()
while True:
	imgR= grab_frame(CamR)
	imgL= grab_frame(CamL)
	#imgR = cv2.imread("./data/stereoR/1.png")
	#imgL = cv2.imread("./data/stereoL/1.png")
#	imgL = cv2.resize(imgL, (640,480), interpolation = cv2.INTER_AREA)
	#imgR = cv2.resize(imgR, (640,480), interpolation = cv2.INTER_AREA)
	#if retL and retR:
	imgR_gray = cv2.cvtColor(imgR,cv2.COLOR_BGR2GRAY)
	imgL_gray = cv2.cvtColor(imgL,cv2.COLOR_BGR2GRAY)
	# Left_nice= cv2.remap(imgL,Left_Stereo_Map_x,Left_Stereo_Map_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
	# Right_nice= cv2.remap(imgR,Right_Stereo_Map_x,Right_Stereo_Map_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
	h,w = imgL.shape[:2]
	Left_nice = cv2.remap(imgL, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
	Right_nice = cv2.remap(imgR, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

	Left_nice= cv2.remap(Left_nice,Left_Stereo_Map_x,Left_Stereo_Map_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
	Right_nice= cv2.remap(Right_nice,Right_Stereo_Map_x,Right_Stereo_Map_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

	output = Right_nice.copy()
	output[:,:,0] = Right_nice[:,:,0]
	output[:,:,1] = Right_nice[:,:,1]
	output[:,:,2] = Left_nice[:,:,2]

	# output = Left_nice+Right_nice
	Left_nice = cv2.cvtColor(Left_nice,cv2.COLOR_BGR2GRAY)
	Right_nice = cv2.cvtColor(Right_nice,cv2.COLOR_BGR2GRAY)
	Left_nice = cv2.resize(Left_nice, (600, 500))
	Right_nice = cv2.resize(Right_nice, (600, 500))
	stereo.setNumDisparities(144)
	stereo.setBlockSize(35)
	# stereo.setPreFilterType(preFilterType)
	# stereo.setPreFilterSize(preFilterSize)
	# stereo.setPreFilterCap(preFilterCap)
	# stereo.setTextureThreshold(textureThreshold)
	# stereo.setUniquenessRatio(uniquenessRatio)
	# stereo.setSpeckleRange(speckleRange)
	# stereo.setSpeckleWindowSize(speckleWindowSize)
	#stereo.setDisp12MaxDiff(5)
	#stereo.setMinDisparity(1)

	
	disparity = stereo.compute(Left_nice,Right_nice) # Calculating disparity using the StereoBM algorithm
	#disparity = (disparity/16.0 - 10)

	im1.set_data(Left_nice)
	im2.set_data(disparity)
	plt.pause(0.2)
	
	# Left_nice = cv2.cvtColor(Left_nice,cv2.COLOR_BGR2GRAY)
	# Right_nice = cv2.cvtColor(Right_nice,cv2.COLOR_BGR2GRAY)

	# stereo = cv2.StereoBM_create(numDisparities=10*16, blockSize=49)
	# disparity = stereo.compute(Left_nice,Right_nice)
	# plt.imshow(disparity,'gray')
	# plt.ion()

plt.ioff() # due to infinite loop, this gets never called.
plt.show()