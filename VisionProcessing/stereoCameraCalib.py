import numpy as np 
import cv2
from tqdm import tqdm

DIM=(1920, 1080)
K=np.array([[1053.9492767154009, 0.0, 951.950093568802], [0.0, 1052.5528725501529, 465.04595064900246], [0.0, 0.0, 1.0]])
D=np.array([[-0.05334899174471995], [0.004050987506634966], [-0.001763065658573223], [-0.0027162184694266523]])
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

# Set the path to the images captured by the left and right cameras
pathL = "./data/stereoL/"
pathR = "./data/stereoR/"

print("Extracting image coordinates of respective 3D pattern ....\n")

# Termination criteria for refining the detected corners
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

img_ptsL = []
img_ptsR = []
obj_pts = []

for i in tqdm(range(1,35)):
    imgL = cv2.imread(pathL+"%d.png"%i)
    imgR = cv2.imread(pathR+"%d.png"%i)
    imgL = cv2.remap(imgL, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    imgR = cv2.remap(imgR, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    imgL_gray = cv2.imread(pathL+"%d.png"%i,0)
    imgR_gray = cv2.imread(pathR+"%d.png"%i,0)
    imgL_gray = cv2.remap(imgL_gray, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    imgR_gray = cv2.remap(imgR_gray, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    outputL = imgL.copy()
    outputR = imgR.copy()

    retR, cornersR =  cv2.findChessboardCorners(outputR,(9,6),None)
    retL, cornersL = cv2.findChessboardCorners(outputL,(9,6),None)

    if retR and retL:
        obj_pts.append(objp)
        cv2.cornerSubPix(imgR_gray,cornersR,(11,11),(-1,-1),criteria)
        cv2.cornerSubPix(imgL_gray,cornersL,(11,11),(-1,-1),criteria)
        cv2.drawChessboardCorners(outputR,(9,6),cornersR,retR)
        cv2.drawChessboardCorners(outputL,(9,6),cornersL,retL)
        cv2.imshow('cornersR',outputR)
        cv2.imshow('cornersL',outputL)
        cv2.waitKey(0)

        img_ptsL.append(cornersL)
        img_ptsR.append(cornersR)


print("Calculating left camera parameters ... ")
# Calibrating left camera
retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(obj_pts,img_ptsL,imgL_gray.shape[::-1],None,None)
hL,wL= imgL_gray.shape[:2]
new_mtxL, roiL= cv2.getOptimalNewCameraMatrix(mtxL,distL,(wL,hL),1,(wL,hL))

print("Calculating right camera parameters ... ")
# Calibrating right camera
retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(obj_pts,img_ptsR,imgR_gray.shape[::-1],None,None)
hR,wR= imgR_gray.shape[:2]
new_mtxR, roiR= cv2.getOptimalNewCameraMatrix(mtxR,distR,(wR,hR),1,(wR,hR))


print("Stereo calibration .....")
flags = 0
flags |= cv2.CALIB_FIX_INTRINSIC
# Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat are calculated.
# Hence intrinsic parameters are the same 

criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
retS, new_mtxL, distL, new_mtxR, distR, Rot, Trns, Emat, Fmat = cv2.stereoCalibrate(obj_pts,
                                                          img_ptsL,
                                                          img_ptsR,
                                                          new_mtxL,
                                                          distL,
                                                          new_mtxR,
                                                          distR,
                                                          imgL_gray.shape[::-1],
                                                          criteria_stereo,
                                                          flags)

# Once we know the transformation between the two cameras we can perform stereo rectification
# StereoRectify function
rectify_scale= 1 # if 0 image croped, if 1 image not croped
rect_l, rect_r, proj_mat_l, proj_mat_r, Q, roiL, roiR= cv2.stereoRectify(new_mtxL, distL, new_mtxR, distR,
                                                 imgL_gray.shape[::-1], Rot, Trns,
                                                 rectify_scale,(0,0))

# Use the rotation matrixes for stereo rectification and camera intrinsics for undistorting the image
# Compute the rectification map (mapping between the original image pixels and 
# their transformed values after applying rectification and undistortion) for left and right camera frames
Left_Stereo_Map= cv2.initUndistortRectifyMap(new_mtxL, distL, rect_l, proj_mat_l,
                                             imgL_gray.shape[::-1], cv2.CV_16SC2)
Right_Stereo_Map= cv2.initUndistortRectifyMap(new_mtxR, distR, rect_r, proj_mat_r,
                                              imgR_gray.shape[::-1], cv2.CV_16SC2)


print("Saving paraeters ......")
cv_file = cv2.FileStorage("data/params_py.xml", cv2.FILE_STORAGE_WRITE)
cv_file.write("Left_Stereo_Map_x",Left_Stereo_Map[0])
cv_file.write("Left_Stereo_Map_y",Left_Stereo_Map[1])
cv_file.write("Right_Stereo_Map_x",Right_Stereo_Map[0])
cv_file.write("Right_Stereo_Map_y",Right_Stereo_Map[1])
cv_file.release()