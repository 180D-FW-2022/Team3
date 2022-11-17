import numpy as np
import cv2
import sys 
# You should replace these 3 lines with the output in calibration step
DIM=(1920, 1080)
K=np.array([[1053.9492767154009, 0.0, 951.950093568802], [0.0, 1052.5528725501529, 465.04595064900246], [0.0, 0.0, 1.0]])
D=np.array([[-0.05334899174471995], [0.004050987506634966], [-0.001763065658573223], [-0.0027162184694266523]])
def undistort(img_path):
    img = cv2.imread(img_path)
    h,w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
	
if __name__ == '__main__':
    for p in sys.argv[1:]:
        undistort(p)
