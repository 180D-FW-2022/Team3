#!/usr/bin/env python
"""Camera Calibration

Calculates calibration parameters from a set of images.

Copyright (C) 2021-2022  Raymond Oung

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""
import cv2
import numpy as np
import os
import argparse
import sys

# Dimensions of the chess board
CHESS_BOARD = (9, 6)

# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Arrays to store object and image points from all images
objpoints = []  # 3d point in real-world space
imgpoints = []  # 2d points in image plane

# Prepare object points
objp = np.zeros((CHESS_BOARD[0] * CHESS_BOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0 : CHESS_BOARD[0], 0 : CHESS_BOARD[1]].T.reshape(-1, 2)

# Parse command line arguments
PARSER = argparse.ArgumentParser(
    description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
)
PARSER.add_argument("dir", help="Directory of images")
PARSER.add_argument("-s", "--show", help="Show images with estimated corners", action="store_true")
ARGS = PARSER.parse_args()

# Collect all images
filename_list = []
for filename in os.listdir(ARGS.dir):
    path = os.path.join(ARGS.dir, filename)

    # filter-out non-functional files
    if not os.path.isfile(path):
        continue

    # filter-out undesired file-types
    if os.path.splitext(path)[1].lower() in (".png", ".jpg", ".jpeg"):
        filename_list.append(path)

if not filename_list:
    sys.exit("[Error] Files not found: {}".format(ARGS.dir))

# Find the corners for each image
for filename in filename_list:
    print(filename)
    
    img = cv2.imread(filename)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, CHESS_BOARD, None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        # Refine pixel coordinates for given 2D points
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        if ARGS.show:
            # Draw the corners onto the image
            img = cv2.drawChessboardCorners(img, CHESS_BOARD, corners2, ret)
            cv2.imshow("img", img)
            cv2.waitKey(500)

if ARGS.show:
    cv2.destroyAllWindows()

# Print license notice
print("""
dancebots-server  Copyright (C) 2021-2022  Raymond Oung
This program comes with ABSOLUTELY NO WARRANTY.
This is free software, and you are welcome to redistribute it
under certain conditions. Refer to the license notice at the
top of the script.

""")

# Calibrate camera
print("Calculating calibration parameters...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("\nIntrinsic Parameters:")
print("fx: {}".format(mtx.item((0, 0))))
print("fy: {}".format(mtx.item((1, 1))))
print("cx: {}".format(mtx.item((0, 2))))
print("cy: {}".format(mtx.item((1, 2))))

print("\nDistortion Coefficients:")
print("k1: {0:+}".format(dist.item((0, 0))))
print("k2: {0:+}".format(dist.item((0, 1))))
print("p1: {0:+}".format(dist.item((0, 2))))
print("p2: {0:+}".format(dist.item((0, 3))))
print("k3: {0:+}".format(dist.item((0, 4))))

# Calculate re-projection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

print("\nRe-projection error: {}".format(mean_error/len(objpoints)))
print("\nFiles Used: {}".format(len(objpoints)))
