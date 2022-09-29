'''
 Based on the following tutorial:
   http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_calib3d/py_calibration/py_calibration.html
'''

import numpy as np
import cv2
import glob
import sys
import random

# Define the chess board rows and columns
rows = 8
cols = 6

# Set the termination criteria for the corner sub-pixel algorithm
criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 30, 0.001)

# Prepare the object points: (0,0,0), (1,0,0), (2,0,0), ..., (6,5,0). They are the same for all images
objectPoints = np.zeros((rows * cols, 3), np.float32)
objectPoints[:, :2] = np.mgrid[0:rows, 0:cols].T.reshape(-1, 2)

# Create the arrays to store the object points and the image points
objectPointsArray = []
imgPointsArray = []

# Loop over the image files
print("reading images from directory "+sys.argv[1])
for path in glob.glob(sys.argv[1]+'/*.jpg'):
    print("reading image "+path)
    # Load the image and convert it to gray scale
    img = cv2.imread(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (rows, cols), None)

    # Make sure the chess board pattern was found in the image
    if ret:
        # Refine the corner position
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # Add the object points and the image points to the arrays
        objectPointsArray.append(objectPoints)
        imgPointsArray.append(corners)

        # Draw the corners on the image
        cv2.drawChessboardCorners(img, (rows, cols), corners, ret)

    # Display the image
    cv2.imshow('chess board', img)
    cv2.waitKey(500)

# Calibrate the camera and save the results
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objectPointsArray, imgPointsArray, gray.shape[::-1], None, None)
np.savez(sys.argv[1]+'/calib_data.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
print("ret", ret)
print("mtx", mtx)
print("dist", dist)
print("rvecs", rvecs)
print("tvecs", tvecs)
print("imageSize", gray.shape[::-1])

# Print the camera calibration error
error = 0

for i in range(len(objectPointsArray)):
    imgPoints, _ = cv2.projectPoints(objectPointsArray[i], rvecs[i], tvecs[i], mtx, dist)
    error += cv2.norm(imgPointsArray[i], imgPoints, cv2.NORM_L2) / len(imgPoints)

print("Total error: ", error / len(objectPointsArray))

# Load one of the test images
one_file = random.choice(glob.glob(sys.argv[1]+"/*.jpg"))
img = cv2.imread(one_file)
h, w = img.shape[:2]

# Obtain the new camera matrix and undistort the image
newCameraMtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
undistortedImg = cv2.undistort(img, mtx, dist, None, mtx)

# Crop the undistorted image
# x, y, w, h = roi
# undistortedImg = undistortedImg[y:y + h, x:x + w]
fx, fy, height, ppx, ppy, width = mtx[0][0], mtx[1][1], h, mtx[0][2], mtx[1][2], w
rk1, rk2, tp1, tp2, rk3 = dist[0]
print(
    f'\n'
    f'"camera_parameters": {{\n'
    f'   "fx": {fx},\n'
    f'   "fy": {fy},\n'
    f'   "height": {height},\n'
    f'   "ppx": {ppx},\n'
    f'   "ppy": {ppy},\n'
    f'   "width": {width}\n'
    f' }},\n'
    f' "distortion_parameters": {{\n'
    f'   "rk1": {rk1},\n'
    f'   "rk2": {rk2},\n'
    f'   "rk3": {rk3},\n'
    f'   "tp1": {tp1},\n'
    f'   "tp2": {tp2}\n'
    f' }},\n'
)

# Display the final result
cv2.imshow('chess board', np.hstack((img, undistortedImg)))
cv2.waitKey(0)
cv2.destroyAllWindows()
