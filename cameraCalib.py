'''
 Based on the following tutorial:
   http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_calib3d/py_calibration/py_calibration.html
'''

import numpy as np
import cv2
import glob
import sys
import random

noGui = '--no-gui' in sys.argv
imagesBasePath = sys.argv[-1]

# imagesBasePath ending with '.py' implies that the user did not pass any arguments
if '--help' in sys.argv or imagesBasePath.endswith('.py'):
    print('Usage: python cameraCalib.py [--no-gui] images_path')
    print('  --no-gui: disable OpenCV GUI (may be required on Linux systems with GTK)')
    print('  images_path: path to directory containing calibration images')
    sys.exit()

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

# Save the grayscale version of the last image
gray = None

# Loop over the image files
print(f"Reading images from directory: {imagesBasePath}")
imagesToParse = glob.glob(imagesBasePath+'/*.jp*g')

if len(imagesToParse) == 0:
    print('Unable to find any jpeg images in the passed directory. ')
    sys.exit()

for (index, path) in enumerate(imagesToParse):
    print(f"Reading image: {path} ({index+1}/{len(imagesToParse)})")
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
        if not noGui:
            # Draw the corners on the image
            cv2.drawChessboardCorners(img, (rows, cols), corners, ret)

    if not noGui:
        # Display the image
        cv2.imshow('chess board', img)
        cv2.waitKey(500)

# Calibrate the camera and save the results
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objectPointsArray, imgPointsArray, gray.shape[::-1], None, None)
np.savez(imagesBasePath+'/calib_data.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
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
one_file = random.choice(imagesToParse)
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
    f'"intrinsic_parameters": {{\n'
    f'   "fx": {fx},\n'
    f'   "fy": {fy},\n'
    f'   "height_px": {height},\n'
    f'   "ppx": {ppx},\n'
    f'   "ppy": {ppy},\n'
    f'   "width_px": {width}\n'
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
if not noGui:
    print('Showing original vs undistorted image')
    print('Press \'0\' to close the window')
    cv2.imshow('chess board', np.hstack((img, undistortedImg)))
    cv2.waitKey(0)
    cv2.destroyAllWindows()
