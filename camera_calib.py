# -dra directly yoinked this from her 4078 archives
# -------------------------------------------------------
# this file calculates the camera matrix using images captured from the checkerboard calibration and saves them into a txt file.
# if not overwritten, this is defaulted to an existing camera calibration setting
# -------------------------------------------------------  

import os
import cv2
import glob 
import numpy as np

if __name__ == '__main__':

    CHECKERBOARD = (6,9) # number of inner corners (row, column)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Creating vector to store vectors of 3D points for each checkerboard image
    objpoints = []
    # Creating vector to store vectors of 2D points for each checkerboard image
    imgpoints = []

    # Defining the world coordinates for 3D points
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    # Extracting path of individual image stored in a given directory
    images = glob.glob('calib_pics/calib_*.png')
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        # If desired number of corner are detected, we refine the pixel coordinates and display them on the images of checker board
        if ret == True:
            objpoints.append(objp)
            
            # refining pixel coordinates for given 2d points.
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)

        cv2.imshow(fname,img)
        cv2.waitKey(0)

    cv2.destroyAllWindows()

    h,w = img.shape[:2]

    # Performing camera calibration by passing the value of known 3D points (objpoints) and corresponding pixel coordinates of the detected corners (imgpoints)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # save the intrinsic parameters 
    dataDir = "{}".format('./calibration/parameters/')
    print("\nIntrinsic parameters:\n", mtx)
    fileNameI = "{}camera_matrix.txt".format(dataDir)
    np.savetxt(fileNameI, mtx, delimiter=',')