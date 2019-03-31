import cv2
#import cv2.aruco as aruco
import time
import glob
import pickle
import numpy as np
import os
dowebcam=False
try:
    import cam_capture
except ModuleNotFoundError:
    dowebcam = True
#dowebcam=True
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
if(dowebcam):
    cam = cv2.VideoCapture(0)
else:
    cam = cam_capture.FlyCamera(0) #intialize the pointgrey camera
calfolder = "calibration_images"
imageprefix = "chessboard_"
automation=True
#'./calibration_images_flycap/t*.png')
#print(images)
if(automation):
    imct = 0
    while(True):
        imgpoints=[]
        ret1, origimg = cam.read()
        img = origimg.copy()
        haschess = False
        if ret1 == True:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (7,6), None)
            # If found, add object points, image points (after refining them)
            if ret == True:
                haschess=True
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners)
                # Draw and display the corners
                cv2.drawChessboardCorners(img, (7,6), corners2, ret)
                cv2.drawChessboardCorners(img2, (7,6), corners2, ret)
            img2 = img.copy()
            cv2.putText(img2, "p to save, q to quit", (0,64),\
                        cv2.FONT_HERSHEY_SIMPLEX,1, (0,255,0),2,cv2.LINE_AA)
            cv2.imshow('img', img2)
        keyp = cv2.waitKey(1)
        if keyp & 0xFF == ord('p'):
            #save the image!
            if(haschess):
                cv2.imwrite(os.path.join(calfolder,imageprefix+str(imct)+".jpg") , origimg );
                imct+=1
                cv2.putText(img, "Saved as {}".format(imageprefix+str(imct)+".jpg"), (0,64),\
                            cv2.FONT_HERSHEY_SIMPLEX,1, (0,255,0),2,cv2.LINE_AA)
                cv2.imshow('img', img)
                cv2.waitKey(1)
                time.sleep(1)
            else:
                cv2.putText(img, "didn't save :(", (0,64),\
                            cv2.FONT_HERSHEY_SIMPLEX,1, (0,255,0),2,cv2.LINE_AA)
                cv2.imshow('img', img)
                cv2.waitKey(1)
                time.sleep(1)
        elif(keyp & 0xFF == ord('q')):
            break
images = glob.glob(os.path.join(calfolder,imageprefix+"*.jpg"))
imgpoints = []
objpoints = []
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (7,6), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(0)
    else:
        print("no chessboard found for {}".format(fname))

ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
cv2.destroyAllWindows()
f = open('calibration.pckl', 'wb')
pickle.dump((cameraMatrix, distCoeffs), f)
f.close()

# Print to console our success
print('Calibration successful. Calibration file used: {}'.format('calibration.pckl'))
