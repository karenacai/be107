import cv2
import cv2.aruco as aruco
import glob
import pickle
import numpy as np
from PIL import Image
import os
import PyCapture2
import time
import h5py

bus = PyCapture2.BusManager()
numCams = bus.getNumOfCameras()
camera = PyCapture2.Camera()
uid = bus.getCameraFromIndex(0)
camera.connect(uid)
camera.startCapture()

calibpath = 'calibration.pckl'

hf = h5py.File('data.h5', 'w')

# Check for camera calibration data
if not os.path.exists('./{}'.format(calibpath)):
    print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
    exit()
else:
    f = open(calibpath, 'rb')
    cameraMatrix, distCoeffs = pickle.load(f)
    f.close()
    if cameraMatrix is None or distCoeffs is None:
        print("Calibration issue. Remove ./calibration.pckl and recalibrate your camera with CalibrateCamera.py.")
        exit()

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_50)

# Create grid board object we're using in our stream
board = aruco.GridBoard_create(
        markersX=5,
        markersY=7,
        markerLength=0.04,
        markerSeparation=0.01,
        dictionary=ARUCO_DICT)

# Create vectors we'll be using for rotations and translations for postures
rvecs, tvecs = None, None

#cam = cv2.VideoCapture(0)
current_ms = lambda: int(round(time.time()*1000))

t_data = np.array([])# initialize empty data array
r_data = np.array([])
timestamp = np.array([])


while(True):
    image = camera.retrieveBuffer()
    row_bytes = float(len(image.getData())) / float(image.getRows());
    gray = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols()) );
    ret = True
    # Capturing each frame of our video stream
    #ret, QueryImg = cam.read()
    if ret == True:
        # grayscale image
        #gray = cv2.cvtColor(QueryImg, cv2.COLOR_BGR2GRAY)
    
        # Detect Aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
  
        # Refine detected markers
        # Eliminates markers not part of our board, adds missing markers to the board
        corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
                image = gray,
                board = board,
                detectedCorners = corners,
                detectedIds = ids,
                rejectedCorners = rejectedImgPoints,
                cameraMatrix = cameraMatrix,
                distCoeffs = distCoeffs)   

        ###########################################################################
        # TODO: Add validation here to reject IDs/corners not part of a gridboard #
        ###########################################################################

        # Outline all of the markers detected in our image
        gray = aruco.drawDetectedMarkers(gray, corners, )#borderColor=(0, 0, 255))

        # Require 15 markers before drawing axis
        if ids is not None:
            # Estimate the posture of the gridboard, which is a construction of 3D space based on the 2D video 
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners,1, cameraMatrix, distCoeffs)
            #print(rvecs, tvecs)
            #print("hi")
            for rvec, tvec in zip(rvecs,tvecs):
            #if pose:
            
                if r_data.size == 0:
                    r_data = rvec[0]
                else:
                    r_data = np.vstack((r_data, rvec[0]))
                if t_data.size == 0:
                    t_data = tvec[0]
                else:
                    t_data = np.vstack((t_data, tvec[0]))
                if timestamp.size == 0:
                    timestamp = np.array([0])
                    t_start = current_ms()
                else:
                    dt = current_ms()-t_start
                    timestamp = np.vstack((timestamp, dt))
                    #print(timestamp)
                #r_data = np.hstack((r_data, rvec[0]))
                #t_data = np.hstack((t_data, tvec[0]))
                # Draw the camera posture calculated from the gridboard
                gray = aruco.drawAxis(gray, cameraMatrix, distCoeffs, rvec, tvec, 0.3)
                cv2.putText(gray, "Id: "+str(ids), (0,64), cv2.FONT_HERSHEY_SIMPLEX,1, (0,255,0),2,cv2.LINE_AA)
            
                # Display our image
        cv2.imshow('QueryImage', gray)

    # Exit at the end of the video on the 'q' keypress
    if cv2.waitKey(1) & 0xFF == ord('q'):
        # write data to hdf5 file
        hf.create_dataset('r_vecs', data=r_data)
        hf.create_dataset('t_vecs', data=t_data)
        hf.create_dataset('time', data=timestamp)
        hf.close()
        break

cv2.destroyAllWindows()