#!/usr/bin/env python
import rospy
import cv2
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
import time

# set up publisher node to take camera input and output to ROS camera topic
pub = rospy.Publisher('/camera/image_mono', Image, queue_size=100)
rospy.init_node('image_publisher', anonymous=True)
bridge = CvBridge()

# set up subscriber node to read from ROS camera topic, do some preliminary image processing and view image data

dowebcam=False
# wrapper (use camera capture, otherwise use webcam for feed)
try:
    import cam_capture
except ModuleNotFoundError:
    dowebcam = True

if(dowebcam):
    cam = cv2.VideoCapture(0)
else:
    cam = cam_capture.FlyCamera(0) #intialize the pointgrey camera

while(True):
    ret1, origimg = cam.read()
    img = origimg.copy()
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('image',img)
    #if cv2.waitKey(1) & 0xFF==ord('q'):
        #break
    try:
        pub.publish(bridge.cv2_to_imgmsg(img))
    except CvBridgeError as e: 
        print(e)
