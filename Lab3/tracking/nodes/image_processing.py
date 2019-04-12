#!/usr/bin/env python
import rospy
import cv2
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
import time
import os

# name of output file; overwrite the flyTracks data everytime you run the code (assumes the file given by the filename exists)
filename = '/home/be107/catkin_ws/src/tracking/nodes/flyTracks.txt'
f = open(filename, 'r+')
f.truncate(0)

# input is a color image, output should be a processed image where it is easy to extract the coordinates of the fly
def imageProcessing(img):
    # insert your image processing algorithm here 
    return img

# input is the processed image, output is the x-y coordinates of the fly
def findFly(processedImg):
    return 0, 0

# show image stream that is coming from a topic in OpenCv
def callback(data):
    try:
        img = bridge.imgmsg_to_cv2(data).copy()
    except CvBridgeError as e:
        print(e)
    
    proc_img = imageProcessing(img)
    x, y = findFly(proc_img)
    # show processed image
    cv2.imshow('image',proc_img)
    cv2.waitKey(2)
    # write the position data to file
    f.write('{0} {1}\n'.format(x, y))
    

# initialize camera subscriber
rospy.init_node('image_processing_node')
img_sub = rospy.Subscriber('/camera/image_mono', Image, callback)
bridge = CvBridge()
rospy.spin()
f.close()
