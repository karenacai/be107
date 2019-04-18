#!/usr/bin/env python
import rospy
import cv2
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import os
import socket
hostname = socket.gethostname()
# name of output file; overwrite the flyTracks data everytime you run the code (assumes the file given by the filename exists)
filename = '/home/be107/catkin_ws/src/image_subscriber/nodes/flyTracks.txt'
flyseek = False
if(flyseek):
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
def callback(data,flyseek=False):
    try:
        img = bridge.imgmsg_to_cv2(data).copy()
    except CvBridgeError as e:
        print(e)

    proc_img = imageProcessing(img)
    if(flyseek):
        x, y = findFly(proc_img)
        # show processed image
        f.write('{0} {1}\n'.format(x, y))
    cv2.imshow('image',proc_img)
    cv2.waitKey(2)
    # write the position data to file



# initialize camera subscriber
rospy.init_node('image_processing_node')
robotname = "be107bot8"
img_sub = rospy.Subscriber('/{}/image'.format(robotname), Image, \
                                            (lambda a: callback(a,flyseek)))
bridge = CvBridge()
rospy.spin()
if(flyseek):
    f.close()
