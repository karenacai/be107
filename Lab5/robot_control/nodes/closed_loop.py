#!/usr/bin/env python
import roslib, rospy
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import os
import socket
import atexit
import curses
import time
import numpy as np

#we get our hostname which should be unique on the network. Actually
#since this code runs on your computer and doesn't make new topics
#this is kinda unused. Be creative! :D
#hostname = socket.gethostname()

#this next part is for compatibility between code running on different
#python versions. In case you want to check if some package has been
#successfully imported, you'll want to indicate the proper error message
#in your try except block. See image_subscriber from lab3
if sys.version_info[0] < 3:
    errorcode = ImportError
else:
    errorcode = ModuleNotFoundError

#this is where you define what robot you are trying to control.
################CHANGE THIS##################
robotname="be107bot8"
#we fill in our topic names with the appropriate robot name
#MAKE SURE YOU FIXED IT, ABOVE.
topic_motor1 = "/{}/motor1".format(robotname)
topic_motor2 = "/{}/motor2".format(robotname)
topic_image = '/{}/image'.format(robotname)
#print(topic_motor1)

mot1 = rospy.Publisher(topic_motor1,Int32,queue_size=10)
mot2 = rospy.Publisher(topic_motor2,Int32,queue_size=10)

# output the properties (slope, etc.) of the line given an image
def extractLine(img_gray):
    return dst


# input is the line properties extracted from the image, output is the motor commands
def visionToMotors(line_prop):
    m1max = 300
    m2max = 300
    m1 = Int32()
    m2 = Int32()
    m1.data = 0
    m2.data = 0

    return m1,m2

# show image stream that is coming from a topic in OpenCv and publish relevant motor commands
def callback(data):
    np_arr = np.fromstring(data.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    proc_image = extractLine(gray)
    
    m1,m2 = visionToMotors(gray)
    mot1.publish(m1)
    mot2.publish(m2)
    cv2.imshow('image',proc_image)
    cv2.waitKey(2)

rospy.init_node('CLcontrol_{}'.format(robotname))
img_sub = rospy.Subscriber(topic_image, CompressedImage, callback,queue_size=1)
rospy.spin()
