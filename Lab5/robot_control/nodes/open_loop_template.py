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


if __name__ == '__main__':
    
    # setting up right version of Python
    if sys.version_info[0] < 3:
        errorcode = ImportError
    else:
        errorcode = ModuleNotFoundError

    #CHANGE THIS to define which robot you are controlling
    robotname="be107bot1"

    # the topic names to subscribe or publish to 
    topic_motor1 = "/{}/motor1".format(robotname)
    topic_motor2 = "/{}/motor2".format(robotname)
    topic_image = '/{}/image'.format(robotname)

    # setting up the publisher nodes
    mot1 = rospy.Publisher(topic_motor1,Int32,queue_size=10)
    mot2 = rospy.Publisher(topic_motor2,Int32,queue_size=10)
    
    #==============================================================#
    # ADD YOUR OWN open-loop code here                             #
    #==============================================================#
    # note change the time.sleep command to determine how long the robot will move forward or how long the robot will turn
    
    while True: 
        # send commands to move robot forward
        # time.sleep() 
        # send commands to turn robot 90 degrees in place
        # time.sleep()
    
