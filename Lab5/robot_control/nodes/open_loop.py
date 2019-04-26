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

robotname="be107bot8"

# the topic names to subscribe or publish to 
topic_motor1 = "/{}/motor1".format(robotname)
topic_motor2 = "/{}/motor2".format(robotname)
topic_image = '/{}/image'.format(robotname)

print(topic_motor1)
print(topic_motor2)

# setting up the publisher nodes
mot1 = rospy.Publisher(topic_motor1,Int32,queue_size=1)
mot2 = rospy.Publisher(topic_motor2,Int32,queue_size=1)

# publish 0 command to motors
def stop_motors():
    print("goodbye")
    mL = Int32()
    mR = Int32()
    mL.data = int(0)
    mR.data = int(0)
    mot1.publish(mL)
    mot2.publish(mR)
    rate.sleep()



# setting up right version of Python
if sys.version_info[0] < 3:
    errorcode = ImportError
else:
    errorcode = ModuleNotFoundError

rospy.init_node('control_{}'.format(robotname))


#rospy.spin()
rate = rospy.Rate(10)
atexit.register(stop_motors)
while not rospy.is_shutdown():
    mL = Int32()
    mR = Int32()
    print("hello")
    mL.data = int(100)
    mR.data = int(100)
    mot1.publish(mL)
    mot2.publish(mR)
    time.sleep(5)
    mL.data = int(0)
    mR.data = int(100)
    mot1.publish(mL)
    mot2.publish(mR)
    time.sleep(4)
    rate.sleep()



#==============================================================#
# ADD YOUR OWN open-loop code here                             #
#==============================================================#
# note change the time.sleep command to determine how long the robot will move forward or how long the robot will turn
#while True: 
#    mL = Int32()
#    mR = Int32()
#    mL.data = 100
#    mR.data = 100
#    mot1.publish(mL)
#    mot2.publish(mR)
#    time.sleep(5)
#    mL.data = 0
#    mR.data = 100
#    time.sleep(3)
    # send commands to move robot forward
    # time.sleep() 
    # send commands to turn robot 90 degrees in place
    # time.sleep()
    
#rospy.init_node('control for_{}'.format(robotname))
#rospy.spin()
