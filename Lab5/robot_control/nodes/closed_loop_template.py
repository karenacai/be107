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
print(topic_motor1)

#here we will actually define the publisher objects. They will publish
#Int32 data, which is just an integer that has a fairly high maximum (absolute)
#value. Much higher than we will ever use. Queue size means that messages that have not
#been read by the robot are held in reserve and sent later. You can decrease this and
#the robot should respond to more immediate information, although in my testing
#everything was pretty immediate with queue_size=10.
mot1 = rospy.Publisher(topic_motor1,Int32,queue_size=10)
mot2 = rospy.Publisher(topic_motor2,Int32,queue_size=10)


# input is the processed image, output is the motor commands
def visionToMotors(img):
    """this function is given a processed image, and outputs a
    directive for robot locomotion. m1 and m2 are Int32 data
    members where a value between -300 and 300 controls motor
    motion. 300 is full forward, -300 is full reverse.
    Intermediate values result in slower motor rotation."""
    m1max = 300
    m2max = 300
    m1 = Int32()
    m2 = Int32()
    m1.data = 0
    m2.data = 0

    return m1,m2
# show image stream that is coming from a topic in OpenCv
def callback(data):
    """this is triggered each time data is recieved by the subscriber.
    It should process the image, show the image, and publish robot
    motor commands to the appropriate topics."""
    #these below functions decompress the image into a
    np_arr = np.fromstring(data.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    proc_img = imageProcessing(img)
    m1,m2 = visionToMotors(proc_img)
    mot1.publish(m1)
    mot2.publish(m2)
    cv2.imshow('image',proc_img)
    cv2.waitKey(2)




# initialize the node. This text is only for the purposes of 'your reference'.
rospy.init_node('CLcontrol_{}'.format(robotname))
#here we are binding our callback function, above, to a topic by the name of 'topic_image'.
#every time a message is published to that topic, callback will be called and the message
#will be passed to it.
img_sub = rospy.Subscriber(topic_image, CompressedImage, callback,queue_size=1)
#this next part just makes everything work. Magic!
rospy.spin()
