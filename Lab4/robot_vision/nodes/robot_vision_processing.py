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
robotname="be107bot1"
#we fill in our topic names with the appropriate robot name
#MAKE SURE YOU FIXED IT, ABOVE.
topic_motor1 = "/{}/motor1".format(robotname)
topic_motor2 = "/{}/motor2".format(robotname)
topic_image = '/{}/image'.format(robotname)
print(topic_image)

#here we will actually define the publisher objects. They will publish
#Int32 data, which is just an integer that has a fairly high maximum (absolute)
#value. Much higher than we will ever use. Queue size means that messages that have not
#been read by the robot are held in reserve and sent later. You can decrease this and
#the robot should respond to more immediate information, although in my testing
#everything was pretty immediate with queue_size=10.
mot1 = rospy.Publisher(topic_motor1,Int32,queue_size=10)
mot2 = rospy.Publisher(topic_motor2,Int32,queue_size=10)


# input is a color image, output should be a processed image where it is easy to extract the coordinates of the fly
def imageProcessing(img):
    """this function performs some image modification techniques 
    such as background subtraction or pattern matching"""
    # insert your image processing algorithm here
    return img

# input is the processed image, output is the motor commands
def visionToMotors(img):
    """this function is given a processed image, and outputs a 
    directive for robot locomotion. m1 and m2 are Int32 data 
    members where a value between -300 and 300 controls motor 
    motion. 300 is full forward, -300 is full reverse. 
    Intermediate values result in slower motor rotation."""
    #here you will do something to convert the image into motion!
    #we specify max values. These are absolute values. Do with them
    #what you wish!
    m1max = 300
    m2max = 300
    #m1 and m2 are ros messages and so have a funky format. First
    #you define their type as Int32 and then set their data equal 
    #to the int itself. It's annoying I know, but it's the way it works.
    m1 = Int32()
    m2 = Int32()
    m1.data = 0
    m2.data = 0
    #you may wish to output a further processed image here, or mangle the
    #input image. For example maybe you want to draw some lines or squares
    #on that image to indicate that you identified something or
    #to denote which half of the image is brighter or something.
    #check out this: https://docs.opencv.org/3.1.0/dc/da5/tutorial_py_drawing_functions.html
    return m1,m2
# show image stream that is coming from a topic in OpenCv
def callback(data):
    """this is triggered each time data is recieved by the subscriber. 
    It should process the image, show the image, and publish robot 
    motor commands to the appropriate topics."""
    #these below functions decompress the image into a
    #numpy array suitable for opencv
    np_arr = np.fromstring(data.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #next we process the image. Define this in imageProcessing, above!
    proc_img = imageProcessing(img)
    #now we convert the processed image into motor motion
    m1,m2 = visionToMotors(proc_img)
    #finally we publish the motor motion to the specified topics. 
    #the names of these topics are indicated far above!
    #mot1.publish(m1)
    #mot2.publish(m2)
    #finally finally, we show the image! this should be a video.
    cv2.imshow('image',proc_img)
    #it's kinda slow. I'm not sure why this is. Possibly the rate of image capture 
    #in the robot half of the program? It isn't using a video codec so that could
    #also be it. It's sufficient for our purposes!!
    #this waitkey business rears it's ugly head once more.
    cv2.waitKey(2)
    



# initialize the node. This text is only for the purposes of 'your reference'.
rospy.init_node('image_processing_node')
#here we are binding our callback function, above, to a topic by the name of 'topic_image'.
#every time a message is published to that topic, callback will be called and the message
#will be passed to it.
img_sub = rospy.Subscriber(topic_image, CompressedImage, callback,queue_size=1)
#this next part just makes everything work. Magic!
rospy.spin()
