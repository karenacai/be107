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
robotname="be107bot4"
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
    cropped_img = img[img.shape[0]*2/5:img.shape[0]*3/5,0:img.shape[1]]
    img_gray = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(img_gray, 65, 255, cv2.THRESH_BINARY)
    edges = cv2.Canny(thresh, 50, 150, apertureSize = 3)


    #edges = edges[int(img_len*3.0/4.0):img_len, 0:img_width]
    return edges

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
    imwidth = img.shape[1]
    midpt = imwidth/2
    #m1 and m2 are ros messages and so have a funky format. First
    #you define their type as Int32 and then set their data equal
    #to the int itself. It's annoying I know, but it's the way it works.
    rho = 1  # distance resolution in pixels of the Hough grid
    theta = np.pi / 180  # angular resolution in radians of the Hough grid
    threshold = 15  # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 50  # minimum number of pixels making up a line
    max_line_gap = 20  # maximum gap in pixels between connectable line segments
    line_image = np.copy(img) * 0  # creating a blank to draw lines on

    # Run Hough on edge detected image
    # Output "lines" is an array containing endpoints of detected line segments
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]),
                        min_line_length, max_line_gap)
    slopes = []
    if(type(lines) == type(None)):
        lines = []
    linepos = []
    if(len(lines)>0):
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)
                slopes += [[x2-x1,y2-y1]]
                linepos += [[x2]]
    if(lines == []):
        avgpos = midpt
    else:
        avgpos = np.mean(linepos)
    p = (midpt-avgpos)/imwidth
    print("midpt is {}".format(midpt))
    # Draw the lines on the  image
    #lines_edges = cv2.addWeighted(img, 0.8, line_image, 1, 0)



    #avg_right = img[0:img.shape[0],0:image.shape[1]/2].mean(axis=1).mean(axis=0)
    #avg_left = img[0:img.shape[0],0:image.shape[1]/2].mean(axis=1).mean(axis=0)
    #lfrac = float(avg_left)/(avg_left+avg_right)
    m1 = Int32()
    m2 = Int32()
    print("p = {}".format(p))
    normalspeed = .2*m1max
    m1.data = normalspeed+p*normalspeed/2
    #int(m1max*lfrac)
    m2.data = normalspeed-p*normalspeed/2
    #int(m2max*(1.0-lfrac))
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
    #print(slopes,avgpos)
    #now we convert the processed image into motor motion
    m1,m2 = visionToMotors(proc_img)
    #finally we publish the motor motion to the specified topics.
    #the names of these topics are indicated far above!
    mot1.publish(m1)
    mot2.publish(m2)
    #finally finally, we show the image! this should be a video.
    cv2.imshow('image',proc_img)
    #it's kinda slow. I'm not sure why this is. Possibly the rate of image capture
    #in the robot half of the program? It isn't using a video codec so that could
    #also be it. It's sufficient for our purposes!!
    #this waitkey business rears it's ugly head once more.
    cv2.waitKey(2)




# initialize the node. This text is only for the purposes of 'your reference'.
rospy.init_node('image_processing_for_{}'.format(robotname))
#here we are binding our callback function, above, to a topic by the name of 'topic_image'.
#every time a message is published to that topic, callback will be called and the message
#will be passed to it.
img_sub = rospy.Subscriber(topic_image, CompressedImage, callback,queue_size=1)
#this next part just makes everything work. Magic!
rospy.spin()
