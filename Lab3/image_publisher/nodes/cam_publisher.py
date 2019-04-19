#!/usr/bin/env python
import rospy
import cv2
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import socket
import io
if sys.version_info[0] < 3:
    errorcode = ImportError
else:
    errorcode = ModuleNotFoundError
#we're going to label our topic with our hostname
#so we don't get confused between all the other robots
#that will be publishing their images
hostname = socket.gethostname()
# set up publisher node to take camera input and output to ROS camera topic
pub = rospy.Publisher('/{}/image'.format(hostname), Image, queue_size=100)
rospy.init_node('image_publisher', anonymous=True)
bridge = CvBridge()


cam = None
#this should work for publishing images from a webcam or from
#the firefly camera or from the raspberry pi camera.
try:
    #first let's see if we have a point grey firefly camera
    import cam_capture
    cam = cam_capture.FlyCamera(0)
except errorcode:
    #if we don't have a point grey firefly camera then this will
    #trigger.
    try:
        import picamera
        import picamera.array
        #we're going to define a new class in this try except block
        #to make the picamera behave the same as a default camera from
        #opencv's perspective
        class dummyCam:
            def __init__(self,camera,res=(640,480),framerate=32):
                self.cam = camera
                camera.resolution = res
                camera.framerate = framerate
                #we need a stream to get the image data. Not sure why but
                #this works the fastest
                self.stream = io.BytesIO()
            def read(self):
                #this part captures an image and adds it to the stream
                self.cam.capture(self.stream, format='jpeg', use_video_port=True)
                #now, we convert the stream into numpy data. First we get
                #the stream data into an array, then decode that array into a
                #different array of RGB values. Actually the values are BGR
                data = np.fromstring(self.stream.getvalue(), dtype=np.uint8)
                img = cv2.imdecode(data, 1)
                #this next part removes the current image from the stream
                #to prepare it for the next image.
                self.stream.seek(0)
                self.stream.truncate()
                return True, img
            def __del__(self):
                #this part runs when this class is destroyed. This will ensure
                #that our program does not hog the camera for ever and ever
                self.cam.close()
        #after defining the class we initialize the class.
        cam = dummyCam(picamera.PiCamera())
    except errorcode:
        #finally if neither of those two things worked, then use
        #the webcam!
        cam = cv2.VideoCapture(0)
while(True):
    ret1, origimg = cam.read()
    img = origimg.copy()
    if(int(time.time())%2==0):
        try:
            pub.publish(bridge.cv2_to_imgmsg(img))
        except CvBridgeError as e:
            print("error!")
            print(e)
