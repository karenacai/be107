#!/usr/bin/env python
import rospy
import cv2
import sys
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
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
pub = rospy.Publisher('/{}/image'.format(hostname), CompressedImage, queue_size=100)
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
                #picamera needs to have the resolution and framerate set
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
    #first we read an image from the camera
    ret1, origimg = cam.read()
    #we copy the image to prevent mangling the original
    img = origimg.copy()
    if(True):
        #don't mind me.. just some mess
        try:
            #we're going to send a CompressedImage message
            msg = CompressedImage()
            #add time stamp
            msg.header.stamp = rospy.Time.now()
            #what format to compress into
            msg.format = "jpeg"
            #now we actually compress the image. Well that's what 'imencode' does.
            #ros messages are actually strings so then we convert the compressed
            #image into a string!!! it's quite huge that way. This might end up 
            #badly when all the robots are sending these things around. We'll see...
            msg.data = np.array(cv2.imencode('.jpg',img)[1]).tostring()
            #once prepared, then publish!
            pub.publish(msg)
        except CvBridgeError as e:
            #actually this should never happen since we aren't using cvbridge any more
            print("error!")
            print(e)
