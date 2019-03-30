# import the necessary packages
import picamera
import picamera.array
import io
import time
import cv2
import rospy
from std_msgs.msg import Int32MultiArray
import numpy as np
import gopigo as go
import random

#this is just a way to specify topics between robots
botname = "be107bot1"

class GoPiGoBot():
    """this class allows us to contain movement-specific variables in a nice way"""
    def __init__(self,recordFreq=0.5,camera=None,stream=None):
        """this function intializes everything, including setting the speed"""
        go.set_right_speed(50)
        go.set_left_speed(50)
        go.enable_encoders()
        self.camera=camera
        self.stream=stream
        #starttime = time.time()
        #this will be our trail. absolute positions!!
        #leftmotor,rightmotor,time(sec)
        #self.resetRecord()
        #self.recordFreq = 0.5 #this is how often we record our position, in seconds!
    def move(self,dist,how="F"):
        """this takes care of moving the robot.
        how can be
        F: which is forward, dist is in cm
        L: which is left (right motor on left motor off), dist is in degrees
        R: which is right (left motor on right motor off), dist is in degrees
        B: which is backward.  dist is in cm
        TL: rotate in place, to the left (tank steering) dist is in pulses
        TR: rotate in place, to the right (tank steering) dist is in pulses
        """
        if(dist<30 and (how=="F" or how=="B")):
            #If dist <30 the robot will
            #try to move 0 distance which means go for ever. Bad news!
            return -1
        elif(dist< 8 and (how=="L" or how=="R")):
            #also the robot rotates for ever if the angle is less than 8
            return -1
        else:
            #take an initial record
            #self.recordEncoders()
            #save the current position because we're about to reset it
            #self.keyframe = self.timerecord[-1][:-1]
            #basically we are recording our position while moving.
            #prevtime = int(time.time())
            #go forward! this runs in the background pretty much
            if(how=="F"):
                go.fwd(dist)
            elif(how=="L"):
                go.turn_left(dist)
            elif(how=="R"):
                go.turn_right(dist)
            elif(how=="B"):
                go.bwd(dist)
            elif(how=="TR"):
                go.enc_tgt(0,1,dist)
                go.right_rot()
            elif(how=="TL"):
                go.enc_tgt(0,1,dist)
                go.left_rot()
            #record while we haven't reached our destination
            #while(go.read_enc_status()):
                #this resets both encoders so we start counting from zero!
                #this next part should only trigger once per recordFreq
                #if(time.time()-prevtime>self.recordFreq):
                    #prevtime = time.time() #make sure to reset the timer...
                    #tell the recordEncoders function which direction
                    #we are going
                    #dir = "forward"
                    #if(how=="B"):
                    #    dir = "backward"
                    #self.recordEncoders(dir=dir)
            return 1

#have to initialize the node which will send the data
rospy.init_node('talker',anonymous=True)
#the publisher is what will actually be sending the data, but a node had
#to exist first
avgpub = rospy.Publisher("/{}/image_average".format(botname),\
                            Int32MultiArray,tcp_nodelay=True,queue_size=10)

#the with statement allows clean initializing and demolishing of objects. In
#this case we are going to have a camera object and once we're done we want to
#free the object so it is available to other programs. That happens automatically
#at the end of the with statement
def getAverageColor(camera,stream):
    camera.capture(stream, format='jpeg', use_video_port=True)
    #now, we convert the stream into numpy data. First we get
    #the stream data into an array, then decode that array into a
    #different array of RGB values. Actually the values are BGR
    data = np.fromstring(stream.getvalue(), dtype=np.uint8)
    img = cv2.imdecode(data, 1)
    #this averages over the array in two dimensions, since we are averaging
    #the entire image.
    average = img.mean(axis=0).mean(axis=0)
    stream.seek(0)
    stream.truncate()
    return average
if(__name__=="__main__"):
    #above means we can import this and it wont run this code
    with picamera.PiCamera() as camera:
        #here we are defining the camera parameters. Sad that we are then
        #averaging the image into a single value afterwards getting a 640x480 image.
        camera.resolution = (640, 480)
        camera.framerate = 32
        #we have to wait a bit for the camera to intialize before capturing frames
        time.sleep(0.5)
        #now we are defining the stream which will take in image data
        stream = io.BytesIO()
        myFlyBot = GoPiGoBot(1.0,camera,stream)
        #this for loop just makes sure that we only do this for 100 frames.
        #you can do it indefinitely but I had problems stopping the program
        #when it was working like that.
        robotdirection=1
        for x in range(100):
            #we are using the capture method, and instead of feeding it
            #to a file we feed it to 'stream'. use_video_port means that we are
            #recording a video and not taking stills. That makes it faster but more
            #noisy and have less pixels.

            #this next part again reshapes the image array so it kinda looks
            #like the image. A grid of 640x480 and with three elements in each cell for
            #bgr
            #pixels = np.float32(img.reshape(-1, 3)) #i think this does nothing since
                                                    #it's not used
            #average = myFlyBot.searchPattern()
            #myFlyBot.move
            average = getAverageColor(camera,stream)
	    
	    norm_red = float(average[2])/sum(average)
	    print(norm_red)
	    dist = 3
            if(norm_red > 0.60):
		if random.random() < 0.90:
                	robotdirection=robotdirection*-1
			dist = dist/2
	    else:
		if random.random() < 0.1:
			robotdirection=robotdirection*-1
            if(robotdirection >0):
                myFlyBot.move(dist,"TL")
            else:
                myFlyBot.move(dist,"TR")
            #now, we initialize the data structure we will use for publishing to
            #the node.
            avgpubarray = Int32MultiArray()
            #the data member contains an array of integers. That's sad actually
            #because the image data is floats. Oh well it gets converted.
            avgpubarray.data = average
            #now we publish the average color
            avgpub.publish(avgpubarray)
            #this next part clears the stream to prepare it for the next frame
            stream.seek(0)
            stream.truncate()
            time.sleep(0.2)
