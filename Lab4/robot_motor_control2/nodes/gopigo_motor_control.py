#!/usr/bin/env python
import roslib, rospy
from std_msgs.msg import Int32
import sys
import socket
import atexit
import time
#see description in "robot vision processing"
if sys.version_info[0] < 3:
    errorcode = ImportError
else:
    errorcode = ModuleNotFoundError



#we're going to define a class to handle the gopigo2 and gopigo3 robots.
class Robot:
    def __init__(self,maxmot1=350,maxmot2=350,deadzone = 10,gpg=None):
        """this class is intended to provide compatibility between gopigo2 and
        gopigo3 robots. Only max speed setting and individual motor control
        is implemented. For driving set distances and turning set angles, you'll
        have to use the functions specific to gopigo2 or gopigo3.
        See here for gopigo2: https://github.com/DexterInd/GoPiGo
        or here for gopigo3:https://github.com/DexterInd/GoPiGo3
        """
        #we'll define a maximum value. GoPiGo3 only uses one maximum speed for
        #both motors, and that will be maxmot1.
        self.maxmot1 = maxmot1
        self.maxmot2 = maxmot2
        #deadzone means that if you tell the motor to spin
        #slower than some value, it will just send a 0. Maybe this
        #is too agressive for the gopigo2, since those motors have less lines
        #per revolution. Not sure! Maybe there should be a scaling factor between the two.
        self.deadzone = 10
        #for gopigo3 you need to define a gopigo object and run all the functions from
        #within that.
        self.gpg = gpg
        self.speedscaling = 1.0
        #the presence of this object is what will tell us whether we are dealing
        #with gopigo2 or 3.
        #setting the motor maximum speeds:
        if(self.gpg==None):
            #the old robots move twice as fast!! scale it down
            self.speedscaling = 0.5
            #go.set_right_speed(maxmot1)
            #go.set_left_speed(maxmot2)
        else:
            self.gpg.set_speed(maxmot1)
    def motor(self,speed,motnum,delay=50):
        """this takes care of setting the motor speed. Speed is a value from -255 to 255
        which gets scaled to whatever the maxmot values are. 255 or -255 is max in either
        direction and 0 (plus or minus the deadzone) is minimum"""
        #the direction just tells us whether the number is negative or
        #positive
        #print("motor {}".format(motnum))
        #print(int(speed))
        direction=speed>0
        #the following prevents HOOLIGANS by capping the maximum values
        if(speed > 255):
            speed = 255
        if(speed < -255):
            speed = -255
        if(abs(speed)<self.deadzone):
            #if the speed we want is low then just stop
            speed = 0
        #now we are actually scaling the speed to what we said
        #the maximum speed was going to be.
        scaledspeed = int((float(speed)/255)*self.maxmot1*self.speedscaling)
        print("scaled speed is {}".format(scaledspeed))
        #here we take the absolute value
        absspeed = abs(scaledspeed)
        #here we are setting the motor speeds differently depending
        #on which type of robot we are dealing with.
        if(self.gpg==None):
            #for gopigo2 we need to tell it a direction and
            #an absolute speed.
            if(int(time.time()*1000)%delay==0):
                if(motnum==1):
                    go.motor1(direction,absspeed)
                elif(motnum==2):
                    go.motor2(direction,absspeed)
        else:
            #gopigo3 accepts positive and negative values
            if(motnum==1):
                self.gpg.set_motor_dps(1,scaledspeed)
            elif(motnum==2):
                self.gpg.set_motor_dps(2,scaledspeed)
        return True
    def motors(self,speed1,speed2,delay=10):
        """wrapper function that allows setting both motors at once.
        It just runs the self.motor() function once for each motor"""
        self.motor(speed1,1,delay)
        self.motor(speed2,2,delay)
#here we are seeing which package is present. If one fails then try the other!
#of course the errorcode is different between python versions too and we account
#for that as well.
try:
    import gopigo as go
    mybot = Robot()
except errorcode:
    import easygopigo3 as easy
    #here we're setting the maximum speed to 250. That might be a bit low!
    mybot = Robot(maxmot1 = 250,gpg=easy.EasyGoPiGo3())

#see robot_vision_processing for description
hostname = socket.gethostname()

topic_motor1 = "/{}/motor1".format(hostname)
topic_motor2 = "/{}/motor2".format(hostname)

def motor_callback(velocity,motnum):
    """here we are converting the input velocity and motnum into
    actual motor motion. Actually this is just an extremely basic
    wrapper for mybot.motor(). I guess we could just use that!
    Well this is just here for clarity"""
    mybot.motor(velocity,motnum)
def stop_motors():
    """this function turns off both motors in case of robot escape"""
    mybot.motors(0,0,1)
#to define a subscriber we have to give it a _function_ object that takes one
#input. Since our motor callback takes two inputs (you have to tell it which
#motor you want to control), we will define a lambda function that takes
#one input and puts it into the first input of motor_callback. The second
#input is hardcoded within the lambda based on which subscriber we are
#talking about (motor1 or motor2). For an overview of lambda functions
#read here: https://www.afternerd.com/blog/python-lambdas/
motor1_sub = rospy.Subscriber(topic_motor1, Int32, \
                            lambda a: motor_callback(int(a.data),1))
motor2_sub = rospy.Subscriber(topic_motor2, Int32, \
                            lambda a: motor_callback(int(a.data),2))

# Init and Run
rospy.init_node('rospigo_motor_control', anonymous=True)
#this next part says we run the stop_motors function when this code exits.
#very useful to prevent run away robots!
atexit.register(stop_motors)
rospy.spin()
