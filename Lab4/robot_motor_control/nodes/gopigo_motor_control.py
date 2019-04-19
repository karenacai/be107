import roslib, rospy
from std_msgs.msg import Int32
import sys
import socket
import atexit
if sys.version_info[0] < 3:
    errorcode = ImportError
else:
    errorcode = ModuleNotFoundError




class Robot:
    def __init__(self,maxmot1=350,maxmot2=350,deadzone = 10,gpg=None):
        self.maxmot1 = maxmot1
        self.maxmot2 = maxmot2
        self.deadzone = 10
        self.gpg = gpg
        if(self.gpg==None):
            go.set_right_speed(maxmot1)
            go.set_left_speed(maxmot2)
        else:
            self.gpg.set_speed(maxmot1)
    def motor(self,speed,motnum):
        """this takes care of setting the motor speed"""
        #the direction just tells us whether the number is negative or
        #positive
        print("motor {}".format(motnum))
        print(int(speed))
        direction=speed>0
        #the following prevents HOOLIGANS
        if(speed > 255):
            speed = 255
        if(speed < -255):
            speed = -255
        if(abs(speed)<self.deadzone):
            #if the speed we want is low then just stop
            speed = 0
        #now we are actually scaling the speed to what we said
        #the maximum speed was going to be.
        scaledspeed = int((speed/255)*self.maxmot1)
        absspeed = abs(scaledspeed)
        if(self.gpg==None):
            if(motnum==1):
                go.motor1(absspeed,direction)
            elif(motnum==2):
                go.motor2(absspeed,direction)
        else:
            if(motnum==1):
                self.gpg.set_motor_dps(1,scaledspeed)
            elif(motnum==2):
                self.gpg.set_motor_dps(2,scaledspeed)
        return True
    def motors(self,speed1,speed2):
        self.motor(speed1,1)
        self.motor(speed2,2)

try:
    import gopigo as go
    mybot = Robot()
except errorcode:
    import easygopigo3 as easy
    mybot = Robot(maxmot1 = 250,gpg=easy.EasyGoPiGo3())


except errormsg:
    import easygopigo3 as easy

hostname = socket.gethostname()

topic_motor1 = "/{}/motor1".format(hostname)
topic_motor2 = "/{}/motor2".format(hostname)

def motor_callback(velocity,motnum):
    mybot.motor(velocity,motnum)
def stop_motors():
    mybot.motors(0,0)

motor1_sub = rospy.Subscriber(topic_motor1, Int32, \
                            lambda a: motor_callback(int(a.data),1))
motor2_sub = rospy.Subscriber(topic_motor2, Int32, \
                            lambda a: motor_callback(int(a.data),2))

# Init and Run
rospy.init_node('rospigo_motor_control', anonymous=True)
atexit.register(stop_motors)
rospy.spin()
