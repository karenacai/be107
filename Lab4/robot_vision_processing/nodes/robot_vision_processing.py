import roslib, rospy
from std_msgs.msg import Int32
import sys
import socket
import atexit
import curses
hostname = socket.gethostname()
if sys.version_info[0] < 3:
    errorcode = ImportError
else:
    errorcode = ModuleNotFoundError

topic_motor1 = "/{}/motor1".format(hostname)
topic_motor2 = "/{}/motor2".format(hostname)

mot1 = rospy.Publisher(topic_motor1,Int32,queue_size=10)
mot2 = rospy.Publisher(topic_motor2,Int32,queue_size=10)


rospy.init_node('talker',anonymous=True)
rate=rospy.Rate(10)

def main(win):
    win.nodelay(True)
    key=""
    motorspeed = [0,0]
    motorincrement = 20
    motordelta = [0,0]
    win.clear()
    win.addstr("WASD to move")
    while 1:
        motordelta = [0,0]
        try:
            key = win.getkey()
            secondkey = ''
            try:
                secondkey = win.getkey()
            except Exception as e:
                pass
            win.clear()
            if('w' in (key+secondkey)):

                motordelta[0] += motorincrement
                motordelta[1] += motorincrement
            if('a' in (key+secondkey)):
                motordelta[0] += motorincrement
            if('d' in (key+secondkey)):
                motordelta[1] += motorincrement
            if('s' in (key+secondkey)):
                motordelta[0] -= motorincrement
                motordelta[1] -= motorincrement
            win.addstr(str(motordelta))
            if(key==os.linesep):
                break
            #motor1dir = (motordelta[0]>=0)*2-1
            #motor1speed = abs(motordelta[0])
            #motor2dir = (motordelta[1]>=0)*2-1
            #motor2speed = abs(motordelta[1])
            m1data = Int32()
            m2data = Int32()
            m1data.data = motordelta
            #[motor1dir,motor1speed]
            m2data.data = motordelta
            #[motor2dir,motor2speed]
            mot1.publish(m1data)
            mot2.publish(m2data)
        except Exception as e:
            m1data = Int32()
            m2data = Int32()
            m1data.data = 0
            m2data.data = 0
            mot1.publish(m1data)
            mot2.publish(m2data)
        rate.sleep()
curses.wrapper(main)
