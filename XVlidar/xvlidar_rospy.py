#!/usr/bin/python
import serial
import time
import math
import rospy
import socket
from sensor_msgs.msg import LaserScan


hostname = socket.gethostname()
rospy.init_node('laser_scan_publisher')
scan_pub = rospy.Publisher('/{}/scan'.format(hostname), LaserScan, queue_size=50)
num_readings = 100
laser_frequency = 40
count = 0
r = rospy.Rate(1.0)

# Some settings and variables
outfile = open("outfile.txt", "w+")
print("Start")

f = serial.Serial(port='/dev/ttyS0',
                            baudrate=115200,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            timeout=0)

def decode_string(string):
    #print string
    data = []

    for byte in string.strip("\n").split(":")[:21]:
        data.append(int(byte,16))

    start = data[0]
    idx = data[1] - 0xa0
    speed = float(data[2] | (data[3] << 8)) / 64.0
    in_checksum = data[-2] + (data[-1] << 8)

    # first data package (4 bytes after header)
    angle = idx*4 + 0
    angle_rad = angle * math.pi / 180.
    dist_mm = data[4] | ((data[5] & 0x1f) << 8)
    quality = data[6] | (data[7] << 8)

    if data[5] & 0x80:
         #print "X - ",
         pass
    else:
        #print "O - ",
        pass
    if data[5] & 0x40:
         print "NOT GOOD"

    #print "Speed: ", speed, ", angle: ", angle, ", dist: ",dist_mm, ", quality: ", quality
    #print "Checksum: ", checksum(data), ", from packet: ", in_checksum
    outfile.write(string+"\n")
    return speed,angle,dist_mm,quality)

byte = f.read(1)
started = False
string = "Start"
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    if(count == 0 ):
        scan = LaserScan()
        scan.header.stamp = current_time
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -3.14
        scan.angle_max = 3.14
        scan.angle_increment = 3.14*2 / num_readings
        scan.time_increment = (1.0 / laser_frequency) / (num_readings)
        scan.range_min = 0.0
        scan.range_max = 100.0
        scan.ranges = [0]*num_readings
        scan.intensities = [0]*num_readings
    if byte != '':
        enc = (byte.encode('hex') + ":")
        if enc == "fa:":
            if started:
                try:
                    speed, angle, dist_mm, quality = decode_string(string)
                    index = int(angle*(2*3.14/360.0)/scan.angle_increment)
                    scan.ranges[index]=dist_mm
                    scan.intensities[index]=quality
                    count+=1
                except Exception, e:
                    print e

            started = True
            string = "fa:"
        elif started:
            string += enc
        else:
            print "Waiting for start"
    if(count>= num_readings):
        scan_pub.publish(scan)
        count = 0
    byte = f.read(1)
    r.sleep()
outfile.close()
print("End")
