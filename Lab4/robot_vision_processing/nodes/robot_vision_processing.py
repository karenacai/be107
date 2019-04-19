import roslib, rospy
from std_msgs.msg import Int32
import sys
import socket
import atexit
import curses
from sensor_msgs.msg import Image
hostname = socket.gethostname()
if sys.version_info[0] < 3:
    errorcode = ImportError
else:
    errorcode = ModuleNotFoundError
robotname="be107bot8"
topic_motor1 = "/{}/motor1".format(robotname)
topic_motor2 = "/{}/motor2".format(robotname)
topic_image = '/{}/image'.format(robotname)

mot1 = rospy.Publisher(topic_motor1,Int32,queue_size=10)
mot2 = rospy.Publisher(topic_motor2,Int32,queue_size=10)


# input is a color image, output should be a processed image where it is easy to extract the coordinates of the fly
def imageProcessing(img):
    # insert your image processing algorithm here
    return img
def crop_right_half(image):
    cropped_img = image[image.shape[1]/2:image.shape[1]]
    return cropped_img
def crop_left_half(image):
    cropped_img = image[0:image.shape[1]/2]
    return cropped_img
# input is the processed image, output is the motor commands
def visionToMotors(img):
    #data = np.fromstring(stream.getvalue(), dtype=np.uint8)
    #img = cv2.imdecode(data, 1)
    #this averages over the array in two dimensions, since we are averaging
    #the entire image.
    m1max = 300
    m2max = 300
    avg_right = crop_right_half(img).mean(axis=1).mean(axis=0)
    avg_left = crop_left_half(img).mean(axis=1).mean(axis=0)
    lfrac = float(avg_left)/(avg_left+avg_right)
    m1 = Int32()
    m2 = Int32()
    m1.data = int(m1max*lfrac)
    m2.data = int(m2max*(1/lfrac))
    return m1,m2
# show image stream that is coming from a topic in OpenCv
def callback(data):
    try:
        img = bridge.imgmsg_to_cv2(data).copy()
    except CvBridgeError as e:
        print(e)

    proc_img = imageProcessing(img)
    rightimg = crop_right_half(proc_img)
    m1,m2 = visionToMotors(proc_img)
    mot1.publish(m1)
    mot2.publish(m2)
    cv2.imshow('image',rightimg)
    cv2.waitKey(2)
    # write the position data to file



# initialize camera subscriber
rospy.init_node('image_processing_node')
img_sub = rospy.Subscriber(topic_image, Image, callback)
bridge = CvBridge()
rospy.spin()
