import PyCapture2
import numpy as np
import cv2

class FlyCamera():
    """this class does all the gritty image capture stuff you need
    to do to get data from the pointgrey cameras"""
    def __init__(self,camIndex=0):
        """this is all the stuff you need to do to initialize the camera. It
        should work for multiple cameras as long as you change the camIndex"""
        bus = PyCapture2.BusManager()
        numCams = bus.getNumOfCameras()
        self.camera = PyCapture2.Camera()
        uid = bus.getCameraFromIndex(0)
        self.camera.connect(uid)
        self.camera.startCapture()
    def read(self):
        """the data from the camera comes in a weird format, and we need to convert
        it into something that openCV can understand. actually we are converting
        a grayscale image into a color image and then back to a grayscale image when
        we tell openCV to track something in the image, but what-ever"""
        image = self.camera.retrieveBuffer()
        imdata = image.getData()
        row_bytes = float(len(imdata)) / float(image.getRows());
        grayimg = np.array(imdata, dtype="uint8").\
                        reshape((image.getRows(), image.getCols()) );
        outimg = cv2.cvtColor(grayimg, cv2.COLOR_BAYER_BG2BGR)
        #that "1" we are returning is what tells opencv that our image fetching actually worked.
        return 1,outimg
