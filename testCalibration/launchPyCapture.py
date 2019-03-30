import PyCapture2
import numpy as np


class FlyCamera():
    def __init__(self,camIndex=0):
        bus = PyCapture2.BusManager()
        numCams = bus.getNumOfCameras()
        self.camera = PyCapture2.Camera()
        uid = bus.getCameraFromIndex(0)
        self.camera.connect(uid)
        self.camera.startCapture()
    def read(self):
        image = self.camera.retrieveBuffer()
        row_bytes = float(len(image.getData())) / float(image.getRows());
        outimg = np.array(image.getData(), dtype="uint8").\
                        reshape((image.getRows(), image.getCols()) );
        return 1,outimg
