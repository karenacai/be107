import cv2
import sys
import numpy as np
import time
import os

img = cv2.imread(os.path.join('antImages','antimage1.png'),0) #this will import the image.

# insert your image processing data here

cv2.imshow('image',img) #when you are ready you create a window to show the image. 
#this needs to have a title (that's the first argument) and the image itself
cv2.waitKey(0) #this part makes it so if any key is pressed, the window will close. 
#Convenience! Also sometimes opencv won't actually show the window unless you have this.
cv2.destroyAllWindows() #just making sure everything's been cleaned up.
