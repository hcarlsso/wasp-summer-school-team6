#!/usr/bin/python

########################################################################################
#
# |B|I|G| |F|A|C|E| |R|O|B|O|T|I|C|S|
#
# HSV Colour selector for object detection using OpenCV
#
#
# Author : Peter Neal
#
# Date : 17 March 2015
# Last Update : 17 March 2015
#
########################################################################################

#!/usr/bin/env python
from __future__ import division
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import time
from geometry_msgs.msg import PointStamped
from object_detecter_2d.msg import object_loc

import tf
from std_msgs.msg import String
from PIL import Image


def nothing(x):
    pass
cv2.namedWindow("camera", 0)
cv2.resizeWindow("camera", 640,480) 
cv2.moveWindow("camera", 400,30)
cv2.waitKey(50)
print 'Create trackbars'
cv2.createTrackbar('HLow','camera',0,255,nothing)
cv2.createTrackbar('SLow','camera',0,255,nothing)
cv2.createTrackbar('VLow','camera',0,255,nothing)

cv2.createTrackbar('HHigh','camera',0,255,nothing)
cv2.createTrackbar('SHigh','camera',0,255,nothing)
cv2.createTrackbar('VHigh','camera',0,255,nothing)

cv_image = Image.new('RGB', (640, 480))

def callback(data):
    np_arr = np.fromstring(data.data, np.uint8)
    global cv_image
    cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    HLow = cv2.getTrackbarPos('HLow','camera')
    SLow = cv2.getTrackbarPos('SLow','camera')
    VLow = cv2.getTrackbarPos('VLow','camera')
    HHigh = cv2.getTrackbarPos('HHigh','camera')
    SHigh = cv2.getTrackbarPos('SHigh','camera')
    VHigh = cv2.getTrackbarPos('VHigh','camera')

    img = cv_image.copy()

    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV) #convert img to HSV and store result in imgHSVyellow
    lower = np.array([HLow, SLow, VLow]) #np arrays for upper and lower thresholds
    upper = np.array([HHigh, SHigh, VHigh])
    imgthreshed = cv2.inRange(imgHSV, lower, upper) #threshold imgHSV
    #imgthreshed = cv2.blur(imgthreshed,(3,3))
    cv2.imshow("View", img)
    cv2.imshow("camera", imgthreshed)
    cv2.waitKey(1)



def main(args):
    rospy.init_node('HSV_Picker', anonymous = False)
    image_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage, callback)
    '''
    while True:

        HLow = cv2.getTrackbarPos('HLow','camera')
        SLow = cv2.getTrackbarPos('SLow','camera')
        VLow = cv2.getTrackbarPos('VLow','camera')
        HHigh = cv2.getTrackbarPos('HHigh','camera')
        SHigh = cv2.getTrackbarPos('SHigh','camera')
        VHigh = cv2.getTrackbarPos('VHigh','camera')
        frame = cv_image.copy()
        img = frame.copy()
        imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV) #convert img to HSV and store result in imgHSVyellow
        lower = np.array([HLow, SLow, VLow]) #np arrays for upper and lower thresholds
        upper = np.array([HHigh, SHigh, VHigh])
        imgthreshed = cv2.inRange(imgHSV, lower, upper) #threshold imgHSV
        #imgthreshed = cv2.blur(imgthreshed,(3,3))
        cv2.imshow("View", img)
        cv2.imshow("camera", imgthreshed)
        cv2.waitKey(1)
    '''
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)




 
