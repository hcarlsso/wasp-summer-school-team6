#!/usr/bin/env python
'''
This source will act as support to finish your project and does not follow best
coding practices.
'''
#Import Python Packages, ROS messages
from __future__ import print_function
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
from sensor_msgs.msg import Image
#import the custom message we created to store objects
from wasp_custom_msgs.msg import object_loc
import tf
from math import hypot

from itertools import cycle
import random


#Define Constants
random.seed(0)

#Focal Length of the Asus Prime sensor camera
focal_leng = 570.34222

#This may change during the competetion, need to be calibrated
square_side_lenth = 0.115 #in mts

rgb_threshold = {
        'blue'  : [
                {
                        'low' : np.array([99,70, 57]),
                        'high' : np.array([162,255,242]),
                },
        ],
        'green' : [
                {
                        'low' : np.array([64,76,70]),
	                'high' : np.array([112,148,111]),
                }
        ],
        'red' : [
                {
                        'low' : np.array([0, 60, 122]),
                        'high' : np.array([92, 244,181]),
                },
                # Hard to get calibration
                {
                        'low' : np.array([140,36,90]),
                        'high' : np.array([255,230,200]),
                }
        ],
}

# Define colors
COLORS_START = 0
COLORS_END = 255
NUMBER_COLORS = 2000
colors_contours = [(random.randint(COLORS_START, COLORS_END), random.randint(COLORS_START, COLORS_END),random.randint(COLORS_START, COLORS_END) ) for i in range(NUMBER_COLORS) ]
                


def threshold_image(hsv, data):
        '''
        Threshold image with multiple thresholds
        '''

        mask_return = None
        for threshold in data:

                # Threshold the HSV image to get only single color portions
                mask = cv2.inRange(hsv, threshold['low'], threshold['high'])

                if mask_return is None:
                        mask_return = mask
                else:
                        # Assume matrix is one and zeros
                        mask_return = cv2.bitwise_or(mask_return, mask)

        return mask_return

def find_masks_rgb(hsv):
        '''
        May have to  run each color on separate node, thread to increase speed.
        '''
        masks = {}
        for color, data in rgb_threshold.iteritems():
                masks[color] = threshold_image(hsv, data)
                        
	return masks
        

#This function finds the lengths of all the sides and estimates the longest.
def Longest_Length(approxcontour):
	#add the first element in the end to complete the loop
	approxcontour = np.concatenate((approxcontour,[approxcontour[0]]))
	#The below lines find the length between two adjacent points
	#and append them in  an array
	ptdiff = lambda (p1,p2): (p1[0]-p2[0], p1[1]-p2[1])
	diffs = map(ptdiff, zip(approxcontour,approxcontour[1:]))
	dist = []
	for d in diffs:
		dist.append(hypot(*d))
	#find maximum of lenghts found
	LongestSide = max(dist)
	return LongestSide

class ContourDetection:

        def __init__(self):
                pass
                
        def draw_contours(self, mask, cv_image, area_threshold):
                '''
                Find contours(borders) for the shapes in the image
                
                '''

                contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

                # Filter out the smaller ones,  can do genertor
                #Discard contours with a small area as this may just be noise
                contours_filtered = [x for x in contours if cv2.contourArea(x) > area_threshold]
        

	        #Pass through each contour and check if it has required properties to classify into required object
	        for i,x in enumerate(contours_filtered):
		
		        #The below 2 functions help you to approximate the contour to a nearest polygon
		        arclength = cv2.arcLength(x, True)
		        approxcontour = cv2.approxPolyDP(x, 0.02 * arclength, True)

		        # cv2.drawContours(cv_image,contours[x],0,(0,255,255),2)
                        # Draw different colors for the shapes
		        cv2.drawContours(cv_image,[approxcontour],0, colors_contours[i] ,2)

                        # Simple generator
                        yield x
                        
#This is the main class for object detection, it has some initializations about nodes
#Call back functions etc
class object_detection:
	def __init__(self):
		#Create Rospy Publisher and subscriber
		self.object_location_pub = rospy.Publisher("/object_location", object_loc, queue_size =1)
		#original images is huge and creates lot of latency, therefore subscribe to compressed image

		self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)
		#Cv Bridge is used to convert images from ROS messages to numpy array for openCV and vice versa
		self.bridge = CvBridge()
		#Obejct to transform listener which will be used to transform the points from one coordinate system to other.

                self.cd = ContourDetection()
	#Callback function for subscribed image
        def publish_coordinates(self, contour):

                #Find the coordinates of the polygon with respect to he camera frame in pixels
                rect_cordi = cv2.minAreaRect(contour)
		obj_x = int(rect_cordi[0][0])
		obj_y = int(rect_cordi[0][1])

                # Temp distance
                Distance = 10
                #Calculate Cordinates wrt to Camera, convert to Map
		#Coordinates and publish message for storing
		#319.5, 239.5 = image centre
		obj_cam_x = ((obj_x - 319.5)*Distance)/focal_leng
		obj_cam_y = ((obj_y - 239.5)*Distance)/focal_leng

		#convert the x,y in camera frame to a geometric stamped point
		P = PointStamped()
		P.header.stamp = rospy.Time.now() - rospy.Time(23)
		#print ('time: ', data.header.stamp)
		P.header.frame_id = 'camera_rgb_optical_frame'
		P.point.x = obj_cam_x
		P.point.y = obj_cam_y
		P.point.z = Distance
                
		#Transform Point into map coordinates
		# trans_pt = self.tl.transformPoint('/map', P)

		#fill in the publisher object to publish
		obj_info_pub = object_loc()
		obj_info_pub.ID = 27 #ID need to be changed
		# obj_info_pub.point.x = trans_pt.point.x
		# obj_info_pub.point.y = trans_pt.point.y
		# obj_info_pub.point.z = trans_pt.point.z

                # Temp
                obj_info_pub.point.x = P.point.x
		obj_info_pub.point.y = P.point.y
		obj_info_pub.point.z = P.point.z

		#publish the message
		self.object_location_pub.publish(obj_info_pub)
        
	def callback(self,data):
		#The below two functions conver the compressed image to opencv Image

                # The AR drone 
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		#Create copy of captured image
		img_cpy = cv_image.copy()
		#Color to HSV and Gray Scale conversion
		hsv = cv2.cvtColor(img_cpy, cv2.COLOR_BGR2HSV)
		#gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		
                masks = find_masks_rgb(hsv)

                for color, mask in masks.iteritems():
                        figures = self.cd.draw_contours(mask, cv_image, 600)
                        for c in  figures:
                                self.publish_coordinates(c)


		#Display the captured image
		cv2.imshow("Image",cv_image)
		#cv2.imshow("HSV", hsv)
		cv2.waitKey(1)
                
        

#Main function for the node
def main(args):
	rospy.init_node('object_detection', anonymous = False)
	ic = object_detection()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting Down object_detection Node')
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
