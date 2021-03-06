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

from itertools import islice
import random
from pprint import pprint
import yaml


config_global =  '''
single_shapes:
  - SHAPE: STAR
    COLOR: RED
    ID: 1
  - SHAPE: TRIANGLE
    COLOR: BLUE
    ID: 2
  - SHAPE: SQUARE
    COLOR: GREEN
    ID: 3
  - SHAPE: SQUARE
    COLOR: RED
    ID: 4
  - SHAPE: CIRCLE
    COLOR: RED
    ID: 5
double_shapes:
  - SHAPE:
      - SQUARE
      - CIRCLE
    COLOR: RED
    ID: 6
  - SHAPE:
      - SQUARE
      - STAR
    COLOR: BLUE
    ID: 7
'''

#Define Constants
random.seed(0)
font = cv2.FONT_HERSHEY_SIMPLEX

#Focal Length of the Asus Prime sensor camera
focal_leng = 570.34222

#This may change during the competetion, need to be calibrated
square_side_lenth = 0.115 #in mts

rgb_threshold = {
        'BLUE'  : [
                {
                        'low' : np.array([99,70, 57]),
                        'high' : np.array([162,255,242]),
                },
        ],
        'GREEN' : [
                {
                        'low' : np.array([55,67,25]),
	                'high' : np.array([98,255,165]),
                }
        ],
        'RED' : [
                {
                        'low' : np.array([0, 60, 122]),
                        'high' : np.array([92, 244,181]),
                },
                # Hard to get calibration
                {
                        'low' : np.array([154,50,90]),
                        'high' : np.array([255,255,255]),
                }
        ],
}

# Define colors
COLORS_START = 0
COLORS_END = 255
NUMBER_COLORS = 2000
colors_contours = [(random.randint(COLORS_START, COLORS_END), random.randint(COLORS_START, COLORS_END),random.randint(COLORS_START, COLORS_END) ) for i in range(NUMBER_COLORS) ]
                

CIRCLE_CONTOUR_SHAPE = np.array([[[257, 220]],
                                 [[283, 223]],
                                 [[294, 238]],
                                 [[294, 253]],
                                 [[279, 270]],
                                 [[258, 270]],
                                 [[244, 256]],
                                 [[242, 237]]])

STAR_CONTOUR_SHAPE = np.array([[[450,  18]],
                               [[448,  47]],
                               [[416,  54]],
                               [[446,  61]],
                               [[457,  95]],
                               [[460,  64]],
                               [[491,  57]],
                               [[461,  49]]])

CONTOUR_SHAPE_TRIANGLE =  np.array([[[406,  46]],
                                    [[376, 111]],
                                    [[443, 112]]])

CONTOUR_SHAPE_SQAURE = np.array([[[376, 122]],
                                 [[378, 187]],
                                 [[446, 186]],
                                 [[444, 124]]])



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
                self.i = 1

                # with open('config_shapes.yaml', 'r') as f:
                config = yaml.load(config_global)

                pprint(config)
                self.allowed_singles = config['single_shapes']
                self.allowed_doubles = config['double_shapes']
                        
        def find_center_contour(self, contour):

                rect_cordi = cv2.minAreaRect(contour)
		obj_x = int(rect_cordi[0][0])
		obj_y = int(rect_cordi[0][1])

                return (obj_x, obj_y)

        def find_double_figure(self, info1 , info2):

                f1 = self.find_figure(info1)
                f2 = self.find_figure(info2)

                return f1 + '_' + f2
                
        def find_figure(self, contour):
                '''
                Send in the approx contour
                '''
                # Four sides
                (size, temp, temp2) = contour.shape
                if cv2.matchShapes(contour,CONTOUR_SHAPE_SQAURE,1,0.0) < 0.1:
                        return 'SQUARE'
                elif cv2.matchShapes(contour,CONTOUR_SHAPE_TRIANGLE,1,0.0) < 0.1:
                        return 'TRIANGLE'
                elif cv2.matchShapes(contour,CIRCLE_CONTOUR_SHAPE,1,0.0) < 0.1:
                        return 'CIRCLE'
                elif cv2.matchShapes(contour,STAR_CONTOUR_SHAPE,1,0.0) < 0.2:
                        return 'STAR'
                else:
                        return 'N/A'

        def find_doubles(self, figures_all):

                single_shapes = []
                double_shapes = []
                doubles_index = []

                for i in sorted(figures_all.keys()):
                        # Dont iterate previous shapes
                        # The last one 
                        if i in doubles_index:
                                continue
                        
                        for j in islice(sorted(figures_all.keys()), i+1, None):      
                                                            
                                res = cv2.pointPolygonTest(
                                        figures_all[i]['approxcontour'],
                                        figures_all[j]['center'],
                                        False
                                )
                                if res == 1:
                                        # j is in i
                                        doubles_index.append(j)
                                        double_shapes.append((i,j))
                                        break
                        else:
                                single_shapes.append(i)


                return (single_shapes, double_shapes)

        def filter_shapes(self, contours_filtered):

                figures_all = {}
	        #Pass through each contour and check if it has required properties to classify into required object
	        for i, x in enumerate(contours_filtered):

                        
		        #The below 2 functions help you to approximate the contour to a nearest polygon
                        arclength = cv2.arcLength(x, True)
                        approxcontour = cv2.approxPolyDP(x, 0.02 * arclength, True)

                        # Find valid shape
                        figure = self.find_figure(approxcontour)
                        if figure == 'N/A':
                                # Crap image
                                continue
		        info = {
                                'arclength' : arclength,
		                'approxcontour' : approxcontour,
                                'center' : self.find_center_contour(x),
                                # Should be renamed to shape
                                'figure' : figure,
                                'contour' : x,
                        }
                        figures_all[i] = info

                return figures_all


        def filter_allowed_figures_single(self, figures_all, single_shapes, color):

                allowed = []
                for i in single_shapes:

                        info = figures_all[i]
                        
                        for info_allowed in self.allowed_singles:
                                criteria = all([
                                        info_allowed['SHAPE']  == info['figure'],
                                        info_allowed['COLOR']  == color,
                                        ])
                                if criteria:
                                        info['color'] = color
                                        allowed.append(
                                                (info_allowed['ID'], info)
                                        )

                return allowed

        def filter_allowed_figures_double(self, figures_all, double_shapes, color):
                
                # Doubles
                allowed = []
                for (i,j) in double_shapes:

                        info_i = figures_all[i]
                        info_j = figures_all[j]

                        for info_allowed in self.allowed_doubles:
                                criteria = all([
                                        info_i['figure'] in info_allowed['SHAPE'],
                                        info_j['figure'] in info_allowed['SHAPE'],
                                        info_allowed['COLOR']  == color,
                                        ])
                                if criteria:
                                        info_i['color'] = color
                                        info_j['color'] = color
                                        allowed.append(
                                                (info_allowed['ID'], info_i, info_j)
                                        )
                return allowed
                
                
        def find_figures(self, mask, cv_image, area_threshold):

                '''
                Find contours(borders) for the shapes in the image
                
                '''

                contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

                # Filter out the smaller ones,  can do genertor
                #Discard contours with a small area as this may just be noise
                contours_filtered = [x for x in contours if cv2.contourArea(x) > area_threshold]

                figures_all = self.filter_shapes(contours_filtered)

                (single_shapes, double_shapes) = self.find_doubles(figures_all)

                return figures_all, single_shapes, double_shapes
                
        def draw_single_figures(self, single_shapes, cv_image):

                #Singles
                for (i, info) in single_shapes:
		        # cv2.drawContours(cv_image,contours[x],0,(0,255,255),2)
                        # Draw different colors for the shapes
		        cv2.drawContours(
                                cv_image,
                                [ info['approxcontour'] ],
                                0,
                                colors_contours[i] ,
                                2
                        )

                        
                        cv2.putText(
                                cv_image,
                                info['figure'] + '_' + info['color'] + '_' + str(i),
                                info['center'],
                                font,
                                0.8,
                                colors_contours[i],
                                2
                        )


        def draw_double_figures(self, double_shapes, cv_image):
                                
                for (i,info_1,info_2) in double_shapes:
                        # First 
                        cv2.drawContours(
                                cv_image,
                                [ info_1['approxcontour'] ],
                                0,
                                colors_contours[i] ,
                                2
                        )
                        # Second
                        cv2.drawContours(
                                cv_image,
                                [ info_2['approxcontour'] ],
                                0,
                                colors_contours[i] ,
                                2
                        )

                        cv2.putText(
                                cv_image,
                                info_1['figure'] + '_' + info_2['figure'] + '_' + info_1['color'] + '_' + str(i),
                                info_1['center'],
                                font,
                                0.8,
                                colors_contours[i],
                                2
                        )
                        
class Publisher:

        def __init__(self):

                #Create Rospy Publisher and subscriber
		self.object_location_pub = rospy.Publisher("/object_location", object_loc, queue_size =1)
                

        def publish(self, info):

                for f in info:
                        self.publish_coordinates(f[1]['contour'], f[0])
                
        def publish_coordinates(self, contour, number):

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
		obj_info_pub.ID = number #ID need to be changed
		# obj_info_pub.point.x = trans_pt.point.x
		# obj_info_pub.point.y = trans_pt.point.y
		# obj_info_pub.point.z = trans_pt.point.z

                # Temp
                obj_info_pub.point.x = P.point.x
		obj_info_pub.point.y = P.point.y
		obj_info_pub.point.z = P.point.z

		#publish the message
		self.object_location_pub.publish(obj_info_pub)

#This is the main class for object detection, it has some initializations about nodes
#Call back functions etc
class object_detection:
	def __init__(self):
		
		#original images is huge and creates lot of latency, therefore subscribe to compressed image
		self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)
		#Cv Bridge is used to convert images from ROS messages to numpy array for openCV and vice versa
		self.bridge = CvBridge()
		#Obejct to transform listener which will be used to transform the points from one coordinate system to other.

                self.cd = ContourDetection()

                self.publisher = Publisher()
	#Callback function for subscribed image        
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
                figures_all = {}
                for color, mask in masks.iteritems():
                        figures, single_shapes, double_shapes = self.cd.find_figures(mask, cv_image, 600)
                        singles = self.cd.filter_allowed_figures_single(figures, single_shapes, color)
                        doubles = self.cd.filter_allowed_figures_double(figures, double_shapes, color)
                        
                        # plotting
                        self.cd.draw_single_figures(singles, cv_image)
                        self.cd.draw_double_figures(doubles, cv_image)


                        # Send the coordinates of the figures found
                        self.publisher.publish(singles)
                        self.publisher.publish(doubles)

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
