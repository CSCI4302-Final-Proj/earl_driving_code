#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import os

import numpy as np
import cv2
import time

from racecar_flexbe_states.msg import Twist_float 

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
	self.pub = rospy.Publisher('/cmd_vel', Twist_float , queue_size=1)
	self.msg = Twist_float()
	

    def imageDepthCallback(self, data):
        try:
	    # we select bgr8 because its the OpenCv encoing by default
	    start_time = time.time()
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
	    minVal, maxVal, _, _= cv2.minMaxLoc(cv_image)
	    cv_image = cv2.convertScaleAbs(cv_image,alpha=(255/maxVal))
	    blurred = cv2.GaussianBlur(cv_image, (37,37),0)
	    ret, thresh = cv2.threshold(blurred, 245, 255, 0)
	
	    # find contour
	    cnts = cv2.findContours(thresh.copy(),cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]

	    #print(cnts)
	    
	    

	    for c in cnts:
		area = cv2.contourArea(c)
		
		#if not AREA/100 <area <AREA/20:
		#    continue
		
		#compute teh center of teh contour
		M = cv2.moments(c)
		cX = int(M['m10'] / M['m00'])
		cY = int(M['m01'] / M['m00'])

		print(area)
		# draw the contour and center of teh shape on teh image
		mask = np.zeros(thresh.shape, 'uint8')

		#cv2.drawContours(mask, [c], -1, 255, 2)
		x,y,w,h = cv2.boundingRect(c)
		mask = cv2.rectangle(mask,(x,y),(x+w,y+h),255,2)
		cv2.circle(mask, (cX,cY), 2, 255, -1)
			
	    
	    '''
	    mask = np.zeros(thresh.shape, 'uint8')
	    w = 60; h = 100
	    width, height =  thresh.shape[:2]
	    mid_x = 330 ; mid_y = 190
	    mask[0 : height, mid_x : 660] = 255
	    
	    #cv2.circle(mask, (mid_x,mid_y), 5 , (0, 255, 0), -1)
	    
	    res = cv2.bitwise_and(thresh , mask)
	   
	    M = cv2.moments(res)

	    cX = int(M['m10'] / M['m00'])
	    cY = int(M['m01'] / M['m00'])

	    cv2.circle(res, (cX,cY), 5 , (0, 255, 0), -1)
	    print(cX, cY ) 
	    #start_time = time.time()
	    # self.msg.vel = 0
	    # self.msg.angle = angle
	    '''
        except CvBridgeError as e:
            print(e)

	except ZeroDivisionError as e:
	    print(e)	
	
	#print(self.follow_corridor.curr_steering_angle)
	# self.pub.publish(self.msg)
	cv2.imshow("original image", blurred)
        cv2.waitKey(1)

	cv2.imshow("Image window", thresh)
	cv2.waitKey(1)
	cv2.imshow("mask", mask)
	cv2.waitKey(1)

	#print( time.time() - start_time)



    def calculate_heading(self):
	pass

    def pid(self):
	pass
   
    def ShapeDetector(self):
	pass

def main():
    rospy.init_node('image_listener', anonymous=True)
    topic = '/camera/depth/image_rect_raw'
    listener = ImageListener(topic)
    try:
        rospy.spin()
    except KeyboardInterrupt:
	print('Shutting down')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
