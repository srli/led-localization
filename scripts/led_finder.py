#!/usr/bin/env python
"""
LED Finder script for SCOPE Boeing

Takes camera video feed as input, outputs pixel locations of LEDs as ROS messages
"""

import roslib; roslib.load_manifest('led_localization')
import rospy
import roslib

from std_msgs.msg import String
from led_localization.msg import *

import cv2
import numpy as np
import cv2.cv as cv

from math import *

def track_color():
	''' captures frame from webcame, creates thresholded and gaussian blur images '''
	ret, frame = cap.read()
	cimg = frame

	gaussian_images = []

	img_HSV = cv2.cvtColor(cimg, cv.CV_BGR2HSV)

	red_Threshed = cv2.inRange(img_HSV, np.array((2,50, 50)), np.array((8,170,200)))
	red_gaussian = cv2.GaussianBlur(red_Threshed, (9,9), 2, 2)

	blue_Threshed = cv2.inRange(img_HSV, np.array((100,0,0)), np.array((120,255,255)))
	blue_gaussian = cv2.GaussianBlur(blue_Threshed, (9,9), 2, 2)

	green_Threshed = cv2.inRange(img_HSV, np.array((66,50,50)), np.array((94,255,255)))
	green_gaussian = cv2.GaussianBlur(green_Threshed, (9,9), 2, 2)

	total_threshed = red_Threshed + blue_Threshed + green_Threshed
	total_gaussian = red_gaussian + blue_gaussian + green_gaussian

	cv2.imshow("threshed", total_threshed)
	c = cv2.waitKey(1)

	gaussian_images.append(red_gaussian)
	gaussian_images.append(blue_gaussian)
	gaussian_images.append(green_gaussian)

	return gaussian_images

def find_location(image):
	"""Takes HSV image and finds the pixel location of the LED in the FOV."""
	#Use hough circles and find the center of the most likely circle.
	pass


def pub_led_location(led_array):
	"""Publishes all the pixel locations of the LED from an array"""
	r = rospy.Rate(100)
	pub = rospy.Publisher('beacon_locals', beacons, queue_size=10)
	msg = beacons()

	msg.red_beacon = 1
	msg.blue_beacon = 2
	msg.green_beacon = 3

	pub.publish(msg)
	r.sleep()


if __name__ == "__main__":
	#rospy.init_node('neato_controller', anonymous=True)
	cap = cv2.VideoCapture(1)
	rospy.init_node('led_finder')

	while not rospy.is_shutdown():

		# get images for each finger
		gaussian_images = 	track_color()

		red_filtered	= 	gaussian_images[0] #red
		blue_filtered	= 	gaussian_images[1] #blue
		green_filtered	= 	gaussian_images[2] #green

		# determines state of each finger for this
		red_location = find_location(red_filtered)
		blue_location = find_location(blue_filtered)
		green_location = find_location(green_filtered)

		led_array = [red_location, blue_location, green_location]

		# determine what the motor command translates as
		pub_led_location(led_array)
