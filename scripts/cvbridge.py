#!/usr/bin/env python
import roslib
roslib.load_manifest('led_localization')
import sys
import rospy
import cv2
import numpy as np
import cv2.cv as cv

from math import *

from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

def find_bright(cimg):
  chans= cv2.split(cimg)

  gray = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)

  blue_maxed = cv2.equalizeHist(chans[0])
  red_maxed = cv2.equalizeHist(chans[2])

  #cv2.imshow('blue', blue_maxed)
  #cv2.imshow('red', red_maxed)

  blue_blur = cv2.GaussianBlur(blue_maxed, (9,9), 0)
  #cv2.imshow('blue blur', blue_blur)

  red_blur = cv2.GaussianBlur(red_maxed, (9,9), 0)
  cv2.imshow('red blur', red_blur)

  #gray = orig
  (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(red_blur)
  cv2.circle(cimg, maxLoc, 15, (255,0,0), 2)

  cv2.imshow("brightest", cimg)
  c = cv2.waitKey(1)

  return cimg

def track_color(cimg):
  ''' captures frame from webcame, creates thresholded and gaussian blur images '''
  gaussian_images = []
  chans= cv2.split(cimg)
  red_maxed = cv2.equalizeHist(chans[2])

  cv2.imshow('red_maxed', red_maxed)
  img_HSV = cv2.cvtColor(cimg, cv.CV_BGR2HSV)

  red_Threshed = cv2.inRange(img_HSV, np.array((0,0, 0)), np.array((12,255,255)))
  red_gaussian = cv2.GaussianBlur(red_Threshed, (9,9), 2, 2)

  # blue_Threshed = cv2.inRange(img_HSV, np.array((100,0,0)), np.array((120,255,255)))
  # blue_gaussian = cv2.GaussianBlur(blue_Threshed, (9,9), 2, 2)

  # green_Threshed = cv2.inRange(img_HSV, np.array((66,50,50)), np.array((94,255,255)))
  # green_gaussian = cv2.GaussianBlur(green_Threshed, (9,9), 2, 2)

  # total_threshed = red_Threshed + blue_Threshed + green_Threshed
  # total_gaussian = red_gaussian + blue_gaussian + green_gaussian

  cv2.imshow("red", red_gaussian)
 # cv2.imshow("threshed", total_threshed)

#  cv2.imshow("threshed", total_threshed)
  c = cv2.waitKey(1)

  gaussian_images.append(red_gaussian)
  # gaussian_images.append(blue_gaussian)
  # gaussian_images.append(green_gaussian)

  return gaussian_images


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("masked_leds",Image)
    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera/rgb/image_color",Image,self.callback)

  def callback(self,data):
    #Converts ROS image to IplImage
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    (rows,cols,channels) = cv_image.shape

    if cols > 60 and rows > 60 :
      filtered_images = track_color(cv_image)
      #images = find_bright(cv_image)
#      res = find_bright(filtered_images[0])
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    #Converts IplImage to ROS
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError, e:
      print e

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)