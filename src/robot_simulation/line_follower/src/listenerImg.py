#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class Listener:
	def __init__(self):
	    rospy.init_node('listener', anonymous=True)

	    self.sub = rospy.Subscriber('chatter', Image, self.callback)
	    
	    self.bridge = CvBridge()
	    #rospy.spin()
	
	def callback(self, data):
	    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
	    try:
		cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		self.perform(cv_image)
	    except CvBridgeError as e:
		print (e)

	def perform(self, image):
		def nothing(x):
			pass

		cv2.namedWindow("Trackbars")

		cv2.createTrackbar("L - H", "Trackbars", 50, 255, nothing)
		cv2.createTrackbar("L - S", "Trackbars", 34, 255, nothing)
		cv2.createTrackbar("L - V", "Trackbars", 65, 255, nothing)
		cv2.createTrackbar("U - H", "Trackbars", 255, 255, nothing)
		cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
		cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		l_h = cv2.getTrackbarPos("L - H", "Trackbars")
		l_s = cv2.getTrackbarPos("L - S", "Trackbars")
		l_v = cv2.getTrackbarPos("L - V", "Trackbars")
		u_h = cv2.getTrackbarPos("U - H", "Trackbars")
		u_s = cv2.getTrackbarPos("U - S", "Trackbars")
		u_v = cv2.getTrackbarPos("U - V", "Trackbars")
    
		lower = np.array([l_h,l_s,l_v])
		upper = np.array([u_h,u_s,u_v])
		mask = cv2.inRange(hsv, lower, upper)
		result = cv2.bitwise_and(image, image, mask = mask)

		cv2.imshow("Mask", mask)
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		cv2.imshow('image', gray)
		cv2.waitKey(10)

if __name__ == '__main__':
    listener = Listener()
    try:
	rospy.spin()
    except rospy.ROSInterruptException as e:
	print(e)
