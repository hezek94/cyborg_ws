#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class KeyboardTeleop:
	def __init__(self):
	    rospy.init_node('keyboard_teleop', anonymous=True)

	    self.sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
	    
	    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	    self.rate = rospy.Rate(10)

	    self.msg = Twist()		

	    self.bridge = CvBridge()
	    #rospy.spin()
	    self.x = 0.0
	    self.z = 0.0

	def callback(self, data):
	    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
		try:
			image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
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
			# cv2.waitKey(10)
		except CvBridgeError as e:
			print (e)
		
		key = cv2.waitKey(1)
		
		if key == ord('w'):
			if self.msg.linear.x == -0.2:
				self.msg.linear.x = self.x = 0.0 
			else:
				self.msg.linear.x = self.x = 0.2 
				self.msg.angular.z = 0.0

		elif key == ord('s'):
			if self.msg.linear.x == 0.2:
				self.msg.linear.x = self.x = 0.0
			else:
				self.msg.linear.x = self.x = -0.2 
				self.msg.angular.z = 0.0

		elif key == ord('a'):
			self.msg.linear.x = self.x
			self.msg.angular.z = 0.3
		elif key == ord('d'):
			self.msg.linear.x = self.x
			self.msg.angular.z = -0.3
		else:
			self.msg.linear.x = 0.0
			self.msg.angular.z = 0.0

		if key == 27:
			rospy.signal_shutdown("shutdown")
			cv2.destroyAllWindows()


		
		
		self.pub.publish(self.msg)

		self.rate.sleep()


if __name__ == '__main__':
    kt = KeyboardTeleop()
    try:
	if not rospy.is_shutdown():
		rospy.spin()
    except rospy.ROSInterruptException as e:
	print(e)
