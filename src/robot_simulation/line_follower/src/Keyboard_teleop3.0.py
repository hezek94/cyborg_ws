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

	    self.cx = 0
	    self.cy = 0
	    #rospy.spin()


	def callback(self, data):
		x = 410
		y = 438
	    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
		try:
			image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

			def nothing(x):
				pass

			cv2.namedWindow("Trackbars")

			cv2.createTrackbar("kp", "Trackbars", 0, 1000, nothing)
			cv2.createTrackbar("ki", "Trackbars", 0, 1000, nothing)
			cv2.createTrackbar("kd", "Trackbars", 0, 1000, nothing)
			cv2.createTrackbar("sp", "Trackbars", 0, 1000, nothing)
			# cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
			# cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

			# hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

			kp = cv2.getTrackbarPos("kp", "Trackbars")
			ki = cv2.getTrackbarPos("ki", "Trackbars")
			kd = cv2.getTrackbarPos("kd", "Trackbars")
			sp = cv2.getTrackbarPos("sp", "Trackbars")
			# u_s = cv2.getTrackbarPos("U - S", "Trackbars")
			# u_v = cv2.getTrackbarPos("U - V", "Trackbars")
		
			# lower = np.array([l_h,l_s,l_v])
			# upper = np.array([u_h,u_s,u_v])
			# thresh = cv2.inRange(hsv, lower, upper)

			cv2.circle(image, (x, y), 7, (255, 255, 255), -1)
			cv2.putText(image, 'ORIGIN', (x - 20, y - 20),
					cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)    #GREEN_COLOR


			imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			blurred = cv2.GaussianBlur(imgray, (5, 5), 0)
			ret, thresh = cv2.threshold(blurred, 45, 255, cv2.THRESH_BINARY_INV)

			_, contours, _= cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


			# cv2.imshow("Mask", cv2.drawContours(image, contours, -1, (0,255,0), 2))
			cv2.imshow('thresh', thresh)
			# cv2.imshow('gray', imgray)

			sensed_output = x
			l = []
			i = '1'
			# loop over the contours
			for c in contours:
				i+='1'
			# compute the center of the contour
				M = cv2.moments(c)
				if M["m00"]!=0:
					cX = int(M["m10"] / M["m00"])
					self.cx = cX
				else:
					cX = self.cx
				if M["m00"]!=0:
					cY = int(M["m01"] / M["m00"])
					self.cy = cY
				else:
					cY = self.cy
				
			# center of the shape on the image
				cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
				cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
				sensed_output = cX
				l.append([cX,cY])
				cv2.putText(image, i, (cX - 20, cY - 20),
					cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)    #RED_COLOR
			# cv2.waitKey(10)
			
			
			cv2.imshow("Image", image)


			

			
		except CvBridgeError as e:
			print (e)
		
		setpoint = x
		
		def pid(self, error):
		    pass

		error = setpoint - sensed_output
		if error>0:
				self.msg.linear.x = 0.2
				self.msg.angular.z = 0.3
		elif error<0:
			self.msg.linear.x = 0.2
			self.msg.angular.z = -0.3
		else:
			self.msg.linear.x = 0.05
			self.msg.angular.z = 0
		
		self.pub.publish(self.msg)

		self.rate.sleep()

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
