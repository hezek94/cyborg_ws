#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class Drone:
	def __init__(self):
		rospy.init_node('listener', anonymous=True)

		self.sub = rospy.Subscriber('/front_cam/camera/image', Image, self.callback)

		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		self.rate = rospy.Rate(10)

		self.msg = Twist()

		self.bridge = CvBridge()
		#rospy.spin()
		
	def callback(self, data):
	#rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError as e:
			print (e)

		key = cv2.waitKey(1)

		if key== 32:
			if self.msg.linear.z == 0:
				self.msg.linear.z = 0.5
			else:
				self.msg.linear.z = 0

		if key== ord('x'):
			self.msg.linear.z = -0.5

		if key == 27: #ESC KEY
			rospy.signal_shutdown("shutdown")
			cv2.destroyAllWindows()

		if key== ord('w'):
			if self.msg.linear.x == 0:
				self.msg.linear.x = 1
			else:
				self.msg.linear.x = 0

		if key== ord('s'):
			if self.msg.linear.x == 0:
				self.msg.linear.x = -1
			else:
				self.msg.linear.x = 0

		if key== ord('a'):
			if self.msg.linear.y == 0:
				self.msg.linear.y = 1
			else:
				self.msg.linear.y = 0

		if key== ord('d'):
			if self.msg.linear.y == 0:
				self.msg.linear.y = -1
			else:
				self.msg.linear.y = 0

		self.pub.publish(self.msg)
		self.rate.sleep()

			

if __name__ == '__main__':
    myDrone = Drone()
    try:
		rospy.spin()
    except rospy.ROSInterruptException as e:
		print(e)
