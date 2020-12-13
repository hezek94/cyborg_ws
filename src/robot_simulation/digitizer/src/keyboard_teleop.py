#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2

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
			cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
			resized_img = cv2.resize(cv_image,None,fx=0.5, fy=0.5, interpolation = 
cv2.INTER_CUBIC)
			cv2.imshow('image', resized_img)
			
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
			self.msg.linear.x = 0
			self.msg.angular.z = 0.9
		elif key == ord('d'):
			self.msg.linear.x = 0
			self.msg.angular.z = -0.9
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
