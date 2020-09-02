#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

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
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		cv2.imshow('image', gray)
		cv2.waitKey(10)

if __name__ == '__main__':
    listener = Listener()
    try:
	rospy.spin()
    except rospy.ROSInterruptException as e:
	print(e)
