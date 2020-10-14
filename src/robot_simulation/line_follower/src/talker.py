#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridge, CvBridgeError
import cv2


class Talker:
	def __init__(self):
		self.pub = rospy.Publisher('chatter', Image, queue_size=10)
		rospy.init_node('talker', anonymous=True)
		self.rate = rospy.Rate(20) # 10hz
		self.msg = Image()
		self.bridge = CvBridge()
		self.read()

	def read(self):
		cap = cv2.VideoCapture('/home/ashuditya/Videos/vid.mp4')
		while not rospy.is_shutdown():
			success, image = cap.read()
			img = image[348:600, 78:500]
			if success:
				cv2.imshow('Image', image)
				cv2.imshow('CROPPEDImg', img)
				if cv2.waitKey(1) == 27:
					break
				self.msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
			self.publish()
		cap.release()
		cv2.destroyAllWindows()
	def publish(self):
		#rospy.loginfo(self.msg)
		self.pub.publish(self.msg)
		self.rate.sleep()

if __name__ == '__main__':
	try:
		talker = Talker()
	except rospy.ROSInterruptException as e:
		print(e)
	
