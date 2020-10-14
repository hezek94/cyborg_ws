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
		image = cv2.imread('/home/ashuditya/cyborg_ws/src/robot_simulation/digitizer/src/img.png')
		while not rospy.is_shutdown():
			success = True
			if success:
				imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
				blurred = cv2.GaussianBlur(imgray, (5, 5), 0)
				ret, thresh = cv2.threshold(blurred, 20, 255, cv2.THRESH_BINARY)
				
				_, contours, _= cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

				image = cv2.drawContours(image, contours, -1, (0,255,0), 2)
				cv2.imshow('Image', image)
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
	
