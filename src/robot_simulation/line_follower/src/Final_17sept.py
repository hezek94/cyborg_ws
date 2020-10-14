#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from dynamic_reconfigure.server import Server
from line_follower.cfg import LineParamConfig

class KeyboardTeleop:
	def __init__(self):
		rospy.init_node('keyboard_teleop', anonymous=True)
	

		self.sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
		
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		self.rate = rospy.Rate(10)

		self.msg = Twist()		

		self.bridge = CvBridge()

		srv = Server(LineParamConfig, self.reconfig)
	
		self.param = {'KP':0.0, 'KI':0.0, 'KD':0.03, 'SP':0}

		self.cx = 0
		self.cy = 0
		self.intg = self.lastError = 0
	    #rospy.spin()
	
	def reconfig(self, config, level):
		self.param = config
		return config

	def pid(self, error):
		prop = error
		self.intg += error
		diff = error - self.lastError
		self.lastError = error
		balance = self.param['KP']*prop+self.param['KI']*0.01*self.intg+self.param['KD']*diff*0.01
		return balance

	def callback(self, data):
		x = 300
		y = 10

	    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
		try:
			image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

			image = image[500:900, 200:800] #[y1:y2, x1:x2]

			
			cv2.circle(image, (x, y), 7, (255, 255, 255), -1)
			cv2.putText(image, 'ORIGIN', (x - 20, y - 20),
					cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)    #GREEN_COLOR


			imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			blurred = cv2.GaussianBlur(imgray, (5, 5), 0)
			ret, thresh = cv2.threshold(blurred, 45, 255, cv2.THRESH_BINARY_INV)

			_, contours, _= cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


			# cv2.imshow("Mask", cv2.drawContours(image, contours, -1, (0,255,0), 2))
			cv2.imshow('thresh', thresh)

			sensed_output = x
			l = []
			i = '1'
			# loop over the contours
			for c in contours:
				
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
				i+='1'
				break
			
			
			cv2.imshow("Image", image)


			

			
		except CvBridgeError as e:
			print (e)
		
		setpoint = x
		error = setpoint - sensed_output
		error = error/20

		balance = self.pid(error)
		self.msg.linear.x = self.param['SP']
		if cY>300:
			self.msg.linear.x = 0.02
			cv2.waitKey(2000)
		self.msg.angular.z = balance

		self.pub.publish(self.msg)
		self.rate.sleep()

		

		

		key = cv2.waitKey(1)
		
		if key == 27:
			rospy.signal_shutdown("shutdown")
			cv2.destroyAllWindows()


		



if __name__ == '__main__':
	kt = KeyboardTeleop()
    	try:
		if not rospy.is_shutdown():
			rospy.spin()
    	except rospy.ROSInterruptException as e:
		print(e)
