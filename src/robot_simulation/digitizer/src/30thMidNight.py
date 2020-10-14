#!/usr/bin/env python

import apriltag
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from line_follower.cfg import LineParamConfig

class KeyboardTeleop:
    def __init__(self):
        rospy.init_node('keyboard_teleop', anonymous=True)


        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(10)

        self.msg = Twist()		

        self.bridge = CvBridge()

        self.param = {'KP':0.0, 'KI':0.0, 'KD':0.03, 'SP':0}




    def callback(self, data):

        #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            #YO!
            #asdhj
            #123
            image = image[49:749, 135:666]
            img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # img = img[600:750, 390:550]
            detector = apriltag.Detector()
            result = detector.detect(img)
            # center = result[0].center
            # corners = result[0].corners
            # print(result)
            cv2.imshow("gray", img)

            imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            blurred = cv2.GaussianBlur(imgray, (5, 5), 0)
            ret, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

            _, contours, _= cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)



            image = cv2.drawContours(image, contours, -1, (0,255,0), 2)



            l = []
            i = 1
            for c in contours:
                cX = cY = 0
                # compute the center of the contour
                M = cv2.moments(c)
                if M["m00"]!=0:
                    cX = int(M["m10"] / M["m00"])
                if M["m00"]!=0:
                    cY = int(M["m01"] / M["m00"])		

                # center of the shape on the image
                cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
                l.append([cX,cY])
                cv2.putText(image, str(i), (cX - 20, cY - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)    #RED_COLOR
                # cv2.waitKey(10)
                i+=1


            cv2.imshow("Image", image)







            cv2.imshow('thresh', thresh)
            #indention error fixed!







        except CvBridgeError as e:
            print (e)





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
