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



    def callback(self, data):
        x = 410
        y = 438

        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            

            # Marking Center Of The Screen
            cv2.circle(image, (x,y), 7, (255,255,255), -1)
            cv2.putText(image, 'ORIGIN', (x-20, y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2) # GREEN COLOR
            cv2.imshow('image', image)





            # Path detection using contours after thresholding images
                # Official docs for explaination of contours in openCV2 in descriuption!
            imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(imgray, (5,5), 0)
            ret, thresh = cv2.threshold(blurred, 45, 255, cv2.THRESH_BINARY_INV)

            _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            cv2.imshow("thresh", thresh)



            # Error Detection
            sensed_output = x # Default

            i = 0
            for c in contours:
                i+=1

                # Compute the center of mass of the contours
                M = cv2.moments(c)
                if M['m00']!=0:
                    cX = int(M['m10']/M['m00'])
                    cY = int(M['m01']/M['m00'])

                # center of the shape of the image
                cv2.drawContours(image, [c], -1, (0,255,0), 2)
                cv2.circle(image, (cX,cY), 7, (255,255,255), -1)
                sensed_output = cX
                cv2.putText(image, str(i), (cX-20, cY-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2) # RED

            cv2.imshow("image", image)

            



            
                
        except CvBridgeError as e:
            print(e)

        # Line following conditions

        set_point = x # CENTER X COORDINATE

        error = set_point-sensed_output

        if error>0:
            self.msg.linear.x = 0.2
            self.msg.angular.z = 0.3
        elif error<0:
            self.msg.linear.x = 0.2
            self.msg.angular.z = -0.3
        else:
            self.msg.linear.x = 0.05
            self.msg.angular.z = 0

        # Publish the data
        self.pub.publish(self.msg)
        self.rate.sleep()


        key = cv2.waitKey(1)

        if key == 27: #ESC KEY
            rospy.signal_shutdown("shutdown")
            cv2.destroyAllWindows()


if __name__ == '__main__':
    kt = KeyboardTeleop()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
