#!/usr/bin/env python

import cv2
import rospy
import numpy as np
import apriltag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math

class Movement:
    def __init__(self):
        rospy.init_node("Movement")
        rospy.sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            track = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            track_gray = cv2.cvtColor(track, cv2.COLOR_BGR2GRAY)
            blank = np.zeros(track_gray.shape, np.uint8)

            detector = apriltag.Detector()
            result = detector.detect(track_gray)

            x1, y1 = result[0].corners[0]
            x2, y2 = result[0].corners[1]
            x0, y0 = result[0].center

            a = 50
            w = 50
            h = 100
            m1 = math.atan2(y2-y1, x2-x1)
            m2 = m1 + np.pi/2
            pt1 = (int(x0 + a*math.cos(m1) + h*math.cos(m2)/2), int(y0 + a*math.sin(m1) + h*math.sin(m2)/2))
            pt2 = (int(x0 + (a+w)*math.cos(m1) + h*math.cos(m2)/2), int(y0 + (a+w)*math.sin(m1) + h*math.sin(m2)/2))
            pt3 = (int(x0 + (a+w)*math.cos(m1) - h*math.cos(m2)/2), int(y0 + (a+w)*math.sin(m1) - h*math.sin(m2)/2))
            pt4 = (int(x0 + a*math.cos(m1) - h*math.cos(m2)/2), int(y0 +  a*math.sin(m1) - h*math.sin(m2)/2))
            roi = np.array([pt1, pt2, pt3, pt4])
            roi.reshape((-1,1,2))

            track = cv2.polylines(track, [roi], isClosed=True, color=(100,100,100), thickness=2)
            blank = cv2.polylines(blank, [roi], isClosed=True, color=255, thickness=3)


            cv2.imshow("Blank", blank)
            cv2.imshow("Track", track)

        except CvBridgeError as e:
            print(e)
            rospy.signal_shutdown("shutdown")

        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            rospy.signal_shutdown("shutdown")

if __name__ == '__main__':
    find = Movement()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospt.ROSInterruptException as e:
        print(e)
