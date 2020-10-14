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
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
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
            # cv2.imshow("gray", img)

            imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            blurred = cv2.GaussianBlur(imgray, (5, 5), 0)
            ret, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

            _, contours, _= cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


            image1 = image.copy()
            image1 = cv2.drawContours(image1, contours, -1, (0,255,0), 2)



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
                cv2.drawContours(image1, [c], -1, (0, 255, 0), 2)
                cv2.circle(image1, (cX, cY), 7, (255, 255, 255), -1)
                l.append([cX,cY])
                cv2.putText(image1, str(i), (cX - 20, cY - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)    #RED_COLOR
                # cv2.waitKey(10)
                i+=1


            cv2.imshow("Image1", image1)







            cv2.imshow('thresh', thresh)


            # PART 2
            # Detecting Shapes Only!




            def nothing(x):
                pass

            cv2.namedWindow("Trackbars")

            cv2.createTrackbar("L - H", "Trackbars", 0, 255, nothing)
            cv2.createTrackbar("L - S", "Trackbars", 97, 255, nothing)
            cv2.createTrackbar("L - V", "Trackbars", 75, 255, nothing)
            cv2.createTrackbar("U - H", "Trackbars", 255, 255, nothing)
            cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
            cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            l_h = cv2.getTrackbarPos("L - H", "Trackbars")
            l_s = cv2.getTrackbarPos("L - S", "Trackbars")
            l_v = cv2.getTrackbarPos("L - V", "Trackbars")
            u_h = cv2.getTrackbarPos("U - H", "Trackbars")
            u_s = cv2.getTrackbarPos("U - S", "Trackbars")
            u_v = cv2.getTrackbarPos("U - V", "Trackbars")

            lower = np.array([l_h,l_s,l_v])
            upper = np.array([u_h,u_s,u_v])
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(image, image, mask = mask)

            cv2.imshow("Mask", mask)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            _, contours, _= cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            image2 = image.copy()
            image2 = cv2.drawContours(image2, contours, -1, (0,255,0), 2)
            cv2.imshow('image_2', image2)


            # PART 3
            # Arranging the shapes

            def detect_shape(contour):
                shape = "unknown"
                peri = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                area = cv2.contourArea(contour)

                M = cv2.moments(contour)
                center = (0,0)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    center = [cX, cY, area]

                side = len(approx)

                if side == 3:
                    shape = "triangle"

                elif side == 4:
                    (x, y, w, h) = cv2.boundingRect(approx)
                    ratio = w / float(h)

                    if (ratio >= 0.95 and ratio <= 1.05):
                        shape = "square"
                    else:
                        shape = "rectangle"

                elif side>7:
                    shape = "circle"

                return side, area, center,shape


            d = {}
            for cnt in contours:
                s,a,c,shape = detect_shape(cnt)
                if shape == 'circle':
                    continue
                if s in d:
                    d[s].append(c)
                else:
                    d[s] = [c]


            im = image2


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
                cv2.drawContours(image2, [c], -1, (0, 255, 0), 2)
                cv2.circle(image2, (cX, cY), 7, (255, 255, 255), -1)
                l.append([cX,cY])
                cv2.putText(image2, str(i), (cX - 20, cY - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)    #RED_COLOR
                # cv2.waitKey(10)
                i+=1


            # j = 0
            # dd = {}
            # for i in sorted (d.keys()):
            #     dd={}
            #     for ii in d[i]:
            #         dd[ii[2]] = [ii[0],ii[1]]
            #     for ii in sorted (dd.keys()):
            #         j+=1
            #     if j>10:
            #         break
            #     cv2.circle(im, (dd[ii][0], dd[ii][1]), 7, (255, 255, 255), -1)
            #     cv2.putText(im, str(j), (dd[ii][0] - 20, dd[ii][1] - 20),
            #         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)    #RED_COLOR
            
            cv2.imshow("IMMM", im)

            






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
