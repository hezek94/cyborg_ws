#!/usr/bin/env python

import apriltag
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from line_follower.cfg import LineParamConfig

class Digitizer:
    def __init__(self):
        rospy.init_node('keyboard_teleop', anonymous=True)
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.msg = Twist()		
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            # PART-1 
            # TRACK DETECTING AND Center Of Mass of the contours

            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            image = image[49:749, 135:666]
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            blurred = cv2.GaussianBlur(imgray, (5, 5), 0)
            ret, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

            _, contours, _= cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


            image1 = image.copy()

            track = []

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

                track.append(image.copy())
                # center of the shape on the image
                cv2.drawContours(track[-1], [c], -1, (0, 255, 0), -1)
                cv2.circle(track[-1], (cX, cY), 7, (255, 255, 255), -1)
                l.append([cX,cY])
                cv2.putText(track[-1], str(i), (cX - 20, cY - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)    #RED_COLOR
                # cv2.waitKey(10)
                i+=1

            # print(l)
            cv2.imshow("Image_1", image1)

            # l contains all the track coordinates

            l = [[379, 670], [373, 661], [0, 0], [270, 567], [289, 512], [410, 503], [136, 393], [273, 340], [420, 290], [246, 179], [129, 189], [262, 116]]

            
            for i in range(2,len(track)):
                cv2.imshow("track"+str(i), track[i])

            ### PART 2
            # Detecting Shapes & their COM Only!

            image2 = image.copy()
            im = image2
            # im = cv2.resize(im, (640, 480)) 
            hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

            lower = np.array([0,97,75])
            upper = np.array([255,255,255])
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(im, im, mask = mask)

            # cv2.imshow("Mask", mask)

            _, contours, _= cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # image2 = cv2.drawContours(image2, contours, -1, (0,255,0), 2)

            # cv2.imshow('image_2', image2)


            ### PART - 3
            # Arranging the shapes


            # def detect_shape(contour):
            #     shape = "unknown"
            #     peri = cv2.arcLength(contour, True)
            #     approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
            #     area = cv2.contourArea(contour)

            #     M = cv2.moments(contour)
            #     center = (0,0)
            #     if M["m00"] != 0:
            #         cX = int(M["m10"] / M["m00"])
            #         cY = int(M["m01"] / M["m00"])
            #         center = [cX, cY, area]

            #     side = len(approx)

            #     if side == 3:
            #         shape = "triangle"

            #     elif side == 4:
            #         (x, y, w, h) = cv2.boundingRect(approx)
            #         ratio = w / float(h)

            #         if (ratio >= 0.95 and ratio <= 1.05):
            #             shape = "square"
            #         else:
            #             shape = "rectangle"

            #     elif side>7:
            #         shape = "circle"

            #     return side, area, center,shape


            # im = image2
            
            # # HoughCircles to detect circles
            # img = result
            # img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            # img = cv2.medianBlur(img,5)
            # cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
            # circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,
            #                             param1=50,param2=30,minRadius=10,maxRadius=0)
            # circles = np.uint16(np.around(circles))
            # for i in circles[0,:]:
            #     # draw the outer circle
            #     cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            #     # draw the center of the circle
            #     cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
            # cv2.imshow("circles",cimg)

            # # Storing centers in dictionary with sides as key
            # d = {}
            # s4=0
            # for cnt in contours:
            #     s,a,c,shape = detect_shape(cnt)
            #     if s in d:
            #         if s==4:
            #             s4+=1
            #         if s4<2:
            #             d[s].append(c)
            #     else:
            #         d[s] = [c]

            # # Manually removing the circle from d using output of HoughCircles
            # if 7 in d:
            #     d.pop(7)

            # # Storing centers in order
            # # Manually arranging the area order
            # l2 = []
            # for i in sorted(d.keys()):
            #     if i == 5:
            #         l2.append([d[i][1][0],d[i][1][1]])
            #         l2.append([d[i][0][0],d[i][0][1]])
            #     else:
            #         for j in d[i]:
            #             l2.append([j[0],j[1]])

            # # print(l2)
            # #{10: [[134, 81, 2041.0]], 3: [[384, 588, 2204.5]], 4: [[159, 537, 6263.0], [376, 430, 2699.5]], 5: [[168, 269, 2160.0], [391, 149, 5609.0]], 7: [[294, 660, 3134.5], [278, 41, 2999.0]]}

            l2 = [[384, 588], [159, 537], [376, 430], [391, 149], [168, 269], [134, 81]]

            j = 0
            for i in l2:
                j+=1
                cv2.circle(im, (i[0], i[1]), 7, (255, 255, 255), -1)
                cv2.putText(im, str(j), (i[0] - 20, i[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)    #RED_COLOR
            cv2.imshow("arrangedShapes",im)

            # l2 contains the shape centers in order

           


            ### PART-4
            # BOT DETECTION, MOVEMENT
            image3 = image.copy()
            
            detector = apriltag.Detector()
            result = detector.detect(imgray)

            x0, y0 = result[0].center
            # Front
            x1, y1 = result[0].corners[1] #LEFT
            x2, y2 = result[0].corners[2] #RIGHT
            # Rear
            x3, y3 = result[0].corners[0] #LEFT
            x4, y4 = result[0].corners[3] #RIGHT


            # mid point:
            # x3 = (x1+x2)/2
            # y3 = (y1+y2)/2

            # x3 = int(x3)
            # y3 = int(y3)

            ### section formula

            # front-left
            xx1 = (2.525*x1-x2)/1.525
            yy1 = (2.525*y1-y2)/1.525

            # front-right
            xx2 = (2.525*x2-x1)/1.525
            yy2 = (2.525*y2-y1)/1.525

            # rear-left
            xx3 = (2.525*x3-x4)/1.525
            yy3 = (2.525*y3-y4)/1.525

            # rear-right
            xx4 = (2.525*x4-x3)/1.525
            yy4 = (2.525*y4-y3)/1.525

            # top-left
            X1 = (1.425*xx1-xx3)/0.425
            Y1 = (1.425*yy1-yy3)/0.425

            # top-right
            X2 = (1.425*xx2-xx4)/0.425
            Y2 = (1.425*yy2-yy4)/0.425

            # bottom-left
            X3 = (2.425*xx1-xx3)/1.425
            Y3 = (2.425*yy1-yy3)/1.425

            # bottom-right
            X4 = (2.425*xx2-xx4)/1.425
            Y4 = (2.425*yy2-yy4)/1.425

            cv2.circle(image3, (int(x0), int(y0)), 7, (255, 255, 255), -1)
            cv2.circle(image3, (int(x1), int(y1)), 7, (255, 255, 255), -1)
            cv2.circle(image3, (int(x2), int(y2)), 7, (255, 255, 255), -1)
            cv2.circle(image3, (int(x3), int(y3)), 7, (255, 255, 255), -1)
            cv2.circle(image3, (int(x4), int(y4)), 7, (255, 255, 255), -1)            
            cv2.circle(image3, (int(xx1), int(yy1)), 7, (255, 0, 0), -1)
            cv2.circle(image3, (int(xx2), int(yy2)), 7, (255, 0, 0), -1)
            cv2.circle(image3, (int(xx3), int(yy3)), 7, (255, 0, 0), -1)
            cv2.circle(image3, (int(xx4), int(yy4)), 7, (255, 0, 0), -1)
            cv2.circle(image3, (int(X1), int(Y1)), 7, (0, 255, 0), -1)
            cv2.circle(image3, (int(X2), int(Y2)), 7, (0, 255, 0), -1)
            cv2.circle(image3, (int(X3), int(Y3)), 7, (0, 255, 0), -1)
            cv2.circle(image3, (int(X4), int(Y4)), 7, (0, 255, 0), -1)
            # cv2.circle(image3, (int(x3), int(y3)), 7, (255, 0, 0), -1)

            # print(x3,y3)
            # print(x,y)
             
             

            cv2.imshow("Image_4", image3)


            ### PART - 5 
            # Perspective transformation

            image5 = image.copy()

            tl = (X1,Y1)
            bl = (X3,Y3)
            tr = (X2,Y2)
            br = (X4,Y4)

            pts1 = np.float32([tl, bl, tr, br]) 
            pts2 = np.float32([[0, 0], [0, 600], [500, 0], [500, 600]]) 

            matrix = cv2.getPerspectiveTransform(pts1, pts2) 
            result_perspectiveChange = cv2.warpPerspective(image5, matrix, (500,600))
            
            original = result_perspectiveChange.copy()

            ###PART - 6
            # Line Follower

            # center
            x = 246
            y = 275

            # Marking Center Of The Screen
            cv2.circle(result_perspectiveChange, (x,y), 7, (255,255,255), -1)
            cv2.putText(result_perspectiveChange, 'ORIGIN', (x-20, y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2) # GREEN COLOR

            # Path detection using contours after thresholding images
            imgray = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(imgray, (5,5), 0)
            ret, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

            _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # cv2.imshow("thresh", thresh)



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
                cv2.drawContours(result_perspectiveChange, [c], -1, (0,255,0), 2)
                cv2.circle(result_perspectiveChange, (cX,cY), 7, (255,255,255), -1)
                sensed_output = cX
                cv2.putText(result_perspectiveChange, str(i), (cX-20, cY-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2) # RED

            cv2.imshow('result_perspectiveChange',result_perspectiveChange) 
            

        except CvBridgeError as e:
            print (e)

        ### Part - 7
        # Bot direction

        













        set_point = x # CENTER X COORDINATE

        error = set_point-sensed_output

        if error>0:
            self.msg.linear.x = 0.1
            self.msg.angular.z = 0.2
        elif error<0:
            self.msg.linear.x = 0.1
            self.msg.angular.z = -0.2
        else:
            self.msg.linear.x = 0
            self.msg.angular.z = 0

        # Publish the data
        self.pub.publish(self.msg)
        self.rate.sleep()

        key = cv2.waitKey(1)

        if key == 27:
            rospy.signal_shutdown("shutdown")
            cv2.destroyAllWindows()

		
		
		# self.pub.publish(self.msg)

		# self.rate.sleep()






        self.pub.publish(self.msg)
        self.rate.sleep()

        

        




if __name__ == '__main__':
    dj = Digitizer()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
