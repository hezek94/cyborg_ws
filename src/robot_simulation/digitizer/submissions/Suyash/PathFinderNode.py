#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class PathFinder:
    def __init__(self):
        rospy.init_node("PathFinder")
        self.sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            track = self.bridge.imgmsg_to_cv2(data, 'bgr8')[55:745, 141:659]
        except CvBridgeError as e:
            print(e)
            rospy.signal_shutdown("shutdown")

        ## Created some images that will be needed further.
        track_gray = cv2.cvtColor(track, cv2.COLOR_BGR2GRAY)
        blank = np.zeros(track.shape[:2], np.uint8)
        final_path_1 = blank.copy()
        final_path_2 = blank.copy()

        ## Created a separate binary "paths" image with only paths.
        paths = cv2.inRange(track_gray, 200, 255)
        paths = cv2.erode(paths, np.ones((3,3), np.uint8), iterations=3)
        paths = cv2.dilate(paths, np.ones((3,3), np.uint8), iterations=2)

        ## Created a separate binary "shapes" image with only shapes (checkpoints).
        shapes = cv2.inRange(track_gray, (50), (200))
        shapes = cv2.erode(shapes, np.ones((3,3), np.uint8), iterations=2)
        shapes = cv2.dilate(shapes, np.ones((3,3), np.uint8), iterations=2)

        img, contours, hierarchy = cv2.findContours(shapes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        checkpt = []
        shapes = blank.copy()                                                                       ### VISUALIZATION

        for cnt in contours:
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            if ((3.14*radius*radius) / cv2.contourArea(cnt)) > 1.2:
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02*peri, True)
                if len(approx) < 11:
                    checkpt.append(approx)
                else:
                    start_pt = center
            else:
                end_pt = center

        cv2.circle(track, tuple(start_pt), 10, (0,255,0), 3)                                        ### VISUALIZATION
        cv2.circle(track, tuple(end_pt), 10, (0,255,255), 3)                                        ### VISUALIZATION

        for cnt in checkpt:                                                                         ###
            cv2.polylines(shapes, [cnt], True, 255, 3)                                              ### VISUALIZATION
            cv2.fillPoly(shapes, [cnt], 255)                                                        ###

        checkpt = sorted(checkpt, key=lambda x:cv2.contourArea(x))
        route_1 = sorted(checkpt, key=lambda x:len(x))
        route_1_coords = []
        route_2 = sorted(checkpt, key=lambda x:len(x), reverse=True)
        route_2_coords = []

        count = 0                                                                                   ### VISUALIZATION
        for cnt in route_1:
            count += 1                                                                              ### VISUALIZATION
            cv2.drawContours(track,[cnt],-1,(0,255,0),4)
            moment=cv2.moments(cnt)
            cx = int(moment['m10'] / moment['m00'])
            cy = int(moment['m01'] / moment['m00'])
            route_1_coords.append([cx,cy])
            cv2.putText(track, str(count),(cx,cy),cv2.FONT_HERSHEY_SIMPLEX,0.5,100,2)               ### VISUALIZATION

        count = 0                                                                                   ### VISUALIZATION
        for cnt in route_2:
            count += 1                                                                              ### VISUALIZATION
            cv2.drawContours(track,[cnt],-1,(0,255,0),4)                                            ### VISUALIZATION
            moment=cv2.moments(cnt)
            cx = int(moment['m10'] / moment['m00'])
            cy = int(moment['m01'] / moment['m00'])
            route_2_coords.append([cx,cy])
            cv2.putText(track, str(count),(cx-10,cy+10),cv2.FONT_HERSHEY_SIMPLEX,0.5,50,2)          ### VISUALIZATION

        temp = []
        route_1_midpts = []
        route_2_midpts = []

        index = 0
        for coords in route_1_coords:
            if index != 0:
                route_1_midpts.append([int(coords[0]/2 + temp[0]/2), int(coords[1]/2 + temp[1]/2)])
            temp = coords
            index += 1

        index = 0
        for coords in route_2_coords:
            if index != 0:
                route_2_midpts.append([int(coords[0]/2 + temp[0]/2), int(coords[1]/2 + temp[1]/2)])
            temp = coords
            index += 1

        img, contours, hierarchy = cv2.findContours(paths, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        route_1_contours = []
        route_2_contours = []

        for cnt in contours:
            moment=cv2.moments(cnt)
            cx = int(moment['m10'] / moment['m00'])
            cy = int(moment['m01'] / moment['m00'])
            for pts in route_1_midpts:
                distance = (pts[0] - cx)**2 + (pts[1] - cy)**2
                if distance**(0.5) < 50:
                    route_1_contours.append(cnt)
            for pts in route_2_midpts:
                distance = (pts[0] - cx)**2 + (pts[1] - cy)**2
                if distance**(0.5) < 50:
                    route_2_contours.append(cnt)

        for cnt in route_1_contours:
            cv2.fillPoly(final_path_1, [cnt], 255)

        for cnt in route_2_contours:
            cv2.fillPoly(final_path_2, [cnt], 255)

        cv2.imshow("Track", track)                                                                  ### VISUALIZATION
        cv2.imshow("Path1", final_path_1)                                                           ### VISUALIZATION
        cv2.imshow("Path2", final_path_2)                                                           ### VISUALIZATION

    if cv2.waitKey(0) == 27:
        cv2.destroyAllWindows()
        rospy.signal_shutdown("shutdown")

if __name__ == '__main__':
    find = PathFinder()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospt.ROSInterruptException as e:
        print(e)
