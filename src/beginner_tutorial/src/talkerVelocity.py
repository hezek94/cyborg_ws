#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('chatter', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = Twist()

    i = 0
    while not rospy.is_shutdown():
        msg.linear.x = 6*i
	msg.linear.y = 6*i+1
	msg.linear.z = 6*i+2

	msg.angular.x = 6*i+3
	msg.angular.y = 6*i+4
	msg.angular.z = 6*i+5

        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
	i = i + 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
