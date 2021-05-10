#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

def cb_func(dat):
    txt=dat.pose.pose.orientation.z
    rospy.loginfo('position.z: %s', txt)

def simple_sub_pub():
    rospy.init_node('sample_sub_sub')#, anonymous=True)
    rospy.Subscriber('odom', nav_msgs.Odometry, cb_func)
    rospy.spin()

if __name__ == '__main__':
    simple_sub_pub() 
