#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import ObjectCount


def get_dark_cb(msg):
    print "(count: %s)" %(msg.count)
    

if __name__ == '__main__':
    rospy.init_node('sub_bb2_obj')
    rospy.Subscriber('/darknet_ros/found_object', ObjectCount, get_dark_cb)
    rospy.spin()
