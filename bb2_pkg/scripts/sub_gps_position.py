#!/usr/bin/env python

import rospy
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged
from darknet_ros_msgs.msg import ObjectCount

class detectObj:

    def __init__(self):
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/PositionChanged', \
                         Ardrone3PilotingStatePositionChanged, get_gps_cb)
        rospy.Subscriber('/darknet_ros/found_object', ObjectCount, found_obj_cb)
        rospy.init_node('sub_bb2_gps')#, anonymous=True)
        
        self.gps_info = Ardrone3PilotingStatePositionChanged()
        self.obj_cnt  = ObjectCount()
        

    def get_gps_cb(msg):
        self.gps_info = msg
        

    def found_obj_cb(msg):
        self.obj_cnt  = msg
        print "(count: %s)" %(self.obj_cnt.count)
        
        if self.obj_cnt.count > 0:
            print "(%s, %s)" %(self.gps_info.latitude, self.gps_info.longitude)
            



if __name__ == '__main__':
    rospy.init_node('found_obj_gps')#, anonymous=True)
    
    rospy.spin()
