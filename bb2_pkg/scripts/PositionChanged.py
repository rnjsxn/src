#!/usr/bin/env python

import rospy
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged

class SubGPS:
    def __init__(self):
        rospy.init_node('get_gps_location', anonymous = True)
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/PositionChanged',
                         Ardrone3PilotingStatePositionChanged,
                         self.get_gps_cb, queue_size = 1)
        self.gps_pos = Ardrone3PilotingStatePositionChanged()
              
    def get_gps_cb(self, msg):
        self.gps_pos = msg
        print "lati = %s, long = %s" %(self.gps_pos.latitude, self.gps_pos.longitude)
        
if __name__ == '__main__':
    try:
        SubGPS()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
