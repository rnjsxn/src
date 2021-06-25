#!/usr/bin/env python

import rospy
from bebop_msgs.msg import Ardrone3GPSStateNumberOfSatelliteChanged

class NumOfSatellite:
    def __init__(self):
        rospy.init_node('num_of_satellite', anonymous = True)
        rospy.Subscriber('/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged',
                         Ardrone3GPSStateNumberOfSatelliteChanged,
                         self.get_num_sat_cb, queue_size = 1)
        self.num_sat = 0
              
    def get_num_sat_cb(self, msg):
        self.num_sat = msg.numberOfSatellite
        print "We can recieve gps signal from %s of satellites!" %(self.num_sat)
        
if __name__ == '__main__':
    try:
        NumOfSatellite()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
