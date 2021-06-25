#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Empty, Bool
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged
from bb2_pkg.GetChar import GetChar

class WayPoints:

    def __init__(self):
        rospy.init_node('waypoints_move', anonymous = True)
                         
        self.start = rospy.Publisher('/autoflight/start', String, queue_size = 1)
        self.pause = rospy.Publisher('/autoflight/pause', Empty, queue_size = 1)
        self.stop  = rospy.Publisher('/autoflight/stop', Empty, queue_size = 1)
        self.home  = rospy.Publisher('/autoflight/navigate_home', Bool, queue_size = 1)
                                     
        self.gps_location = Ardrone3PilotingStatePositionChanged()
        

if __name__ == '__main__':
    empty = Empty()
    try:
        wp = WayPoints()
        wp.start.publish("")
        rospy.sleep(5)
        wp.pause.publish(empty)
        rospy.sleep(2)
        wp.start.publish("")
        
        
    except rospy.ROSInterruptException:  pass

