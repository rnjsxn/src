#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Empty, Bool
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged
#from darknet_ros_msgs.msg import ObjectCount
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2
from bb2_pkg.GetChar import GetChar

class WayPoints:

    def __init__(self):
        rospy.init_node('waypoints_move'), #anonymous = True)
                         
        self.start = rospy.Publisher('/autoflight/start', String, queue_size = 1)
        self.pause = rospy.Publisher('/autoflight/pause', Empty, queue_size = 1)
        self.stop  = rospy.Publisher('/autoflight/stop', Empty, queue_size = 1)
        self.home  = rospy.Publisher('/autoflight/navigate_home', Bool, queue_size = 1)
                                     
        self.gps_location = Ardrone3PilotingStatePositionChanged()
        
        self.key_input = GetChar()

if __name__ == '__main__':
    empty = Empty()
    string = String()
    Bool = bool()
    try:
        wp = WayPoints()
        key = ' '
        while key != 'Q':
            key = mb.key_input.getch()

            if key == '1':
                wp.start.publish(string)
                m1 = key + string
                print(m1)
                #rospy.sleep(5)
            elif key == '2':
                wp.pause.publish(empty)
                m2 = key + empty
                print (m2)
                #rospy.sleep(2)
            elif key == '3':
                wp.start.publish(empty)
                m3 = key + empty
                print (m3)
            elif key == '4':
                wp.home.publish(Bool)
                m4 = key + Bool
                print (m4)
        
        
    except rospy.ROSInterruptException:  pass

