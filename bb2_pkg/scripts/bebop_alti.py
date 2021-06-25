#!/usr/bin/env python

import rospy
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class SubAlti:

    def __init__(self):
        rospy.init_node('get_altitude', anonymous = True)

        self.pub0 = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
        self.pub1 = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 1)
        self.pub2 = rospy.Publisher('/bebop/land',    Empty, queue_size = 1)

        self.empty_msg    = Empty()
        self.tw_msg       = Twist()

        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",
                          Ardrone3PilotingStateAltitudeChanged, self.get_alti_cb)

        self.alti = Ardrone3PilotingStateAltitudeChanged()
        self.alti_flag = True
      
    def get_alti_cb(self, msg):
        self.alti = msg
        print "altitude = %s" %(self.alti.altitude)

    def up_higher(self):
        print "up higher"
        while self.alti_flag == True:
            self.tw_msg.linear.z = 3
            self.pub0.publish(self.tw_msg)
            if self.alti.altitude >= 20:
                self.alti_flag = False
                print "finish higher"
                break

    def takeoff(self):
        self.pub1.publish(self.empty_msg);  print "takeoff";    rospy.sleep(0.5);
        self.pub1.publish(self.empty_msg);  rospy.sleep(3.0)

    def landing(self):
        rospy.sleep(2.0)
        self.pub2.publish(self.empty_msg);  print "landing"

if __name__ == '__main__':
    try:
        alti = SubAlti()
        alti.takeoff()
        alti.up_higher()
        alti.landing()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
