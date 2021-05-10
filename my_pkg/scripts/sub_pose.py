#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import degrees,radians,pi

SPEED = 1.0

class Rotate2:
    def __init__(self):
        rospy.init_node('sub_turtle_pose')
        rospy.Subscriber('turtle/pose', Pose, self.get_pose)
        #self.pub=rospy.Publisher('turtle1/cmd_vel',Twist)
        #self.tw=Twist()

        self.x= self.y=self.th=0

    def get_pose(self, dat):
        self.x = round(dat.x,2)
        self.y = round(dat.y,2)
        self.th = round(dat.theta,2)
        #rospy.loginfo("x = %s(m), y = %s(m), th = %s(deg)", self.x,self.y,degrees(self.th))

    def rotate2(self,angle):
        pb=rospy.Publisher('turtle1/cmd_vel',Twist, queue_size=10)
        tw= Twist()

        if angel < 0:
            speed= SPEED * -1
            while self.th >= engle:
                tw.angular.z= speed
                pb.publish(tw)
        else:
            speed = SPEED
            while self.th <= abs(engle):
                tw.angular.z= speed
                pb.publish(tw)

        tw.angular = 0
        pb.publish(tw)  

#    rospy.spin()

if __name__=='__main__':
    try:
        while not rospy.is)shutdown():
    rt2= Rotate2()

    ang= radians(float(input("input angel to turn(deg): ")))

    rt2.rotate2(ang)

    rospy.spin()
    