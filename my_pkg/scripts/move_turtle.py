#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import radians
from my_lib.GetChar import GetChar


class MoveTurtle:

    def __init__(self):
        rospy.init_node('move_turtle_node')
        self.p = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.t = Twist()

        self.lin_speed = 1.5
        self.ang_speed = 1.5

        self.t.linear.x  = self.t.linear.y  = self.t.linear.z  = 0
        self.t.angular.x = self.t.angular.y = self.t.angular.z = 0

    def move(self, distance):

        speed = self.lin_speed

        if distance < 0:
            speed = speed * -1

        self.t.linear.x = speed

        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        while(current_distance < distance):
            self.p.publish(self.t)
            t1 = rospy.Time.now().to_sec()
            current_distance= speed * (t1 - t0)
        
        print "end move"
        self.t.linear.x = 0
        self.p.publish(self.t)


    def rotate(self, angle):

        speed  = self.ang_speed

        if angle < 0:
            speed = speed * -1

        self.t.angular.z = speed

        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < angle):
            self.p.publish(self.t)
            t1 = rospy.Time.now().to_sec()
            current_angle = speed * (t1 - t0)
        
        print "end rotate"
        self.t.angular.z = 0
        self.p.publish(self.t)


if __name__ == "__main__":

    mt = MoveTurtle()
        
    dist = int(input("distance(m) = "))
    angl = radians(int(input("angle (deg) = ")))

    mt.rotate(angl)
    mt.move(dist)

