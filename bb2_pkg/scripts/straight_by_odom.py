#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from math import radians, degrees, pi, sqrt

LIN_SPD = 0.25

class StrightBB2:

    def __init__(self):
    
        rospy.init_node('bb2_Stright_by_odom', anonymous = True)
        
        rospy.Subscriber('/bebop/odom', Odometry, self.get_odom_cb )
        
        self.pub0 = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
        self.pub1 = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 1)
        self.pub2 = rospy.Publisher('/bebop/land',    Empty, queue_size = 1)
        self.pub3 = rospy.Publisher('/bebop/reset',   Empty, queue_size = 1)
        
        self.empty_msg = Empty()
        self.x_now = 0.0
        self.y_now = 0.0
        self.x_org = 0.0
        self.y_org = 0.0
        self.rate = rospy.Rate(10)
        
        
    def get_odom_cb(self, dat):
        self.x_now = dat.pose.pose.position.x
        self.y_now = dat.pose.pose.position.y
        
        
    def print_xy(self, angle):
        print("theta = %f = %f" %(angle, degrees(angle)))
        
    def update_org(self):
        # save current tb3pose.x, y to org.x, y when called this function
        self.x_org = self.x_now
        self.y_org = self.y_now
        
    def elapsed_dist(self):
        # calcurate and return elapsed distance
        return sqrt(pow((self.x_now - self.x_org), 2) + pow((self.y_now - self.y_org), 2))
    
    def straight(self, distance, tolerance):
        # forward or backward until elaped distance is equal to target distance
        tw = Twist()
        
        if distance >= 0:   # distance(+): forward
            tw.linear.x =  LIN_SPD
        else:               # distance(-): backward
            tw.linear.x = -LIN_SPD
            
        self.update_org()   # update starting point
        print("start at (%s, %s)" %(round(self.x_org, 2), round(self.y_org, 2)))
        
        while self.elapsed_dist() < abs(distance) - tolerance:
            self.pub0.publish(tw)
        
        tw.linear.x = 0;    self.pub0.publish(tw) # stop move
        rospy.sleep(2.0)
        print("stop  at (%s, %s)" %(round(self.x_now, 2), round(self.y_now, 2)))


if __name__ == '__main__':
    try:
        bb2 = StrightBB2()
        
        distance  = float(input("input distanc to straight(m): "))
        tolerance = float(input("input tolerance(0.0 ~ 0.5)  : "))
        
        bb2.pub1.publish(bb2.empty_msg); rospy.sleep(3)
        
        bb2.straight(distance, tolerance)
        
        bb2.pub2.publish(bb2.empty_msg)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
