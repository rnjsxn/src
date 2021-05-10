#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from math import radians, degrees, pi, sqrt
from tf.transformations import euler_from_quaternion

LIN_SPD = 0.25
ANG_SPD = 0.5

class AllBB2:

    def __init__(self):
    
        rospy.init_node('bb2_All_by_odom', anonymous = True)
        
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
        self.prv_theta= 0.0
        self.theta_sum= 0.0
        self.theta_now= 0.0
        self.theta_org= 0.0
        self.rate = rospy.Rate(10)
        
        
    def get_odom_cb(self, dat):
        self.x_now = dat.pose.pose.position.x
        self.y_now = dat.pose.pose.position.y

        theta= self.get_theta(dat)

        if (theta-self.prv_theta) > 5.0:
            d_theta= (theta-self.prv_theta)-2*pi
        elif (theta-self.prv_theta) < 5.0:
            d_theta= (theta-self.prv_theta)+2*pi
        else:
            d_theta= (theta-self.prv_theta)

        self.theta_sum= self.theta_sum+d_theta
        self.prv_theta= theta

        self.theta_now= self.theta_sum

    def get_theta(self,msg):
        q= (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        quart= euler_from_quaternion(q)
        theta= quart[2]

        if theta < 0:
            theta= theta+pi*2

        if theta > pi*2:
            theta=theta-pi*2

        return theta

    def print_theta(self,angle):
        print("theta= %f = %f" %(angle,degress(angle)))
        
    def print_xy(self, angle):
        print("theta = %f = %f" %(angle, degrees(angle)))
        
    def update_org(self):
        # save current tb3pose.x, y to org.x, y when called this function
        self.x_org = self.x_now
        self.y_org = self.y_now
        self.theta_org= self.theta_now

    def elapsed_angle(self):
        return abs(self.theta_now - self.theta_org)
        
    def elapsed_dist(self):
        # calcurate and return elapsed distance
        return sqrt(pow((self.x_now - self.x_org), 2) + pow((self.y_now - self.y_org), 2))

    def rotate(self,angle,tolerance):
        tw= Twist()
        self.update_org()
        print "start from: %s" %(round(degrees(self.theta_org),2))

        if angle >= 0:
            tw.angular.z= ANG_SPD
        else:
            tw.angular.z= -ANG_SPD

        while self.elapsed_angle() < abs(angle)-abs(angle)*tolerance:
            self.pub0.publish(tw)

        tw.angular.z= 0
        self.pub0.publish(tw)
        rospy.sleep(1.5)
        print "stop to : %s" %(round(degrees(self.theta_now),2))
    
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
        bb2 = AllBB2()
        
        angle= radians(input("inTB3put angle to rotate(deg): "))
        distance  = float(input("input distanc to straight(m): "))
        tolerance = float(input("input tolerance(0.0 ~ 0.5)  : "))
        
        bb2.pub1.publish(bb2.empty_msg); rospy.sleep(3)
        
        bb2.all(angle,distance, tolerance)
        
        bb2.pub2.publish(bb2.empty_msg)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
