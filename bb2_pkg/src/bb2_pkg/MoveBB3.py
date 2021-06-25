#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from math import radians, degrees, pi, sqrt
from bb2_pkg.msg import Pos_XYZ_th

LIN_SPD = 0.125
ANG_SPD = 0.50

class MoveBB3:

    def __init__(self):
        rospy.Subscriber('/bb2_pose_odom', Pos_XYZ_th, self.get_pos_xyzth_cb)
        self.pub0 = rospy.Publisher('/dk1/cmd_vel', Twist, queue_size = 1)
        self.pub1 = rospy.Publisher('/dk1/takeoff', Empty, queue_size = 1)
        self.pub2 = rospy.Publisher('/dk1/land',    Empty, queue_size = 1)
        self.pub3 = rospy.Publisher('/dk1/reset',   Empty, queue_size = 1)

        self.pub4 = rospy.Publisher('/dk2/cmd_vel', Twist, queue_size = 1)
        self.pub5 = rospy.Publisher('/dk2/takeoff', Empty, queue_size = 1)
        self.pub6 = rospy.Publisher('/dk2/land',    Empty, queue_size = 1)
        self.pub7 = rospy.Publisher('/dk2/reset',   Empty, queue_size = 1)
        
        self.pub8 = rospy.Publisher('/dk3/cmd_vel', Twist, queue_size = 1)
        self.pub9 = rospy.Publisher('/dk3/takeoff', Empty, queue_size = 1)
        self.pub10 = rospy.Publisher('/dk3/land',    Empty, queue_size = 1)
        self.pub11 = rospy.Publisher('/dk3/reset',   Empty, queue_size = 1)

        self.empty_msg = Empty()
        self.xyzth_now = self.xyzth_org = Pos_XYZ_th()
        
        
    def get_pos_xyzth_cb(self, msg):
        self.xyzth_now = msg
        
        
    def print_xyzth(self, msg):
        print "x = %s, y = %s, z = %s, th = %s" %(msg.x, msg.y, msg.z, degrees(msg.th))
        
        
    def update_org(self):
        self.xyzth_org = self.xyzth_now
        
        
    def elapsed_dist(self):
        return sqrt(pow((self.xyzth_now.x - self.xyzth_org.x), 2) + pow((self.xyzth_now.y - self.xyzth_org.y), 2))
        
        
    def elapsed_angle(self):
        return abs(self.xyzth_now.th - self.xyzth_org.th)
        
        
    def elapsed_height(self):
        return abs(self.xyzth_now.z - self.xyzth_org.z)
        
        
    def move_x(self, distance, tolerance):
        tw = Twist()
        
        if distance >= 0:   # distance(+): forward
            tw.linear.x =  LIN_SPD
        else:               # distance(-): backward
            tw.linear.x = -LIN_SPD
            
        self.update_org()   # update starting point
        print "start at (%s, %s)" %(round(self.xyzth_org.x, 2), round(self.xyzth_org.y, 2))
        
        while self.elapsed_dist() < abs(distance) - abs(distance) * tolerance:
            self.pub0.publish(tw)
        
        tw.linear.x = 0    
        self.pub0.publish(tw)
        self.pub4.publish(tw)
        self.pub8.publish(tw) # stop move
        rospy.sleep(2.0)
        print "stop  at (%s, %s)" %(round(self.xyzth_now.x, 2), round(self.xyzth_now.y, 2))
        
        
    def move_y(self, distance, tolerance):
        tw = Twist()
        
        if distance >= 0:   # distance(+): move left
            tw.linear.y =  LIN_SPD
        else:               # distance(-): move right
            tw.linear.y = -LIN_SPD
            
        self.update_org()   # update starting point
        print "start at (%s, %s)" %(round(self.xyzth_org.x, 2), round(self.xyzth_org.y, 2))
        
        while self.elapsed_dist() < abs(distance) - abs(distance) * tolerance:
            self.pub0.publish(tw)
        
        tw.linear.y = 0;    self.pub0.publish(tw) # stop move
        rospy.sleep(2.0)
        print "stop  at (%s, %s)" %(round(self.xyzth_now.x, 2), round(self.xyzth_now.y, 2))
        
        
    def move_z(self, height, tolerance):
        tw = Twist()
        
        if height >= 0:	# height(+): ascend
            tw.linear.z =  LIN_SPD
        else:			# height(-): descend
            tw.linear.z = -LIN_SPD
        
        self.update_org()
        print "start from: %s" %(round(self.xyzth_org.z, 2))
        
        while self.elapsed_height() < abs(height) - abs(height) * tolerance:
            self.pub0.publish(tw)
            self.pub4.publish(tw)
            self.pub8.publish(tw)
            
        tw.linear.z =  0  
        self.pub0.publish(tw) 
        self.pub4.publish(tw) 
        self.pub8.publish(tw) # stop move
        rospy.sleep(2.0)
        print "stop to   : %s" %(round(self.xyzth_now.z, 2))
        
        
    def rotate(self, angle, tolerance):
        tw = Twist()
        
        if angle >= 0:	# angle(+): rotate left(ccw)
            tw.angular.z =  ANG_SPD
        else:			# angle(-): rotate right(cw)
            tw.angular.z = -ANG_SPD
        
        self.update_org()
        print "start from: %s" %(round(degrees(self.xyzth_org.th), 2))
        
        while self.elapsed_angle() < abs(angle) - abs(angle) * tolerance:
            self.pub0.publish(tw)
            self.pub4.publish(tw)
            self.pub8.publish(tw)
            
        tw.angular.z =  0  
        self.pub0.publish(tw) 
        self.pub4.publish(tw)
        self.pub8.publish(tw) # stop move
        rospy.sleep(2.5)
        print "stop to   : %s" %(round(degrees(self.xyzth_now.th), 2))
        
    
    def takeoff(self):
        self.pub1.publish(self.empty_msg)
        self.pub5.publish(self.empty_msg)
        self.pub9.publish(self.empty_msg)
        print "takeoff"
        
   
    def landing(self):
        self.pub2.publish(self.empty_msg)  
        self.pub6.publish(self.empty_msg)
        self.pub10.publish(self.empty_msg)
        print "landing"
        
    
    def emergency(self):
        self.pub3.publish(self.empty_msg) 
        self.pub7.publish(self.empty_msg)
        self.pub11.publish(self.empty_msg)
        print "emergency"
