#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from math import degrees, pi
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

TARGET_ID = 13
MAX_SPEED =  0.22
LIM_NEAR  =  0.15
LIM_FAR   =  0.20


class AlignMarker:

    def __init__(self):
    
        rospy.init_node('align_to_marker', anonymous = True)        
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_marker)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        
        self.tw = Twist()
        self.tw.angular.z = MAX_SPEED / 4
        self.pub.publish(self.tw)
        
        self.theta_now   = 0
        self.theta_start = 0
        self.theta_end   = 0
        
        self.is_1st_find = True
        self.is_1st_lost = True
        
        
    def get_marker(self, msg):
          
        for msg in msg.markers:
        
            if msg.id == TARGET_ID:
                
                if self.is_1st_find == True:
                    self.theta_start = self.get_theta(msg)
                    self.is_1st_find = False
            
            else: # lost marker
                if self.is_1st_find == False and self.is_1st_lost == True:
                    self.theta_end = self.get_theta(msg)
                    self.is_1st_lost = False
                    
                    self.move2marker()
                    
                    
                    
                
    def get_theta(self, msg):
        
        """
        x --->  z (yaw  ) 
        y ---> -y (pitch) 
        z --->  x (roll ) 
        """
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
             
        quart = euler_from_quaternion(q)
        theta = quart[1]
        
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2

        return theta
                
                
    def move2marker(self):
        
        self.tw.angular.z = 0
        self.pub.publish(self.tw)
        
        target = (self.theta_start + self.theta_end) / 2
        
        self.tw.angular.z = -MAX_SPEED / 4        
        self.pub.publish(self.tw)
        
        while self.theta_now > target:  pass
        
        self.tw.angular.z = 0
        self.pub.publish(self.tw)
    
        
    #def print_pose(self, msg):
    #   print "x = %f, y = %f, theta = %f = %f" %(msg.x, msg.y, msg.theta, degrees(msg.theta))
          

if __name__ == '__main__':
    try:
        AlignMarker()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass