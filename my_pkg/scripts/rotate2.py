#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import degrees, radians, pi

SPEED = 0.5

class Rotate2:

    def __init__(self):        
        rospy.init_node('sub_turtle_pose')
        rospy.Subscriber('/turtle1/pose', Pose, self.get_pose)

        self.th = 0


    def get_pose(self, dat):
        self.th = round(dat.theta, 2)
    
    def get_goal(self, angle):
        
        th_start = self.th
        th_goal  = th_start + angle
        '''
        if th_goal > 2 * pi:
            return th_goal - 2 * pi
        '''    
        return th_goal
        

    def rotate2(self, angle):

        pb     = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        tw     = Twist()
        
        goal   = self.get_goal(angle)
        
        th_start = self.th
        th_last = self.th 
        
        print "self.th = %s(deg), goal = %s(deg)" %(degrees(self.th), degrees(goal))

        if angle >= 0:
        
            tw.angular.z =  SPEED
            
            if goal >= 2 * pi:
            
                goal1 = 2 * pi
                goal2 = goal - 2 * pi
                
                while self.th <= goal1:
                    
                    if abs(th_last - self.th) > pi:
                        print "==="
                        break
                          
                    pb.publish(tw)                    
                    
                    th_last = self.th 
                    
                tw.angular.z = 0;   pb.publish(tw)
                print "step1 finished"
                
                while self.th <= goal2:                
                    pb.publish(tw)
                tw.angular.z = 0;   pb.publish(tw)
                print "step2 finished"
                
            else:
                while self.th <= goal:                
                    pb.publish(tw)
                tw.angular.z = 0;   pb.publish(tw)
        else: # angle < 0:
            if goal > -2 * pi:
                tw.angular.z = -SPEED  
                while self.th >= goal:
                    pb.publish(tw)
                tw.angular.z = 0;   pb.publish(tw)
            else:
                while self.th > -2 * pi:                
                    pb.publish(tw)
                tw.angular.z = 0;   pb.publish(tw)
                while self.th > -(abs(goal) - 2 * pi):                
                    pb.publish(tw)
                tw.angular.z = 0;   pb.publish(tw)
                
        print "th_now = %s" %(degrees(self.th))


if __name__ == '__main__':

    rt2 = Rotate2()
    ang = radians(float(input("input angle to turn from -180 ~ 180(deg): ")))
    rt2.rotate2(ang)
        
    rospy.spin()
