#!/usr/bin/env python

import rospy
from bb2_pkg.MoveBB2 import MoveBB2
from math import radians, degrees

if __name__ == '__main__':

    rospy.init_node('bb2_sub_odom', anonymous = True)
    
    try:
        bb2 = MoveBB2()
        
        # bb2.takeoff();  rospy.sleep(3.0)      
        
        dist_z = float(input("input distance to move to z in meter: "))
        bb2.move_z(dist_z, 0.1)
        
        angle  = radians(float(input("input angle to rotate in degree: ")))
        bb2.rotate(angle,  0.1)
        
        dist_x = float(input("input distance to move to x in meter: "))
        bb2.move_x(dist_x, 0.1)
        
        dist_y = float(input("input distance to move to y in meter: "))
        bb2.move_y(dist_y, 0.1)        
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        bb2.landing()
