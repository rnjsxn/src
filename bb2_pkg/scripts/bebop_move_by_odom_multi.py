#!/usr/bin/env python

import rospy
from bb2_pkg.MoveBB3 import MoveBB3
from math import radians, degrees

angle1 = radians(float(input("input angle1 to rotate in degree: ")))
dist1 = float(input("input distance1 to move to x in meter: "))


if __name__ == '__main__':

    rospy.init_node('bb2_sub_odom', anonymous = True)
    
    try:
        bb2 = MoveBB3()
        dist_z = float(input("input distance to move to z in meter: "))
        bb2.move_z(dist_z, 0.1)

        while True:
        
        # bb2.takeoff();  rospy.sleep(3.0)   
            angle  = angle1
            bb2.rotate(angle,  0.1)
        
            dist_x = dist1
            bb2.move_x(dist_x, 0.1)

            angle  = -angle1
            bb2.rotate(angle,  0.1)

            dist_x = dist1
            bb2.move_x(dist_x, 0.1)

            angle  = angle1
            bb2.rotate(angle,  0.1)

            dist_x = dist1
            bb2.move_x(dist_x, 0.1)  

            angle = -angle1
            bb2.rotate(angle, 0.1)

            dist_x = dist1
            bb2.move_x(dist_x, 0.1)              
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        bb2.landing()
