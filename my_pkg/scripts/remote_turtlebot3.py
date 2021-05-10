#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from my_lib.GetChar import GetChar

LIN_MAX= 0.20
LIN_MIN= -0.20
LIN_STP= 0.01

ANG_MAX= 2.80 
ANG_MIN= -2.80
ANG_STP= 0.14


msg = """
---------------------------------------
              (forward)
                 'w'

  (left)'a'      's'       'd'(right)
              (backward)
---------------------------------------
type 'Q' for quit program...

"""


if __name__ == '__main__':
    
    rospy.init_node('remote_turtlebot3')

    pub  = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    tw   = Twist()
    rate = rospy.Rate(10)
    kb   = GetChar()

    tw.linear.x  = tw.linear.y  = tw.linear.z  = 0.0
    tw.angular.x = tw.angular.y = tw.angular.z = 0.0

    count = 0
    ch    = 0

    print msg

    while not rospy.is_shutdown():
        ch = kb.getch()

        if   ch == 'w':
            if (tw.linear.x + LIN_STP) <= LIN_MAX:
                tw.linear.x  =  tw.linear.x + LIN_STP
            else:
                tw.linear.x = LIN_MAX
            
        elif ch == 's': 
            if (tw.linear.x - LIN_STP) >= LIN_MIN:
                tw.linear.x  =  tw.linear.x - LIN_STP
            else:
                tw.linear.x = LIN_MIN

        elif ch == 'a':
            if (tw.angular.z + ANG_STP) <= ANG_MAX:
                tw.angular.z  =  tw.angular.z + ANG_STP
            else:
                tw.angular.z = ANG_MAX

        elif ch == 'd':
            if (tw.angular.z - ANG_STP) >= ANG_MIN:
                tw.angular.z  =  tw.angular.z - ANG_STP
            else:
                tw.angular.z = ANG_MIN

        elif ch ==' ':
            tw.linear.x = tw.angular.z = 0

        elif ch == 'Q':
            break

        else:
            pass

        pub.publish(tw)
        print "current linear.x = %s, angular.z = %s" %(tw.linear.x, tw.angular.z)

        count = count + 1
        if count == 10:
            count = 0
            print msg

        rate.sleep()

