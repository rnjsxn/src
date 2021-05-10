#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from bb2_pkg.GetChar import GetChar
 
msg = """
---------------------------------------------------
 1:take off, 2:landing, 3:emergency, sp:stop(hover)
---------------------------------------------------
        w                           i                      
   a    s    d                j     k     l
---------------------------------------------------
w/s : up  / down           i/k : foward / backword
a/d : ccw / cw             j/l : left   / righ
---------------------------------------------------
-/+ : decrease / increase linear  speed by 10%
,/. : decrease / increase angular speed by 10%
---------------------------------------------------
Type 'Q' to quit
"""

e = "Communications Failed"

# set direction for each move
moveBindings = {
    'w':( 0, 0, 0, 1), 'a':( 0, 0, 1, 0), 'i':( 1, 0, 0, 0), 'j':( 0, 1, 0, 0), 
    's':( 0, 0, 0,-1), 'd':( 0, 0,-1, 0), 'k':(-1, 0, 0, 0), 'l':( 0,-1, 0, 0),
    ' ':( 0, 0, 0, 0)
}

# '+', '-': for linear velocity / '>', '<': for angular velocity
speedBindings = {
#                   '+'             '<'             '>'
    '-':(0.9, 1.0), '=':(1.1, 1.0), ',':(1.0, 0.9), '.':(1.0, 1.1), '0':(1.0, 1.0)
}


class MoveBebop():

    def __init__(self):
    
        rospy.init_node('bebop_teleop_key')
        
        self.pub0 = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
        self.pub1 = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 1)
        self.pub2 = rospy.Publisher('/bebop/land',    Empty, queue_size = 1)
        self.pub3 = rospy.Publisher('/bebop/reset',   Empty, queue_size = 1)
        
        self.empty_msg = Empty()
        self.key_input = GetChar()
        
        self.lin_spd = rospy.get_param("~speed", 0.5)
        self.ang_spd = rospy.get_param("~turn",  1.0)
        
        self.x       =  0      # for linear.x
        self.y       =  0      # for linear.y
        self.z       =  0      # for angular.z
        self.th      =  0      # for linear.z
        self.count   =  0
        self.cnt4msg = 10      # print how2use every (cnt4msg)time
        
    def get_speed(self, lin, ang):
        return "current speed:\tlinear = %s, angular = %s " % (lin, ang)


if __name__ == '__main__': 
    try:
        mb = MoveBebop()
        
        print(msg)
        print(mb.get_speed(mb.lin_spd, mb.ang_spd))
        
        key = ' '
        
        while key != 'Q':   # while not rospy.is_shutdown():
        
            key = mb.key_input.getch()
        
            if   key == '1':
                mb.pub1.publish(mb.empty_msg);  mb.count = (mb.count + 1) % mb.cnt4msg; print "taking off"
                
            elif key == '2':
                mb.pub2.publish(mb.empty_msg);  mb.count = (mb.count + 1) % mb.cnt4msg; print "landing"
                
            elif key == '3':
                mb.pub3.publish(mb.empty_msg);  mb.count = (mb.count + 1) % mb.cnt4msg; print "emergency stop!!!"
                
            elif key in moveBindings.keys():
            
                mb.x  = moveBindings[key][0]
                mb.y  = moveBindings[key][1]
                mb.z  = moveBindings[key][2]
                mb.th = moveBindings[key][3]
                    
                if   mb.x ==  1:
                    print "forward"
                elif mb.x == -1:
                    print "backward"
                      
                if   mb.y ==  1:
                    print "move left"
                elif mb.y == -1:
                    print "move right"
                      
                if   mb.z ==  1:
                    print "turn left"
                elif mb.z == -1:
                    print "turn right"
                
                if   mb.th ==  1:
                    print "assending"
                elif mb.th == -1:
                    print "dessending"
            
                mb.count = (mb.count + 1) % mb.cnt4msg
                
            elif key in speedBindings.keys():
                mb.lin_spd = mb.lin_spd * speedBindings[key][0]
                mb.ang_spd = mb.ang_spd * speedBindings[key][1]
 
                print(mb.get_speed(mb.lin_spd, mb.ang_spd))
            
                mb.count = (mb.count + 1) % mb.cnt4msg
                
            else:            
                mb.count = (mb.count + 1) % mb.cnt4msg
            
            if (mb.count == 0):
                    print(msg)
 
            twist = Twist()
            
            twist.linear.x  = mb.x  * mb.lin_spd
            twist.linear.y  = mb.y  * mb.lin_spd
            twist.linear.z  = mb.th * mb.lin_spd
            
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = mb.z  * mb.ang_spd
            
            mb.pub0.publish(twist)
            
        mb.pub2.publish(mb.empty_msg)
        print "landing"
 
    except KeyboardInterrupt:   # rospy.ROSInterruptException:
        mb.pub2.publish(mb.empty_msg)
        print "landing"
