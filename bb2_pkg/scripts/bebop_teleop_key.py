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
CTRL-C to quit
"""

e = "Communications Failed"
 
moveBindings = {
    'w':( 0, 0, 0, 1), 'a':( 0, 0, 1, 0), 'i':( 1, 0, 0, 0), 'j':( 0, 1, 0, 0), 
    's':( 0, 0, 0,-1), 'd':( 0, 0,-1, 0), 'k':(-1, 0, 0, 0), 'l':( 0,-1, 0, 0),
    ' ':( 0, 0, 0, 0)
}

speedBindings = {
    '-':(0.9,1.0), '+':(1.1,1.0), ',':(1.0,0.9), '.':(1.0,1.1), '0':(1.0,1.0)
}

 
def get_speed(lin, ang):
    return "current speed:\tlinear = %s, angular = %s " % (lin, ang)

 
if __name__=="__main__":
 
    pub0 = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
    pub1 = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
    pub2 = rospy.Publisher('bebop/land',    Empty, queue_size = 1)
    pub3 = rospy.Publisher('bebop/reset',   Empty, queue_size = 1)
    
    empty_msg = Empty()
    key_input = GetChar()
    
    rospy.init_node('bebop_teleop_key')
    
    lin_spd = rospy.get_param("~speed", 0.5)
    ang_spd = rospy.get_param("~turn",  1.0)
    
    x = 0;    y = 0;    z = 0;    th = 0;
    
    status = 0
 
    try:
        print(msg)
        print(get_speed(lin_spd, ang_spd))
        
        while not rospy.is_shutdown():
        
            key = key_input.getch()
        
            if key in moveBindings.keys():
                x  = moveBindings[key][0]
                y  = moveBindings[key][1]
                z  = moveBindings[key][2]
                th = moveBindings[key][3]
                
            elif key in speedBindings.keys():
                lin_spd = ang_spd * speedBindings[key][0]
                ang_spd = ang_spd * speedBindings[key][1]
 
                print(get_speed(lin_spd, ang_spd))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
                
            elif key == '1':
                pub1.publish(empty_msg)
                
            elif key == '2':
                pub2.publish(empty_msg)
                
            elif key == '3':
                pub3.publish(empty_msg)
                
            else:
                x  = 0;    y  = 0;    z  = 0;    th = 0
                if (key == '\x03'):
                    break
 
            twist = Twist()
            
            twist.linear.x  = x  * lin_spd
            twist.linear.y  = y  * lin_spd
            twist.linear.z  = th * lin_spd
            
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = z  * ang_spd
            
            pub0.publish(twist)


    except Exception as e:
        print(e)

 
    finally:
        twist = Twist()
        twist.linear.x  = 0; twist.linear.y  = 0; twist.linear.z  = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub0.publish(twist)

