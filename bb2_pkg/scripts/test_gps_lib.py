#!/usr/bin/env python

import rospy
from bb2_pkg.GPS import GPS
from haversine import haversine
'''-------------------------------------------------------------------------------------

                  longitude 1 degree 
                |<- 90075.833903 m -->| 
                |                     | (longitude)
    ----------  +---------p1----------+  36.444029
      ^         |         0|          |          input latitude  of Point1:  35.944029
      |         |          |          |          input longitude of Point1: 126.184297 --+
      |         |          |          |          input latitude  of Point2:  35.944029   |
      |         |          |          |          input longitude of Point2: 127.184297 --+
   latitude     |          |        90|     distance = 90075.293451, bearing = 89.706518
   1 degree    p4---------p0----------p2 35.444029
111016.503262 m |270      .|          |          input latitude  of Point1:  35.444029 --+
      |         |       .  |          |          input longitude of Point1: 126.684297   |
      |         |     .    |          |          input latitude  of Point2:  36.444029 --+
      |         |   .      |          |          input longitude of Point2: 126.684297
      v         | .        |180       |      distance = 111016.503262, bearing = 0.000018
    ---------- p5---------p3----------+   34.444029
(longuitude) 126.184297 126.684297 127.184297

   longitude 1.0000000 = 90075.833903(m)       latitude 1.000000000 = 111016.503262(m)
             0.1000000 =  9007.583390(m)                0.100000000 =  11101.650326(m)
             0.0100000 =   900.758339(m)                0.010000000 =   1110.165033(m)
             0.0010000 =    90.075834(m)                0.001000000 =    111.016503(m)
             0.0001000 =     9.007583(m)                0.000100000 =     11.101650(m)
             0.0000100 =     0.900758(m)                0.000010000 =     1.1101650(m)
             0.0000010 =     0.090076(m)                0.000001000 =     0.1110165(m)
             0.0000001 =     0.009008(m)                0.000000100 =     0.0111017(m)
               
--------------------------------------------------------------------------------------'''
if __name__ == '__main__':
    try:
        rospy.init_node('test_gps_lib', anonymous = True)
        
        gps = GPS()
        
        p0 = (35.444029, 126.684297)
        p1 = (36.444029, 126.684297)
        p2 = (35.444029, 127.184297)
        p3 = (34.444029, 126.684297)
        p4 = (35.444029, 126.184297)
        p5 = (36.444029, 126.184297)
        
        print "dist_gps = %s"   %(gps.get_distance(p0, p5))
        print "dist_hav = %s\n" %(haversine(p0, p5) * 1000.0)
        
        print "point1: %s"   %(gps.get_bearing(p0, p1))
        print "point2: %s"   %(gps.get_bearing(p0, p2))
        print "point3: %s"   %(gps.get_bearing(p0, p3))
        print "point4: %s\n" %(gps.get_bearing(p0, p4))
        
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
