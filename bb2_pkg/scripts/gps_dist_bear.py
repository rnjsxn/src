#!/usr/bin/env python
import rospy
from math import pow, degrees, radians, atan2 
from scipy import cos, sin, arctan, sqrt, arctan2
from haversine import haversine
'''         
                |<-- 100(m)-->|<-- 100(m)-->|
           --- p8------------p1-------------p2-> 35.234694 (35.233795+0.0008993204)
            ^   | .-45        |0          . |
            |   |   .         |         . 45|
           100  |     .       |       .     |
           (m)  |       .     |     .       |
            |   |         .   |   .         |
            v   |-90        . | .           |
           --- p7------------p0-------------p3-> 35.233795
            ^   |           . | .         90|
            |   |         .   |   .         |
           100  |       .     |     .       |
           (m)  |     .       |       .     |
            |   -135.         |         .   |
            v   | .           |       135 . |
           --- p6------------p5-------------p4-> 35.232895 (35.233795-0.0008993204)
                v             v             v
             129.081752    129.082850    129.083947
             
     (129.082850-0.0010978720)    (129.082850+0.0010978720) 
     
        
        distance  of latitude   1(deg) = 111195.0802340(m/deg)  p1( 35, 129) p2( 36, 129)
        distance  of longtitude 1(deg) =  91085.2969372(m/deg)  p1( 35, 129) p2( 35, 130)
        latitude  of distance   1(m)   =      0.00000899320363720(deg/m)
        longitude of distance   1(m)   =      0.00001097872031629(deg/m)
        
        -------------+-----------------+-----------------
         Distance(m) |  latitude(deg)  |  longitude(deg)
        -------------+-----------------+-----------------
               1.0   |   0.0000089932  |   0.0000109787
              10.0   |   0.0000899320  |   0.0001097872
             100.0   |   0.0008993204  |   0.0010978720
        -------------+-----------------+-----------------

        p0 = (35.233795, 129.082850)
        
        p1 = (35.234694, 129.082850);   p5 = (35.232895, 129.082850) 
        p2 = (35.234694, 129.083947);   p6 = (35.232895, 129.081752) 
        p3 = (35.233795, 129.083947);   p7 = (35.233795, 129.081752) 
        p4 = (35.232895, 129.083947);   p8 = (35.234694, 129.081752) 
'''
def bearing((lat1, long1), (lat2, long2)):
    
    Lat1,  Lat2  = radians(lat1),  radians(lat2) 
    Long1, Long2 = radians(long1), radians(long2) 
    
    y = sin(Long2-Long1)*cos(Lat2) 
    x = cos(Lat1)*sin(Lat2) - sin(Lat1)*cos(Lat2)*cos(Long2-Long1) 
    
    return degrees(atan2(y, x))   
    
if __name__ == '__main__':
    try:
        rospy.init_node('get_distance_n_bearing_from_gps', anonymous = True)
        
        a = (35, 129);  b = (36, 129);  c = (35, 130)
        print "latitude  1(deg) is   %s(m)" %(haversine(a,b) * 1000)
        print "longitude 1(deg) is   %s(m)" %(haversine(a,c) * 1000)
        
        p0 = (35.233795, 129.082850)        
        p1 = (35.234694, 129.082850);   p5 = (35.232895, 129.082850) 
        p2 = (35.234694, 129.083947);   p6 = (35.232895, 129.081752) 
        p3 = (35.233795, 129.083947);   p7 = (35.233795, 129.081752) 
        p4 = (35.232895, 129.083947);   p8 = (35.234694, 129.081752) 
        
        print "p1: dist = %s(m),\tbearing = %s(deg)" %(haversine(p0,p1)*1000, bearing(p0,p1))
        print "p2: dist = %s(m),\tbearing = %s(deg)" %(haversine(p0,p2)*1000, bearing(p0,p2))
        print "p3: dist = %s(m),\tbearing = %s(deg)" %(haversine(p0,p3)*1000, bearing(p0,p3))
        print "p4: dist = %s(m),\tbearing = %s(deg)" %(haversine(p0,p4)*1000, bearing(p0,p4))
        print "p5: dist = %s(m),\tbearing = %s(deg)" %(haversine(p0,p5)*1000, bearing(p0,p5))
        print "p6: dist = %s(m),\tbearing = %s(deg)" %(haversine(p0,p6)*1000, bearing(p0,p6))
        print "p7: dist = %s(m),\tbearing = %s(deg)" %(haversine(p0,p7)*1000, bearing(p0,p7))
        print "p8: dist = %s(m),\tbearing = %s(deg)" %(haversine(p0,p8)*1000, bearing(p0,p8))
        
    except rospy.ROSInterruptException:  pass
