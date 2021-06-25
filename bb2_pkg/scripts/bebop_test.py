#!/usr/bin/env python

import rospy, sys
from math import pow, degrees, radians, atan2, pi
from scipy import cos, sin, arctan, sqrt, arctan2
from haversine import haversine
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged, \
                           Ardrone3PilotingStateAttitudeChanged, \
                           Ardrone3PilotingStateAltitudeChanged, \
                           Ardrone3GPSStateNumberOfSatelliteChanged
from bb2_pkg.MoveBB2 import MoveBB2


USE_SPHINX = bool(int(sys.argv[1]))

OFFSET_LAT   = -13.645105
OFFSET_LON   = 126.715070

LIN_SPD      =   0.5
ANG_SPD      =   0.5

START_LATI   = 500.0
START_LONG   = 500.0

LAT_M        =   0.000009008
LON_M        =   0.000010972

class MoveByGPS:
    
    def __init__(self):
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/PositionChanged", 
                          Ardrone3PilotingStatePositionChanged, self.get_gps_cb)
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AttitudeChanged",
                          Ardrone3PilotingStateAttitudeChanged, self.get_atti_cb)
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",
                          Ardrone3PilotingStateAltitudeChanged, self.get_alti_cb)
        rospy.Subscriber("/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged", 
                          Ardrone3GPSStateNumberOfSatelliteChanged, self.get_num_sat_cb)
        self.pub1 = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 1)
        self.pub2 = rospy.Publisher('/bebop/land', Empty, queue_size = 1)
        
        self.currentGPS = self.startGPS = self.restartGPS = Ardrone3PilotingStatePositionChanged()
        
        self.empty_msg    = Empty()
                
        self.atti_current =   0.0        
        self.alti_current =   Ardrone3PilotingStateAltitudeChanged()
         
        self.start_lati   =   0.0
        self.start_long   =   0.0
        
        self.restart_lati =   0.0
        self.restart_long =   0.0
        
        self.tol_lati     =   LAT_M * 10.0
        self.tol_long     =   LON_M * 10.0
        
        self.target_dist  =   0.0
        self.target_atti  =   0.0
        
        self.num_sat      =   0
        
        self.is_there_enough_satellites = False
        self.is_gps_pos_of_home_saved   = False
        self.gps_pos_is_not_0_and_500   = False
        self.alti_flag = True

    def get_gps_cb(self, msg):  
        if USE_SPHINX is True:
            self.currentGPS.latitude  = msg.latitude  + OFFSET_LAT
            self.currentGPS.longitude = msg.longitude + OFFSET_LON
        else:
            self.currentGPS.latitude  = msg.latitude
            self.currentGPS.longitude = msg.longitude
            
        self.currentGPS.altitude = msg.altitude
        #print "(%s, %s)" %(self.currentGPS.latitude, self.currentGPS.longitude)
        
           
    def get_atti_cb(self, msg):
        self.atti_current = msg.yaw
        #print "%s" %(degrees(self.atti_current))
        
               
    def get_alti_cb(self, msg):
        self.alti_current = msg
        #print "%s" %(self.alti_current.altitude)
           
    def get_num_sat_cb(self, msg):
        self.num_sat = msg.numberOfSatellite
        
        if self.num_sat > 10:        
            self.is_there_enough_satellites = True
        else:
            self.is_there_enough_satellites = False        
        #print "number of satellites = %s" %(self.num_sat)

    def up_higher(self):
        print "up higher"
        pub0 = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
        tw = Twist()        
        while self.alti_flag == True:
            tw.linear.z = 3
            pub0.publish(tw)
            if self.alti_current.altitude >=3:
                self.alti_flag = False
                print "finish higher"
        print "current alti %s" %(self.alti_current.altitude)

    def bearing(self, (lat1, long1), (lat2, long2)):    
        Lat1,  Lat2  = radians(lat1),  radians(lat2) 
        Long1, Long2 = radians(long1), radians(long2)         
        y = sin(Long2-Long1)*cos(Lat2) 
        x = cos(Lat1)*sin(Lat2) - sin(Lat1)*cos(Lat2)*cos(Long2-Long1)         
        return atan2(y, x)  #return degrees(atan2(y, x))
        
        
    def save_starting_point(self):
        while self.currentGPS.latitude == 0.0 or self.currentGPS.latitude == 500.0:  pass
        self.startGPS = self.currentGPS
        print "gps coordination of starting point(%s, %s)" %(self.startGPS.latitude, self.startGPS.longitude)
        
        
    def save_restarting_point(self):
        while self.currentGPS.latitude == 0.0 or self.currentGPS.latitude == 500.0:  pass
        self.restartGPS = self.currentGPS
        #print "gps coordination of restarting point(%s, %s)" %(self.restartGPS.latitude, self.restartGPS.longitude)
        return (self.restartGPS.latitude, self.restartGPS.longitude)
            
    
    def rotate_to_target(self, target):    
        
        if   abs(self.atti_current - target) > radians(330):
            print "360 > case >= 330"
            tol = abs(self.atti_current - target) * 0.06625
        elif abs(self.atti_current - target) > radians(300):
            print "330 > case >= 300"
            tol = abs(self.atti_current - target) * 0.074875
        elif abs(self.atti_current - target) > radians(270):
            print "300 > case >= 270"
            tol = abs(self.atti_current - target) * 0.07875
        elif abs(self.atti_current - target) > radians(240):
            print "270 > case >= 240"
            tol = abs(self.atti_current - target) * 0.0925
        elif abs(self.atti_current - target) > radians(210):
            print "240 > case >= 210"
            tol = abs(self.atti_current - target) * 0.095
        elif abs(self.atti_current - target) > radians(180):
            print "210 > case >= 180"
            tol = abs(self.atti_current - target) * 0.125
        elif abs(self.atti_current - target) > radians(150):
            print "180 > case >= 150"
            tol = abs(self.atti_current - target) * 0.125
        elif abs(self.atti_current - target) > radians(120):
            print "150 > case >= 120"
            tol = abs(self.atti_current - target) * 0.125
        elif abs(self.atti_current - target) > radians(90):
            print "120 > case >=  90"
            tol = abs(self.atti_current - target) * 0.1775
        elif abs(self.atti_current - target) > radians(60):
            print " 90 > case >=  60"
            tol = abs(self.atti_current - target) * 0.275
        elif abs(self.atti_current - target) > radians(30):
            print " 60 > case >=  30"
            tol = abs(self.atti_current - target) * 0.4125
        else:
            print " 30 > case >=   0"
            tol = abs(self.atti_current - target) * 0.775
            
        tw  = Twist()
        pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
        
        print "current: %s, target: %s, tolerance: %s" %(self.atti_current, target, tol)
        
        if   self.atti_current >= 0 and target >= 0:
            '''                                   |     T             C         T
            <-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+->
            -180               -90                0                 90               180
            '''
            if   target - tol > self.atti_current:
                tw.angular.z = -ANG_SPD
                while target - tol > self.atti_current:
                    pub.publish(tw)
            elif self.atti_current - tol > target:
                tw.angular.z =  ANG_SPD
                while self.atti_current - tol > target:
                    pub.publish(tw)
            else:   pass
        
        elif self.atti_current >= 0 and target <  0:
            '''                     T             |         C
            <-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+->
            -180               -90                0                 90               180
            '''
            tw.angular.z =  ANG_SPD
            while self.atti_current - tol > target:
                pub.publish(tw)
        
        elif self.atti_current <  0 and target >= 0:
            '''                     C             |               T
            <-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+->
            -180               -90                0                 90               180
            '''
            tw.angular.z = -ANG_SPD
            while self.atti_current < target - tol:
                pub.publish(tw)
        
        elif self.atti_current <  0 and target <  0:
            '''   T           C         T         |
            <-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+->
            -180               -90                0                 90               180
            '''
            if   target - tol > self.atti_current:
                tw.angular.z = -ANG_SPD
                while target - tol > self.atti_current:
                    pub.publish(tw)
            elif self.atti_current - tol > target:
                tw.angular.z =  ANG_SPD
                while self.atti_current - tol > target:
                    pub.publish(tw)
            else:   pass
            
        else:   pass
        
        tw.angular.z = 0;   pub.publish(tw);    rospy.sleep(1.5)
        
        
    def move_to_target(self, target_lati, target_long):
    
        tw  = Twist()
        bb2 = MoveBB2()

        p1  = (self.restartGPS.latitude, self.restartGPS.longitude)
        p2  = (target_lati, target_long)
        
        pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
        
        count = 0
            
        while self.currentGPS.latitude  < (target_lati - self.tol_lati) or \
              self.currentGPS.latitude  > (target_lati + self.tol_lati) or \
              self.currentGPS.longitude < (target_long - self.tol_long) or \
              self.currentGPS.longitude > (target_long + self.tol_long):
            '''
            count = count + 1
            if count % 10000 == 0:
                count = 0;
                print "gps coordination of restarting point(%s, %s)" %(self.restartGPS.latitude, self.restartGPS.longitude)
            '''
            tw.linear.x = LIN_SPD * 1.5
        
            target_atti = self.bearing(p1, p2)
            
            if abs(self.atti_current - degrees(target_atti)) > 5:
                if self.atti_current - degrees(target_atti) > 5:
                    tw.angular.z =  ANG_SPD * 0.0055
                else:# self.atti_current - degrees(target_atti) < -5:
                    tw.angular.z = -ANG_SPD * 0.0055
            else:
                tw.angular.z = 0.0
            
            pub.publish(tw)
            rospy.sleep(0.3)
            p1 = self.save_restarting_point()    
            if (haversine(p1,p2)*1000) <=40:
                print("current dist: %s"%(haversine(p1,p2)*1000))                
                print("currentGPS %s : %s"%(self.currentGPS.latitude,self.currentGPS.longitude))
                print("restart: %s: %s"%(self.restartGPS.latitude, self.restartGPS.longitude))
                tw.linear.x=tw.angular.z=0.0
                pub.publish(tw)
                rospy.sleep(3)
                break
            else:
                pass
        target_acc = self.bearing(p1,p2)

        print "target_acc bearing = %s\n" %(degrees(target_acc))
        print "rotate_acc start from %s" %(degrees(self.atti_current))
        self.rotate_to_target(target_acc)
        print "rotate_acc end to %s\n" %(degrees(self.atti_current))
        tw.linear.x = tw.angular.z = 0.0;   pub.publish(tw); rospy.sleep(0.5)
        pp = haversine(p1,p2)*1000
        print "before pp : %s" %(pp)     
        bb2.move_x(pp, 0.125)
        
        rospy.sleep(1)
        print "finish pp"             

    def takeoff(self):
        self.pub1.publish(self.empty_msg);  print "takeoff";    rospy.sleep(0.5);
        self.pub1.publish(self.empty_msg);  rospy.sleep(3.0)
        
   
    def landing(self):
        self.pub2.publish(self.empty_msg);  print "landing"
        
                
if __name__ == '__main__':

    try:
        rospy.init_node("move_by_gps", anonymous=True); rospy.sleep(1.0)
        
        mbg = MoveByGPS();  mbg.takeoff()                
        mbg.up_higher()
        
        while mbg.is_there_enough_satellites == False:   pass
        print "number of available satellites is %s" %(mbg.num_sat)
        mbg.save_starting_point();  mbg.save_restarting_point()
        print "the GPS coordination (%s, %s)" %(mbg.startGPS.latitude, mbg.startGPS.longitude)
        print "is saved as position of homebase!!!"
        
        while mbg.atti_current == 0.0:  pass
        print "current heading is %s(deg)\n" %(degrees(mbg.atti_current))
        
        p1 = (mbg.restartGPS.latitude, mbg.restartGPS.longitude)
        print "p1(%s, %s)\n" %(mbg.restartGPS.latitude, mbg.restartGPS.longitude)
        
        target_lati = float(input("input latitude  of destination: "))
        target_long = float(input("input longitude of destination: "))
        
        p2 = (target_lati, target_long)
        print "\np2(%s, %s)\n" %(p2[0], p2[1])
        #print "\np2(%s, %s)\n" %(mbg.targetGPS.latitude, mbg.targetGPS.longitude)
        
        target_distance = haversine(p1, p2) * 1000
        target_attitude = mbg.bearing(p1, p2)
        
        print "target  bearing = %s\n" %(degrees(target_attitude))
        print "rotate start from %s" %(degrees(mbg.atti_current))
        mbg.rotate_to_target(target_attitude)
        print "rotate end to %s\n" %(degrees(mbg.atti_current))
        
        print "move to target start from (%s, %s)" %(p1[0], p1[1])
        mbg.move_to_target(p2[0], p2[1])
        print "move to target end to (%s, %s)\n" %(mbg.currentGPS.latitude, mbg.currentGPS.longitude)
        print "test gps p1 %s %s" %(p1[0],p1[1])
        print "test gps p2 %s %s" %(p2[0],p2[1])
        
        print "final target distance %s" %(haversine(p1,p2))

        home_p0 = (35.233795, 129.082850)
        home_p1 = (mbg.restartGPS.latitude, mbg.restartGPS.longitude)
        print "home restart : %s %s" %(home_p1[0],home_p1[1])
        if target_attitude > 0:
            home_bearing = target_attitude - pi
        else:
            home_bearing = pi + target_attitude    
        print "home bearing = %s\n" %(degrees(home_bearing))
        print "rotate(home) start from %s" %(degrees(mbg.atti_current))
        mbg.rotate_to_target(home_bearing)
        print "rotate(home) end to %s\n" %(degrees(mbg.atti_current))        
        
        print "move to home start from (%s, %s)" %(home_p1[0], home_p1[1])
        mbg.move_to_target(home_p0[0], home_p0[1])
        print "move to home end to (%s, %s)\n" %(mbg.currentGPS.latitude, mbg.currentGPS.longitude)
        print "final home distance %s" %(haversine(home_p0, home_p1))
        mbg.landing()
        rospy.spin()
            
    except rospy.ROSInterruptException: pass
'''         
                |<-- 100 m -->|<-- 100 m -->|
           --- p8------------p1-------------p2-> 35.234892 (35.233795+0.000900911)
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
           --- p6------------p5-------------p4-> 35.232698 (35.233795-0.000900911)
                v             v             v
             129.073840    129.082850    129.091859
             
     (129.082850-0.001097275)    (129.082850+0.001097275) 
     
        
        distance of latitude   1(deg) = 111011.0311340(m/deg)  p1( 35, 129) p2( 36, 129)
        distance of longtitude 1(deg) =  91134.8833075(m/deg)  p1( 35, 129) p2( 35, 130)
        
        -------------+----------------+-----------------
         Distance(m) |  latitude(deg) |  longitude(deg)
        -------------+----------------+-----------------
               1.0   |   0.000009008  |    0.000010972
              10.0   |   0.000090081  |    0.000109727
             100.0   |   0.000900911  |    0.001097275
        -------------+----------------+-----------------

        p0 = (35.233795, 129.082850)
        
        p1 = (35.234892, 129.082850);   p5 = (35.232698, 129.082850) 
        p2 = (35.234892, 129.091859);   p6 = (35.232698, 129.073840) 
        p3 = (35.233795, 129.091859);   p7 = (35.233795, 129.073840) 
        p4 = (35.232698, 129.091859);   p8 = (35.234892, 129.073840) 
'''
