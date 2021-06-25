#!/usr/bin/env python

import rospy, sys
from math import degrees, acos, sin, cos
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged, \
                           Ardrone3PilotingStateAttitudeChanged, \
                           Ardrone3PilotingStateAltitudeChanged, \
                           Ardrone3GPSStateNumberOfSatelliteChanged

USE_SPHINX = bool(int(sys.argv[1]))

OFFSET_LAT   = -13.645105
OFFSET_LON   = 126.715070

RADIAN = 0.0174532925
ANGLE = 57.2957795786
PI = 3.141592625
METER = 111189.57696
LIMITE_DIST = 2.5

class MoveByGPS:

    def __init__(self):

        rospy.Subscriber("/jw1/states/ardrone3/PilotingState/PositionChanged", 
                                Ardrone3PilotingStatePositionChanged, self.get_gps_cb)
        rospy.Subscriber("/jw1/states/ardrone3/PilotingState/AttitudeChanged",
                                Ardrone3PilotingStateAttitudeChanged, self.get_atti_cb)
        rospy.Subscriber("/jw1/states/ardrone3/PilotingState/AltitudeChanged",
                                Ardrone3PilotingStateAltitudeChanged, self.get_alti_cb)
        rospy.Subscriber("/jw1/states/ardrone3/GPSState/NumberOfSatelliteChanged", 
                                Ardrone3GPSStateNumberOfSatelliteChanged, self.get_num_sat_cb)

        rospy.Subscriber("/jw2/states/ardrone3/PilotingState/PositionChanged", 
                                Ardrone3PilotingStatePositionChanged, self.get_gps_cb)
        rospy.Subscriber("/jw2/states/ardrone3/PilotingState/AttitudeChanged",
                                Ardrone3PilotingStateAttitudeChanged, self.get_atti_cb)
        rospy.Subscriber("/jw2/states/ardrone3/PilotingState/AltitudeChanged",
                                Ardrone3PilotingStateAltitudeChanged, self.get_alti_cb)
        rospy.Subscriber("/jw2/states/ardrone3/GPSState/NumberOfSatelliteChanged", 
                                Ardrone3GPSStateNumberOfSatelliteChanged, self.get_num_sat_cb)

        rospy.Subscriber("/jw3/states/ardrone3/PilotingState/PositionChanged", 
                                Ardrone3PilotingStatePositionChanged, self.get_gps_cb)
        rospy.Subscriber("/jw3/states/ardrone3/PilotingState/AttitudeChanged",
                                Ardrone3PilotingStateAttitudeChanged, self.get_atti_cb)
        rospy.Subscriber("/jw3/states/ardrone3/PilotingState/AltitudeChanged",
                                Ardrone3PilotingStateAltitudeChanged, self.get_alti_cb)
        rospy.Subscriber("/jw3/states/ardrone3/GPSState/NumberOfSatelliteChanged", 
                                Ardrone3GPSStateNumberOfSatelliteChanged, self.get_num_sat_cb)


        self.pub0 = rospy.Publisher('/jw1/cmd_vel', Twist, queue_size = 1)
        self.pub1 = rospy.Publisher('/jw1/takeoff', Empty, queue_size = 1)
        self.pub2 = rospy.Publisher('/jw1/land', Empty, queue_size = 1)

        self.pub3 = rospy.Publisher('/jw2/cmd_vel', Twist, queue_size = 1)
        self.pub4 = rospy.Publisher('/jw2/takeoff', Empty, queue_size = 1)
        self.pub5 = rospy.Publisher('/jw2/land', Empty, queue_size = 1)

        self.pub6 = rospy.Publisher('/jw3/cmd_vel', Twist, queue_size = 1)
        self.pub7 = rospy.Publisher('/jw3/takeoff', Empty, queue_size = 1)
        self.pub8 = rospy.Publisher('/jw3/land', Empty, queue_size = 1)
            
        self.currentGPS = Ardrone3PilotingStatePositionChanged()
        self.currentGPS_s1 = Ardrone3PilotingStatePositionChanged()
        self.currentGPS_s2 = Ardrone3PilotingStatePositionChanged()

        self.num_sat = 0
        self.num_sat_s1 = 0
        self.num_sat_s2 = 0		

        self.saveGPS = [0,0,0,0,0,0]
        self.empty_msg    = Empty()

        self.atli_current = 0.0
        self.atti_current = 0.0

        self.atli_current_s1 = 0.0
        self.atti_current_s1 = 0.0

        self.atli_current_s2 = 0.0
        self.atti_current_s2 = 0.0

        self.xy = [0,0,0,0,0,0]
        self.latlon = [0,0,0,0,0,0]

        self.isflagsavegps = True
        self.is_there_enough_satellites = False
        

    def get_gps_cb(self, msg):  
        if USE_SPHINX is True:
            self.currentGPS.latitude  = msg.latitude  + OFFSET_LAT
            self.currentGPS.longitude = msg.longitude + OFFSET_LON

            self.currentGPS_s1.latitude  = msg.latitude  + OFFSET_LAT
            self.currentGPS_s1.longitude = msg.longitude + OFFSET_LON

            self.currentGPS_s2.latitude  = msg.latitude  + OFFSET_LAT
            self.currentGPS_s2.longitude = msg.longitude + OFFSET_LON
        else:
            self.currentGPS.latitude  = msg.latitude
            self.currentGPS.longitude = msg.longitude
                    
            self.currentGPS.altitude = msg.altitude

            self.currentGPS_s1.latitude  = msg.latitude
            self.currentGPS_s1.longitude = msg.longitude
                    
            self.currentGPS_s1.altitude = msg.altitude

            self.currentGPS_s2.latitude  = msg.latitude
            self.currentGPS_s2.longitude = msg.longitude
                    
            self.currentGPS_s2.altitude = msg.altitude
            #print "(%s, %s)" %(self.currentGPS.latitude, self.currentGPS.longitude)
							
    def get_atti_cb(self, msg):
        self.atti_current = msg.yaw
        self.atti_current_s1 = msg.yaw
        self.atti_current_s2 = msg.yaw
        #print "%s" %(degrees(self.atti_current))

                
    def get_alti_cb(self, msg):
        self.atli_current = msg.altitude
        self.atli_current_s1 = msg.altitude
        self.atli_current_s2 = msg.altitude
                    
    def get_num_sat_cb(self, msg):
        self.num_sat = msg.numberOfSatellite
        self.num_sat_s1 = msg.numberOfSatellite
        self.num_sat_s2 = msg.numberOfSatellite

        if self.num_sat > 10 and self.num_sat_s1 > 10:        
            self.is_there_enough_satellites = True
        else:
            self.is_there_enough_satellites = False        
            #print "number of satellites = %s" %(self.num_sat)

    def save_starting_point(self):
        while self.currentGPS.latitude == 0.0 or self.currentGPS.latitude == 500.0:  pass
        self.startGPS = self.currentGPS
        while self.currentGPS_s1.latitude == 0.0 or self.currentGPS_s1.latitude == 500.0:  pass
        self.startGPS_s1 = self.currentGPS_s1
        #print "gps coordination of starting point(%s, %s)" %(self.startGPS.latitude, self.startGPS.longitude)
        if self.isflagsavegps == True:
            self.saveGPS[0] = self.currentGPS.latitude
            self.saveGPS[1] = self.currentGPS.longitude
            self.saveGPS[2] = self.currentGPS.latitude
            self.saveGPS[3] = self.currentGPS.longitude
            self.saveGPS[4] = self.currentGPS.latitude
            self.saveGPS[5] = self.currentGPS.longitude
            print(self.saveGPS[0], self.saveGPS[1], self.saveGPS[2], self.saveGPS[3],self.saveGPS[4],self.saveGPS[5])
            self.isflagsavegps = False

    def converttoxy(self):
        x = (self.currentGPS.longitude - self.saveGPS[1]) * 91117
        y = (self.currentGPS.latitude - self.saveGPS[0]) * 110943

        x_1 = (self.currentGPS_s1.longitude - self.saveGPS[3]) * 91117
        y_1 = (self.currentGPS_s1.latitude - self.saveGPS[2]) * 110943

        x_2 = (self.currentGPS_s1.longitude - self.saveGPS[5]) * 91117
        y_2 = (self.currentGPS_s1.latitude - self.saveGPS[4]) * 110943

        self.xy[0] = x 
        self.xy[1] = y

        self.xy[2] = x_1 + 5
        self.xy[3] = y_1 + 5

        self.xy[4] = x_2 - 5
        self.xy[5] = y_2 - 5

        return self.xy

    def converttolatlon(self):
        longitude = self.xy[0] / 91117 + self.saveGPS[1] 
        latitude = self.xy[1] / 110943 + self.saveGPS[0]

        self.latlon[0] = longitude
        self.latlon[1] = latitude

        longitude_s1 = self.xy[2] / 91117 + self.saveGPS[3] 
        latitude_s1 = self.xy[3] / 110943 + self.saveGPS[2]

        self.latlon[2] = longitude_s1
        self.latlon[3] = latitude_s1

        longitude_s2 = self.xy[4] / 91117 + self.saveGPS[5] 
        latitude_s2 = self.xy[5] / 110943 + self.saveGPS[4]

        self.latlon[4] = longitude_s2
        self.latlon[5] = latitude_s2

        return self.latlon

    def move_to_target(self, g_lati, g_long, g_alti, speed):
        g_lati_radian = g_lati * RADIAN
        g_long_radian = g_long * RADIAN

        while not rospy.is_shutdown():
            linXYZ_AngZ = [0,0,0,0]
            linXYZ_AngZ_s1 = [0,0,0,0]
            linXYZ_AngZ_s2 = [0,0,0,0]
            
            self.converttoxy()
            self.converttolatlon()

            c_lati_radian = self.latlon[1] * RADIAN
            c_long_radian = self.latlon[0] * RADIAN

            c_lati_radian_s1 = self.latlon[3] * RADIAN
            c_long_radian_s1 = self.latlon[2] * RADIAN

            c_lati_radian_s2 = self.latlon[5] * RADIAN
            c_long_radian_s2 = self.latlon[4] * RADIAN

            dist_rad = acos(sin(c_lati_radian) * sin(g_lati_radian) + cos(c_lati_radian)
                            * cos(g_lati_radian) * cos(c_long_radian - g_long_radian))

            dist_rad_s1 = acos(sin(c_lati_radian_s1) * sin(g_lati_radian) + cos(c_lati_radian_s1)
                            * cos(g_lati_radian) * cos(c_long_radian_s1 - g_long_radian))

            dist_rad_s2 = acos(sin(c_lati_radian_s2) * sin(g_lati_radian) + cos(c_lati_radian_s2)
                            * cos(g_lati_radian) * cos(c_long_radian_s2 - g_long_radian))

            dist = dist_rad * ANGLE * METER - (speed * LIMITE_DIST)
            dist_s1 = dist_rad_s1 * ANGLE * METER - (speed * LIMITE_DIST)
            dist_s2 = dist_rad_s2 * ANGLE * METER - (speed * LIMITE_DIST)

            if dist < LIMITE_DIST or dist_s2 < LIMITE_DIST:
                break           

            radian = acos((sin(g_lati_radian) - sin(c_lati_radian) * cos(dist_rad))
                        / (cos(c_lati_radian) * sin(dist_rad)))

            radian_s1 = acos((sin(g_lati_radian) - sin(c_lati_radian_s1) * cos(dist_rad_s1))
                        / (cos(c_lati_radian_s1) * sin(dist_rad_s1)))

            radian_s2 = acos((sin(g_lati_radian) - sin(c_lati_radian_s2) * cos(dist_rad_s2))
                        / (cos(c_lati_radian_s2) * sin(dist_rad_s2)))

            c_bearing = self.atti_current * ANGLE
            c_bearing_s1 = self.atti_current_s1 * ANGLE
            c_bearing_s2 = self.atti_current_s2 * ANGLE


            if c_bearing <=0:
                c_bearing = 360 - abs(c_bearing)
            if c_bearing >= 355:
                c_bearing = 355
            if c_bearing <= 5:
                c_bearing = 5

            if c_bearing_s1 <=0:
                c_bearing_s1 = 360 - abs(c_bearing_s1)
            if c_bearing_s1 >= 355:
                c_bearing_s1 = 355
            if c_bearing_s1 <= 5:
                c_bearing_s1 = 5

            if c_bearing_s2 <=0:
                c_bearing_s2 = 360 - abs(c_bearing_s2)
            if c_bearing_s2 >= 355:
                c_bearing_s2 = 355
            if c_bearing_s2 <= 5:
                c_bearing_s2 = 5

            true_bearing = radian * ANGLE
            true_bearing_s1 = radian_s1 * ANGLE
            true_bearing_s2 = radian_s2 * ANGLE

            if g_long < self.latlon[0]:
                true_bearing = 360 - true_bearing
            if true_bearing >= 355:
                true_bearing = 355
            if true_bearing <= 5:
                true_bearing = 5

            if g_long < self.latlon[2]:
                true_bearing_s1 = 360 - true_bearing_s1
            if true_bearing_s1 >= 355:
                true_bearing_s1 = 355
            if true_bearing_s1 <= 5:
                true_bearing_s1 = 5

            if g_long < self.latlon[4]:
                true_bearing_s2 = 360 - true_bearing_s2
            if true_bearing_s2 >= 355:
                true_bearing_s2 = 355
            if true_bearing_s2 <= 5:
                true_bearing_s2 = 5

            if self.atli_current < g_alti - 0.3:
                linXYZ_AngZ[2] = 0.3
            elif self.atli_current > g_alti + 0.3:
                linXYZ_AngZ[2] = -0.3
            else:
                if true_bearing < c_bearing + 15 and true_bearing > c_bearing - 15:
                    linXYZ_AngZ[0] = speed

                    if true_bearing <= c_bearing + 15 and true_bearing >= c_bearing + 5:
                        linXYZ_AngZ[1] =  - (speed / 3)
                        linXYZ_AngZ[3] =  - (speed / 2)
                    elif true_bearing >= c_bearing - 15 and true_bearing <= c_bearing - 5:
                        linXYZ_AngZ[1] = speed / 3
                        linXYZ_AngZ[3] = speed / 2
                else:
                    if true_bearing < c_bearing:
                        linXYZ_AngZ[3] = speed
                    elif true_bearing > c_bearing:
                        linXYZ_AngZ[3] = - (speed)

            if self.atli_current_s1 < g_alti - 0.3:
                linXYZ_AngZ_s1[2] = 0.3
            elif self.atli_current_s1 > g_alti + 0.3:
                linXYZ_AngZ_s1[2] = -0.3
            else:
                if true_bearing_s1 < c_bearing_s1 + 15 and true_bearing_s1 > c_bearing_s1 - 15:
                    linXYZ_AngZ_s1[0] = speed

                    if true_bearing_s1 <= c_bearing_s1 + 15 and true_bearing_s1 >= c_bearing_s1 + 5:
                        linXYZ_AngZ_s1[1] =  - (speed / 3)
                        linXYZ_AngZ_s1[3] =  - (speed / 2)
                    elif true_bearing_s1 >= c_bearing_s1 - 15 and true_bearing_s1 <= c_bearing_s1 - 5:
                        linXYZ_AngZ_s1[1] = speed / 3
                        linXYZ_AngZ_s1[3] = speed / 2
                else:
                    if true_bearing_s1 < c_bearing_s1:
                        linXYZ_AngZ_s1[3] = speed
                    elif true_bearing_s1 > c_bearing_s1:
                        linXYZ_AngZ_s1[3] = - (speed)

            if self.atli_current_s2 < g_alti - 0.3:
                linXYZ_AngZ_s2[2] = 0.3
            elif self.atli_current_s2 > g_alti + 0.3:
                linXYZ_AngZ_s2[2] = -0.3
            else:
                if true_bearing_s2 < c_bearing_s2 + 15 and true_bearing_s2 > c_bearing_s2 - 15:
                    linXYZ_AngZ_s2[0] = speed

                    if true_bearing_s2 <= c_bearing_s2 + 15 and true_bearing_s2 >= c_bearing_s2 + 5:
                        linXYZ_AngZ_s2[1] =  - (speed / 3)
                        linXYZ_AngZ_s2[3] =  - (speed / 2)
                    elif true_bearing_s2 >= c_bearing_s2 - 15 and true_bearing_s2 <= c_bearing_s2 - 5:
                        linXYZ_AngZ_s2[1] = speed / 3
                        linXYZ_AngZ_s2[3] = speed / 2
                else:
                    if true_bearing_s2 < c_bearing_s2:
                        linXYZ_AngZ_s2[3] = speed
                    elif true_bearing_s2 > c_bearing_s2:
                        linXYZ_AngZ_s2[3] = - (speed)

            twist = Twist()
            twist.linear.x = linXYZ_AngZ[0]
            twist.linear.y = linXYZ_AngZ[1]
            twist.linear.z = linXYZ_AngZ[2]
            twist.angular.z = linXYZ_AngZ[3]

            twist.linear.x = linXYZ_AngZ_s1[0]
            twist.linear.y = linXYZ_AngZ_s1[1]
            twist.linear.z = linXYZ_AngZ_s1[2]
            twist.angular.z = linXYZ_AngZ_s1[3]

            twist.linear.x = linXYZ_AngZ_s2[0]
            twist.linear.y = linXYZ_AngZ_s2[1]
            twist.linear.z = linXYZ_AngZ_s2[2]
            twist.angular.z = linXYZ_AngZ_s2[3]
            self.pub0.publish(twist)
            self.pub3.publish(twist)
            self.pub6.publish(twist)

            print("goal_m : %.2f , current : %.2f, dist : %.2f" %(true_bearing, c_bearing, dist))
            #print("goal_s1 : %.2f , current : %.2f, dist : %.2f" %(true_bearing_s1, c_bearing_s1, dist_s1))
            #print("goal_s2 : %.2f , current : %.2f, dist : %.2f" %(true_bearing_s2, c_bearing_s2, dist_s2))

    def takeoff(self):
        self.pub1.publish(self.empty_msg);  print "takeoff master";    rospy.sleep(0.5);
        self.pub1.publish(self.empty_msg);  rospy.sleep(3.0)

        self.pub4.publish(self.empty_msg);  print "takeoff slave1";    rospy.sleep(0.5);
        self.pub4.publish(self.empty_msg);  rospy.sleep(3.0)

        self.pub7.publish(self.empty_msg);  print "takeoff slave2";    rospy.sleep(0.5);
        self.pub7.publish(self.empty_msg);  rospy.sleep(3.0)


    def landing(self):
        self.pub2.publish(self.empty_msg);  print "landing master"
        self.pub5.publish(self.empty_msg);  print "landing slave1"
        self.pub8.publish(self.empty_msg);  print "landing slave2"

if __name__ == '__main__':
    try:
        rospy.init_node("move_by_gps", anonymous=True); rospy.sleep(1.0)
        mbg = MoveByGPS();  mbg.takeoff()
        while mbg.is_there_enough_satellites == False:  pass
        print "number of available satellites is %s" %(mbg.num_sat)
        mbg.save_starting_point()
        print "the GPS coordination master (%s, %s)" %(mbg.saveGPS[0], mbg.saveGPS[1])
        print "the GPS coordination slave1 (%s, %s)" %(mbg.saveGPS[2], mbg.saveGPS[3])
        print "the GPS coordination slave2 (%s, %s)" %(mbg.saveGPS[4], mbg.saveGPS[5])
        print "is saved as position of homebase!!!"

        while mbg.atti_current == 0.0 and mb.atti_current_s1 == 0.0:  pass	
                
        target_lati = float(input("input latitude  of destination: "))        
        target_long = float(input("input longitude of destination: "))
        target_alti = float(input("input altitude of destination: "))
        target_speed = float(input("input speed of destination: "))
        mbg.move_to_target(target_lati, target_long, target_alti, target_speed)

        #mbg.landing()
            
        rospy.spin()

    except rospy.ROSInterruptException: pass
