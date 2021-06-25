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

		rospy.Subscriber("/bebop/states/ardrone3/PilotingState/PositionChanged", 
				                  Ardrone3PilotingStatePositionChanged, self.get_gps_cb)
		rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AttitudeChanged",
				                  Ardrone3PilotingStateAttitudeChanged, self.get_atti_cb)
		rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",
				                  Ardrone3PilotingStateAltitudeChanged, self.get_alti_cb)
		rospy.Subscriber("/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged", 
				                  Ardrone3GPSStateNumberOfSatelliteChanged, self.get_num_sat_cb)

		self.pub0 = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
		self.pub1 = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 1)
		self.pub2 = rospy.Publisher('/bebop/land', Empty, queue_size = 1)
				
		self.currentGPS = self.startGPS = self.restartGPS = Ardrone3PilotingStatePositionChanged()

		self.empty_msg    = Empty()
		self.atli_current = 0.0
		self.atti_current = 0.0
		self.atli_current = 0.0

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
		self.atli_current = msg.altitude
                   
	def get_num_sat_cb(self, msg):
		self.num_sat = msg.numberOfSatellite
        
		if self.num_sat > 10:        
			self.is_there_enough_satellites = True
		else:
			self.is_there_enough_satellites = False        
			#print "number of satellites = %s" %(self.num_sat)

	def save_starting_point(self):
		while self.currentGPS.latitude == 0.0 or self.currentGPS.latitude == 500.0:  pass
		self.startGPS = self.currentGPS
		print "gps coordination of starting point(%s, %s)" %(self.startGPS.latitude, self.startGPS.longitude)

	def save_restarting_point(self):
		while self.currentGPS.latitude == 0.0 or self.currentGPS.latitude == 500.0:  pass
		self.restartGPS = self.currentGPS
		#print "gps coordination of restarting point(%s, %s)" %(self.restartGPS.latitude, self.restartGPS.longitude)
		return (self.restartGPS.latitude, self.restartGPS.longitude)

	def move_to_target(self, g_lati, g_long, g_alti, speed):
		g_lati_radian = g_lati * RADIAN
		g_long_radian = g_long * RADIAN

		while not rospy.is_shutdown():
			linXYZ_AngZ = [0,0,0,0]
			
			c_lati_radian = self.currentGPS.latitude * RADIAN
			c_long_radian = self.currentGPS.longitude * RADIAN
	
			dist_rad = acos(sin(c_lati_radian) * sin(g_lati_radian) + cos(c_lati_radian)
                          * cos(g_lati_radian) * cos(c_long_radian - g_long_radian))

			dist = dist_rad * ANGLE * METER - (speed * LIMITE_DIST)

			if dist < LIMITE_DIST:
				break

			radian = acos((sin(g_lati_radian) - sin(c_lati_radian) * cos(dist_rad))
                        / (cos(c_lati_radian) * sin(dist_rad)))

			c_bearing = self.atti_current * ANGLE
			
			if c_bearing <=0:
				c_bearing = 360 - abs(c_bearing)
			if c_bearing >= 355:
				c_bearing = 355
			if c_bearing <= 5:
				c_bearing = 5

			true_bearing = radian * ANGLE
			if g_long < self.currentGPS.longitude:
				true_bearing = 360 - true_bearing
			if true_bearing >= 355:
				true_bearing = 355
			if true_bearing <= 5:
				true_bearing = 5

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

			twist = Twist()
			twist.linear.x = linXYZ_AngZ[0]
			twist.linear.y = linXYZ_AngZ[1]
			twist.linear.z = linXYZ_AngZ[2]
			twist.angular.z = linXYZ_AngZ[3]
			self.pub0.publish(twist)

			print("goal : %.2f , current : %.2f, dist : %.2f" %(true_bearing, c_bearing, dist))

	def takeoff(self):
		self.pub1.publish(self.empty_msg);  print "takeoff";    rospy.sleep(0.5);
		self.pub1.publish(self.empty_msg);  rospy.sleep(3.0)
        
   
	def landing(self):
		self.pub2.publish(self.empty_msg);  print "landing"

if __name__ == '__main__':
	try:
		rospy.init_node("move_by_gps", anonymous=True); rospy.sleep(1.0)
		mbg = MoveByGPS();  mbg.takeoff()
		while mbg.is_there_enough_satellites == False:  pass
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
		target_alti = float(input("input altitude of destination: "))
		target_speed = float(input("input speed of destination"))

		mbg.move_to_target(target_lati, target_long, target_alti, target_speed)

		rospy.spin()

	except rospy.ROSInterruptException: pass
