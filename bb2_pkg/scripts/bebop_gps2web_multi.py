#!/usr/bin/env python

import rospy, sys
from std_msgs.msg import String
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged
from selenium import webdriver

USE_SPHINX = bool(int(sys.argv[1]))
'''
    GPS for center of map  (  35.233795,  129.082850 )
    Parot-Sphinx start GPS (  48.878900,    2.367780 )
    diffrence              ( -13.645105, +126.715070 )
'''
OFFSET_LAT = -13.645105
OFFSET_LON = 126.715070
drv = webdriver.Chrome(executable_path="/home/dk/chromedriver")

def get_gps_cb(msg):    
    if USE_SPHINX is True:
        latitude  = msg.latitude  + OFFSET_LAT
        longitude = msg.longitude + OFFSET_LON
    else:
        latitude  = msg.latitude; longitude = msg.longitude        
    print("latitude = %s, longitude = %s" %(latitude, longitude))
    drv.execute_script("update_gps(%s, %s)" %(str(latitude), str(longitude)))

if __name__ == '__main__':
    drv.get('http://localhost:8080')
    rospy.sleep(3)
    rospy.init_node("get_gps_position", anonymous=True)
    rospy.Subscriber("/dk1/states/ardrone3/PilotingState/PositionChanged", 
                     Ardrone3PilotingStatePositionChanged,
                     get_gps_cb)
    rospy.Subscriber("/dk2/states/ardrone3/PilotingState/PositionChanged", 
                     Ardrone3PilotingStatePositionChanged,
                     get_gps_cb)
    rospy.Subscriber("/dk3/states/ardrone3/PilotingState/PositionChanged", 
                     Ardrone3PilotingStatePositionChanged,
                     get_gps_cb)
    rospy.spin()
